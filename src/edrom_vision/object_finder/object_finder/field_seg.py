#!/usr/bin/env python3
"""
Segmentacao CLASSICA (sem rede neural) de campo e linhas para RoboCup.

Complementa um detector de objetos (ex.: yolov26n-edrom.pt): a rede acha
bola/trave/robo e este modulo acha campo, borda do campo e linhas em poucos
milissegundos de CPU. Mesma ideia dos pipelines classicos do Bit-Bots, MRL,
B-Human e NimbRo:

  1. reduz o frame (work_width) e converte para HSV
  2. calibra a cor do campo DINAMICAMENTE: pico do histograma de matiz na
     parte de baixo da imagem (que quase sempre e campo), com faixas de S/V
     adaptadas e suavizadas no tempo -- nada de calibrar tabela de cor a mao
  3. mascara de verde + morfologia
  4. borda do campo: topo do verde em cada coluna -> convex hull (o campo
     visto pela camera e um quadrilatero convexo; a hull "pula" por cima de
     robos e outros oclusores)
  5. linhas: top-hat no canal V (claro em relacao a vizinhanca, tolera mudanca
     de luz global) + pouca saturacao + nao-verde + dentro da hull, descartando
     o que esta dentro das caixas do detector (bola/robos/traves sao brancos)

Rotulos na saida:
  0=background  1=lines  2=field
"""
from collections import namedtuple

import cv2
import numpy as np

BACKGROUND, LINES, FIELD = 0, 1, 2

# label: (h,w) uint8 na resolucao de trabalho | hull: (K,1,2) int32 em pixels do
# frame original, ou None | params: (h_lo, h_hi, s_min, v_min) da cor do campo
SegResult = namedtuple("SegResult", "label hull params")


class FieldSegmenter:
    # Faixa "larga" onde procurar o verde do campo na calibracao (H do OpenCV: 0-179)
    GREEN_HUE_RANGE = (25, 100)
    CALIB_MIN_S = 20         # ignora pixels quase cinza ao calibrar
    CALIB_MIN_V = 25         # ignora pixels quase pretos ao calibrar
    CALIB_ROI_TOP = 0.6      # calibra so com as linhas abaixo de 60% da altura

    def __init__(self, work_width=320, recalib_every=10, ema=0.3,
                 line_kernel=15, line_min_tophat=25, line_min_area=6):
        self.work_width = work_width
        self.recalib_every = recalib_every
        self.ema = ema
        self.line_min_tophat = line_min_tophat
        self.line_min_area = line_min_area
        self.params = None
        self.n = 0
        self._k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self._k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        self._k_erode = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self._k_tophat = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (line_kernel, line_kernel))

    def __call__(self, frame_bgr, exclude_boxes=None):
        """frame_bgr: imagem BGR. exclude_boxes: (N,>=4) xyxy em pixels do frame."""
        h, w = frame_bgr.shape[:2]
        sw = self.work_width
        sh = round(h * sw / w)
        scale = sw / w
        small = cv2.resize(frame_bgr, (sw, sh), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        H, S, V = cv2.split(hsv)

        excl = np.zeros((sh, sw), np.uint8)
        if exclude_boxes is not None and len(exclude_boxes):
            for x1, y1, x2, y2 in (np.asarray(exclude_boxes)[:, :4] * scale).astype(int):
                cv2.rectangle(excl, (x1, y1), (x2, y2), 255, -1)

        if self.params is None or self.n % self.recalib_every == 0:
            self._calibrate(H, S, V, excl)
        self.n += 1

        label = np.zeros((sh, sw), np.uint8)
        if self.params is None:  # ainda nao viu verde suficiente para calibrar
            return SegResult(label, None, None)

        h_lo, h_hi, s_min, v_min = self.params
        green_raw = cv2.inRange(hsv, (int(h_lo), int(s_min), int(v_min)), (int(h_hi), 255, 255))
        green_raw = cv2.morphologyEx(green_raw, cv2.MORPH_OPEN, self._k_open)
        # O fechamento tapa as linhas (finas e nao-verdes): serve para a borda,
        # mas para achar as linhas usamos green_raw
        green = cv2.morphologyEx(green_raw, cv2.MORPH_CLOSE, self._k_close)

        hull = self._field_hull(green)
        if hull is None:
            return SegResult(label, None, tuple(self.params))
        inside = np.zeros_like(green)
        cv2.fillConvexPoly(inside, hull, 255)

        lines = self._lines(S, V, green_raw, inside, excl)
        label[inside > 0] = FIELD
        label[lines] = LINES

        hull_frame = np.round(hull / scale).astype(np.int32)
        return SegResult(label, hull_frame, tuple(self.params))

    def _calibrate(self, H, S, V, excl):
        top = int(H.shape[0] * self.CALIB_ROI_TOP)
        h, s, v, ex = H[top:], S[top:], V[top:], excl[top:]
        lo, hi = self.GREEN_HUE_RANGE
        sel = (ex == 0) & (s > self.CALIB_MIN_S) & (v > self.CALIB_MIN_V) & (h >= lo) & (h <= hi)
        if sel.sum() < 0.05 * h.size:
            return  # pouco verde a vista (robo olhando pra fora?): mantem a calibracao anterior

        hist = np.bincount(h[sel], minlength=180).astype(np.float32)
        hist = np.convolve(hist, np.ones(5, np.float32) / 5, mode="same")
        peak = int(hist.argmax())

        near = sel & (np.abs(h.astype(np.int16) - peak) <= 15)
        half = float(np.clip(2.5 * h[near].std(), 8, 20))
        new = np.array([
            peak - half,
            peak + half,
            max(15.0, np.percentile(s[near], 5) * 0.7),
            max(15.0, np.percentile(v[near], 2) * 0.5),  # folga grande p/ sombras
        ], dtype=np.float32)
        self.params = new if self.params is None else (1 - self.ema) * self.params + self.ema * new

    @staticmethod
    def _field_hull(green):
        n, lab, stats, _ = cv2.connectedComponentsWithStats(green, connectivity=8)
        if n <= 1:
            return None
        areas = stats[1:, cv2.CC_STAT_AREA]
        keep = np.zeros(n, bool)
        keep[1:] = areas >= max(areas.max() * 0.15, green.size * 0.01)
        if not keep.any():
            return None
        m = keep[lab]

        cols = np.flatnonzero(m.any(0))
        tops = m[:, cols].argmax(0)  # primeira linha verde de cima pra baixo
        bottom = m.shape[0] - 1
        pts = np.concatenate([np.stack([cols, tops], 1),
                              [[cols[0], bottom], [cols[-1], bottom]]]).astype(np.int32)
        return cv2.convexHull(pts)

    def _lines(self, S, V, green_raw, inside, excl):
        tophat = cv2.morphologyEx(V, cv2.MORPH_TOPHAT, self._k_tophat)
        inner = cv2.erode(inside, self._k_erode)  # evita a transicao campo/fundo
        field_s = S[green_raw > 0]
        s_max = max(40.0, 0.8 * float(np.median(field_s))) if field_s.size else 60.0

        cand = ((tophat > self.line_min_tophat) & (S < s_max) & (green_raw == 0)
                & (inner > 0) & (excl == 0)).astype(np.uint8)

        n, lab, stats, _ = cv2.connectedComponentsWithStats(cand, connectivity=8)
        keep = stats[:, cv2.CC_STAT_AREA] >= self.line_min_area
        keep[0] = False
        return keep[lab]

