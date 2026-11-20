#!/usr/bin/env python3
# coding=utf-8

import threading
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image as ROS_Image


ARUCO_DICTS = {
    'DICT_4X4_50':  cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_5X5_50':  cv2.aruco.DICT_5X5_50,
    'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
    'DICT_6X6_50':  cv2.aruco.DICT_6X6_50,
    'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
}


class IPMCalibration(Node):
    """
    Nó de calibração do IPM (Inverse Perspective Mapping) usando marcadores ArUco.

    Posicione os marcadores em posições conhecidas no chão, aponte a câmera e
    pressione [c] para computar a homografia. Pressione [s] para salvar.

    Parâmetros ROS:
        use_simulation      (bool)    — assina /camera/image em vez de usar webcam
        vision.camera_idx   (int)     — índice da webcam (padrão 0)
        ipm.aruco_dict      (string)  — dicionário ArUco (padrão DICT_4X4_50)
        ipm.marker_ids      (int[])   — IDs dos marcadores a usar (padrão [0,1,2,3])
        ipm.marker_world_points (float[]) — coordenadas reais em cm, flat:
                                           [x0,y0, x1,y1, ...] na mesma ordem
                                           que marker_ids (padrão: quadrado 100cm)
        ipm.output_path     (string)  — caminho de saída do .npy
                                        (padrão: share/localization_pkg/resource/)
    """

    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info('=== Calibração IPM por ArUco iniciada ===')

        # ── Parâmetros ────────────────────────────────────────────────────────
        self.use_simulation = self.declare_parameter(
            'use_simulation', False).get_parameter_value().bool_value
        self.camera_idx = self.declare_parameter(
            'vision.camera_idx', 0).get_parameter_value().integer_value
        dict_name = self.declare_parameter(
            'ipm.aruco_dict', 'DICT_4X4_50').get_parameter_value().string_value

        ids_param = self.declare_parameter(
            'ipm.marker_ids', [0, 1, 2, 3]).get_parameter_value().integer_array_value

        # Quadrado de 100 cm como padrão: TL, TR, BR, BL
        default_world = [0.0, 0.0,  100.0, 0.0,  100.0, 100.0,  0.0, 100.0]
        world_param = self.declare_parameter(
            'ipm.marker_world_points', default_world).get_parameter_value().double_array_value

        out_param = self.declare_parameter(
            'ipm.output_path', '').get_parameter_value().string_value
        self.output_path = out_param if out_param else self._default_output_path()
        self.get_logger().info(f'Saída: {self.output_path}')

        # ── Detector ArUco ────────────────────────────────────────────────────
        aruco_dict = cv2.aruco.getPredefinedDictionary(
            ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50))
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)

        # Mapa ID → (x, y) em cm no plano do chão
        flat = list(world_param)
        self.world_map = {
            int(mid): (flat[2 * i], flat[2 * i + 1])
            for i, mid in enumerate(ids_param)
            if 2 * i + 1 < len(flat)
        }
        self.get_logger().info(f'Marcadores configurados: {self.world_map}')
        self.get_logger().info('Controles: [c] Capturar   [s] Salvar   [q] Sair')

        # ── Estado ────────────────────────────────────────────────────────────
        self.homography = None
        self.current_frame = None
        self.capture_lock = threading.Lock()

        # ── Câmera ────────────────────────────────────────────────────────────
        if self.use_simulation:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            self.create_subscription(ROS_Image, '/camera/image', self._img_cb, 10)
        else:
            self.cap = cv2.VideoCapture(self.camera_idx)
            if not self.cap.isOpened():
                self.get_logger().error(f'Câmera {self.camera_idx} não encontrada.')
                rclpy.shutdown()
                return
            threading.Thread(target=self._capture_loop, daemon=True).start()

        self.create_timer(1.0 / 30.0, self._process)

    # ── Caminho padrão de saída ───────────────────────────────────────────────

    def _default_output_path(self):
        try:
            share = get_package_share_directory('localization_pkg')
            return os.path.join(share, 'resource', 'homography_matrix.npy')
        except Exception:
            return os.path.expanduser('~/homography_matrix.npy')

    # ── Captura ───────────────────────────────────────────────────────────────

    def _img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.capture_lock:
                self.current_frame = frame
        except Exception as e:
            self.get_logger().error(f'Conversão de imagem falhou: {e}')

    def _capture_loop(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                with self.capture_lock:
                    self.current_frame = frame

    # ── Loop principal ────────────────────────────────────────────────────────

    def _process(self):
        with self.capture_lock:
            if self.current_frame is None:
                return
            frame = self.current_frame.copy()

        display = self._draw(frame)
        cv2.imshow('IPM Calibration - ArUco', display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            self._compute(frame)
        elif key == ord('s'):
            self._save()
        elif key == ord('q'):
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()

    # ── Detecção e visualização ───────────────────────────────────────────────

    def _detect(self, frame):
        corners, ids, _ = self.detector.detectMarkers(frame)
        return corners, ids

    def _draw(self, frame):
        display = frame.copy()
        corners, ids = self._detect(frame)

        detected = 0
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(display, corners, ids)
            for i, mid in enumerate(ids.flatten()):
                mid = int(mid)
                if mid not in self.world_map:
                    continue
                detected += 1
                cx = int(np.mean(corners[i][0][:, 0]))
                cy = int(np.mean(corners[i][0][:, 1]))
                wx, wy = self.world_map[mid]
                cv2.putText(display, f'ID:{mid} ({wx:.0f},{wy:.0f})cm',
                            (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

        total = len(self.world_map)
        ready = detected >= 4
        color = (0, 255, 0) if ready else (0, 165, 255)
        cv2.putText(display, f'Detectados: {detected}/{total}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        if ready:
            cv2.putText(display, '[c] Capturar  [s] Salvar',
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if self.homography is not None:
            cv2.putText(display, 'Homografia calculada — [s] para salvar',
                        (10, display.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return display

    # ── Cálculo da homografia ─────────────────────────────────────────────────

    def _compute(self, frame):
        corners, ids = self._detect(frame)
        if ids is None:
            self.get_logger().warn('Nenhum marcador detectado no frame.')
            return

        img_pts, world_pts = [], []
        for i, mid in enumerate(ids.flatten()):
            mid = int(mid)
            if mid in self.world_map:
                cx = float(np.mean(corners[i][0][:, 0]))
                cy = float(np.mean(corners[i][0][:, 1]))
                img_pts.append([cx, cy])
                world_pts.append(list(self.world_map[mid]))

        if len(img_pts) < 4:
            self.get_logger().warn(
                f'Apenas {len(img_pts)} marcador(es) válido(s). Mínimo 4.')
            return

        src = np.array(img_pts, dtype=np.float32)
        dst = np.array(world_pts, dtype=np.float32)
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)

        if H is None:
            self.get_logger().error('findHomography falhou.')
            return

        inliers = int(mask.sum()) if mask is not None else len(img_pts)
        self.homography = H
        self.get_logger().info(
            f'Homografia calculada ({inliers}/{len(img_pts)} inliers).')
        self.get_logger().info(f'H =\n{H}')
        self._show_warped(frame, H, dst)

    def _show_warped(self, frame, H, world_pts):
        max_x = int(np.max(world_pts[:, 0])) + 50
        max_y = int(np.max(world_pts[:, 1])) + 50
        warped = cv2.warpPerspective(frame, H, (max_x, max_y))
        cv2.imshow('IPM - Vista de Topo (Verificação)', warped)
        cv2.waitKey(1)

    # ── Salvamento ────────────────────────────────────────────────────────────

    def _save(self):
        if self.homography is None:
            self.get_logger().warn('Nenhuma homografia disponível. Use [c] primeiro.')
            return
        try:
            out_dir = os.path.dirname(self.output_path)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
            np.save(self.output_path, self.homography)
            self.get_logger().info(f'homography_matrix.npy salvo em: {self.output_path}')
            self.get_logger().info(
                'Para tornar permanente, copie o arquivo para '
                'src/edrom_vision/localization_pkg/resource/ e recompile.')
        except Exception as e:
            self.get_logger().error(f'Erro ao salvar: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = IPMCalibration('ipm_calibration')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
