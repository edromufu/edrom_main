#!/usr/bin/env python3
"""
No de visao `vision` (pipeline hibrido do yoeopth):

  * deteccao: YOLO26n da Ultralytics (modelo/yolov26n-edrom.pt)
      0=ball 1=goalpost 2=robot 3=L-Intersection 4=T-Intersection 5=X-Intersection
  * segmentacao: classica, por algoritmo (field_seg.py, OpenCV, poucos ms)
      campo, borda do campo (convex hull) e linhas

Entrada:  /camera/image_raw (sensor_msgs/Image, publicado pelo usb_cam)
Saidas (soccer_vision_2d_msgs, coordenadas em pixels da imagem da camera):
  balls_in_image           BallArray
  goal_posts_in_image      GoalpostArray
  robots_in_image          RobotArray
  markings_in_image        MarkingArray (intersecoes L/T/X + segmentos das linhas)
  field_boundary_in_image  FieldBoundary (pontos da borda do campo)
Depuracao: janela do OpenCV (show_window) e vision/debug_image (sensor_msgs/Image, so
e gerado se alguem assinar)
"""
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
import soccer_vision_2d_msgs.msg as sv2dm
from vision_msgs.msg import Point2D

from object_finder.field_seg import LINES, FieldSegmenter

DEFAULT_MODEL = 'yolov26n-edrom.pt'

# nome da classe -> numero de raios da intersecao (MarkingIntersection.num_rays)
INTERSECTION_RAYS = {'L-Intersection': 2, 'T-Intersection': 3, 'X-Intersection': 4}

# Cores do overlay de depuracao (BGR)
DET_COLORS = {'ball': (0, 165, 255), 'goalpost': (0, 255, 255), 'robot': (60, 60, 220)}
OTHER_COLOR = (255, 255, 0)
LINE_COLOR = (255, 0, 0)
SEG_COLORS = np.array([(0, 0, 0), LINE_COLOR, (40, 160, 40)], dtype=np.uint8)  # fundo, linha, campo
HULL_COLOR = (255, 0, 255)
SEGMENT_COLOR = (0, 0, 255)


def point2d(x, y):
    p = Point2D()
    p.x, p.y = float(x), float(y)
    return p


class YoeoVision(Node):

    def __init__(self):
        super().__init__('vision')

        p = self.declare_parameter
        self.image_topic = p('image_topic', '/camera/image_raw').value
        model_path = p('model_path', '').value
        self.imgsz = int(p('imgsz', 416).value)
        self.conf = float(p('conf', 0.4).value)
        self.iou = float(p('iou', 0.45).value)
        self.max_det = int(p('max_det', 20).value)
        device = p('device', 'cpu').value
        threads = int(p('threads', 0).value)
        self.classic_seg = bool(p('classic_seg', True).value)
        seg_width = int(p('seg_width', 320).value)
        # Segmentos de linha (HoughLinesP sobre a mascara de linhas)
        self.hough_threshold = int(p('lines.hough_threshold', 15).value)
        self.hough_min_length = int(p('lines.min_length', 12).value)
        self.hough_max_gap = int(p('lines.max_gap', 4).value)
        self.print_every = int(p('print_every', 150).value)
        self.show_window = bool(p('show_window', True).value)
        if self.show_window and not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
            self.get_logger().warn('show_window: sem monitor (DISPLAY vazio), janela desligada. '
                                   'Use o topico vision/debug_image.')
            self.show_window = False

        if threads > 0:
            import torch
            torch.set_num_threads(threads)

        # ---------- modelo ----------
        if not model_path:
            model_path = str(Path(get_package_share_directory('object_finder'))
                             / 'modelo' / DEFAULT_MODEL)
        if not os.path.exists(model_path):
            raise FileNotFoundError(f'Modelo nao encontrado: {model_path}')
        from ultralytics import YOLO
        self.model = YOLO(model_path, task='detect')
        names = self.model.names
        self.names = [names[k] for k in sorted(names)]
        # Pasta *_openvino_model: a Ultralytics escolhe o device com "intel:cpu/gpu"
        is_ov = os.path.isdir(model_path) or model_path.endswith('.xml')
        self.device = f'intel:{device.lower()}' if is_ov else device
        self.get_logger().info(
            f'Modelo: {model_path} ({"openvino" if is_ov else "torch"}, {self.device}, '
            f'{self.imgsz}px) | classes: {self.names}')

        self.segmenter = FieldSegmenter(work_width=seg_width) if self.classic_seg else None
        self.get_logger().info(
            f'Segmentacao classica: {f"ligada ({seg_width}px)" if self.segmenter else "desligada"}')


        # ---------- ROS ----------
        self.bridge = CvBridge()
        # So o frame mais recente interessa: se a inferencia atrasar, descarta os antigos
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, self.image_topic, self.image_callback, qos)

        self.pub_ball = self.create_publisher(sv2dm.BallArray, 'balls_in_image', 1)
        self.pub_goal = self.create_publisher(sv2dm.GoalpostArray, 'goal_posts_in_image', 1)
        self.pub_robot = self.create_publisher(sv2dm.RobotArray, 'robots_in_image', 1)
        self.pub_markings = self.create_publisher(sv2dm.MarkingArray, 'markings_in_image', 1)
        self.pub_boundary = self.create_publisher(
            sv2dm.FieldBoundary, 'field_boundary_in_image', 1)
        self.pub_debug = self.create_publisher(Image, 'vision/debug_image', 1)

        self.n = 0
        self.last_frame_time = None
        # Sem imagem nenhuma o no fica mudo (e a janela nao abre): avisa no log
        self.create_timer(3.0, self.check_camera)
        self.t_det = 0.0
        self.t_seg = 0.0
        self.t_total = 0.0

    # ------------------------------------------------------------------
    def image_callback(self, msg):
        try:
            self.process(msg)
        except Exception as e:  # nao derruba o no por causa de um frame ruim
            self.get_logger().error(f'Erro processando frame: {e}')

    def check_camera(self):
        now = time.monotonic()
        if self.last_frame_time is None or now - self.last_frame_time > 3.0:
            self.get_logger().warn(
                f'Nenhuma imagem chegando em {self.image_topic}. A camera esta conectada '
                f'(ls /dev/video*) e o usb_cam subiu sem erro? Confira o video_device.')

    def process(self, msg):
        self.last_frame_time = time.monotonic()
        t0 = time.perf_counter()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        w = frame.shape[1]

        # ---------- deteccao (YOLO26n, end-to-end) ----------
        r = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf, iou=self.iou,
                               max_det=self.max_det, device=self.device, verbose=False)[0]
        boxes = r.boxes.data.cpu().numpy()  # (N,6) x1,y1,x2,y2,conf,cls
        t1 = time.perf_counter()

        # ---------- segmentacao classica ----------
        seg = None
        segments = []
        if self.segmenter is not None:
            # bola/robos/traves sao brancos: as caixas nao podem virar "linha"
            seg = self.segmenter(frame, exclude_boxes=boxes)
            segments = self.line_segments(seg.label, w)
        t2 = time.perf_counter()

        header = msg.header
        msg_ball = sv2dm.BallArray(header=header)
        msg_goal = sv2dm.GoalpostArray(header=header)
        msg_robot = sv2dm.RobotArray(header=header)
        msg_mark = sv2dm.MarkingArray(header=header)
        msg_bound = sv2dm.FieldBoundary(header=header)

        for x1, y1, x2, y2, conf, cls in boxes:
            cls = int(cls)
            name = self.names[cls] if cls < len(self.names) else f'cls{cls}'
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            bw, bh = float(x2 - x1), float(y2 - y1)

            if name == 'ball':
                o = sv2dm.Ball()
                o.center = point2d(cx, cy)
            elif name == 'goalpost':
                o = sv2dm.Goalpost()
            elif name == 'robot':
                o = sv2dm.Robot()
            elif name in INTERSECTION_RAYS:
                o = sv2dm.MarkingIntersection()
                o.center = point2d(cx, cy)
                o.num_rays = INTERSECTION_RAYS[name]
                o.heading_rays = []  # o detector nao estima a direcao dos raios
                o.confidence.confidence = float(conf)
                msg_mark.intersections.append(o)
                continue
            else:
                continue

            o.bb.center.position.x = float(cx)
            o.bb.center.position.y = float(cy)
            o.bb.size_x, o.bb.size_y = bw, bh
            o.confidence.confidence = float(conf)
            {'ball': msg_ball.balls, 'goalpost': msg_goal.posts,
             'robot': msg_robot.robots}[name].append(o)

        for (x1, y1, x2, y2) in segments:
            s = sv2dm.MarkingSegment()
            s.start = point2d(x1, y1)
            s.end = point2d(x2, y2)
            s.confidence.confidence = 1.0
            msg_mark.segments.append(s)

        if seg is not None and seg.hull is not None:
            msg_bound.points = [point2d(x, y) for x, y in seg.hull.reshape(-1, 2)]
            msg_bound.confidence.confidence = 1.0
        else:
            msg_bound.confidence.confidence = 0.0

        self.pub_ball.publish(msg_ball)
        self.pub_goal.publish(msg_goal)
        self.pub_robot.publish(msg_robot)
        self.pub_markings.publish(msg_mark)
        self.pub_boundary.publish(msg_bound)
        t3 = time.perf_counter()

        # ---------- depuracao ----------
        want_topic = self.pub_debug.get_subscription_count() > 0
        if want_topic or self.show_window:
            fps = 1.0 / (t3 - t0) if t3 > t0 else 0.0
            out = self.draw_overlay(frame, boxes, seg, segments, fps,
                                    (t1 - t0) * 1e3, (t2 - t1) * 1e3)
            if want_topic:
                dbg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
                dbg.header = header
                self.pub_debug.publish(dbg)
            if self.show_window:
                cv2.imshow('object_finder', out)
                cv2.waitKey(1)

        # ---------- estatisticas ----------
        self.n += 1
        self.t_det += t1 - t0
        self.t_seg += t2 - t1
        self.t_total += t3 - t0
        if self.print_every > 0 and self.n % self.print_every == 0:
            k = self.print_every
            self.get_logger().info(
                f'{k} frames: deteccao {self.t_det / k * 1e3:.1f} ms | '
                f'segmentacao {self.t_seg / k * 1e3:.1f} ms | '
                f'total {self.t_total / k * 1e3:.1f} ms ({k / self.t_total:.1f} fps max)')
            self.t_det = self.t_seg = self.t_total = 0.0

    # ------------------------------------------------------------------
    def line_segments(self, label, frame_w):
        """Segmentos de reta (x1,y1,x2,y2 em pixels do frame) da mascara de linhas."""
        mask = (label == LINES).astype(np.uint8) * 255
        if not mask.any():
            return []
        found = cv2.HoughLinesP(mask, 1, np.pi / 180, self.hough_threshold,
                                minLineLength=self.hough_min_length,
                                maxLineGap=self.hough_max_gap)
        if found is None:
            return []
        scale = frame_w / label.shape[1]
        return (found.reshape(-1, 4).astype(np.float32) * scale).tolist()

    def draw_overlay(self, frame, boxes, seg, segments, fps, det_ms, seg_ms):
        """Desenha segmentacao + borda + segmentos + caixas no frame."""
        h, w = frame.shape[:2]
        if seg is not None:
            lab = cv2.resize(seg.label, (w, h), interpolation=cv2.INTER_NEAREST)
            out = cv2.addWeighted(frame, 0.7, SEG_COLORS[lab], 0.3, 0)
            # linhas opacas: com transparencia o azul some sobre a linha branca
            out[lab == LINES] = LINE_COLOR
            if seg.hull is not None:
                cv2.polylines(out, [seg.hull], True, HULL_COLOR, 2)
        else:
            out = frame.copy()

        for x1, y1, x2, y2 in segments:
            cv2.line(out, (int(x1), int(y1)), (int(x2), int(y2)), SEGMENT_COLOR, 1)

        for x1, y1, x2, y2, conf, cls in boxes:
            cls = int(cls)
            name = self.names[cls] if cls < len(self.names) else f'cls{cls}'
            color = DET_COLORS.get(name, OTHER_COLOR)
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            label = f'{name} {conf:.2f}'
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(out, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
            cv2.putText(out, label, (x1 + 2, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        status = (f'FPS {fps:5.1f} | det {det_ms:.1f}ms | seg {seg_ms:.1f}ms | '
                  f'{len(boxes)} det | {len(segments)} seg')
        cv2.rectangle(out, (0, 0), (w, 22), (0, 0, 0), -1)
        cv2.putText(out, status, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = YoeoVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
