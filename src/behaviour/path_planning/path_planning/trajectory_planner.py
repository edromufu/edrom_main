#!/usr/bin/env python3
# coding=utf-8
"""Nó de Path Planning para o Robô"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray

from .hybridplanner import Blackboard, HybridPlanner, TypeRef, Reference

ROBOT_RADIUS = 0.12
MARGIN = 0.05


def compute_simulated_velocities(path, initial_yaw, max_linear=0.2, max_angular=0.8):
    """
    Gera o array [(dx, dy, psi), ...] para cada ponto do caminho planejado,
    supondo que o robô caminhe segundo o modelo cinemático da simulação.
    """
    if len(path) < 2:
        return [(0.0, 0.0, 0.0) for _ in path]

    velocities = []
    sim_yaw = initial_yaw
    goal = path[-1]

    for i in range(len(path) - 1):
        curr_pos = path[i]
        next_pos = path[i + 1]

        if math.dist(curr_pos, goal) <= 0.05:
            velocities.append((0.0, 0.0, 0.0))
            continue

        world_dx = next_pos[0] - curr_pos[0]
        world_dy = next_pos[1] - curr_pos[1]
        desired_yaw = math.atan2(world_dy, world_dx)

        error_yaw = math.atan2(math.sin(desired_yaw - sim_yaw), math.cos(desired_yaw - sim_yaw))
        psi_vel = max(-max_angular, min(max_angular, 2.0 * error_yaw))

        dist_step = math.hypot(world_dx, world_dy)
        dist_goal = math.dist(curr_pos, goal)
        speed = min(max_linear, 2.0 * dist_step, 2.0 * max(0.0, dist_goal))

        dx_local = speed * math.cos(error_yaw) if abs(error_yaw) < 0.6 else 0.0
        dy_local = speed * math.sin(error_yaw) if abs(error_yaw) < 0.6 else 0.0

        velocities.append((dx_local, dy_local, psi_vel))
        sim_yaw = desired_yaw

    velocities.append((0.0, 0.0, 0.0))
    return velocities


class TrajectoryPlanner(Node):
    """
    Canal de entrada esperado (/planning/world_state - Float64MultiArray):
    [min_x, max_x, min_y, max_y,   # 0..3: Dimensões do campo (m)
     me_x, me_y, me_yaw,           # 4..6: Onde estou (x, y, psi)
     goal_x, goal_y,               # 7..8: Onde devo ir (x, y)
     ball_x, ball_y,               # 9..10: Onde está a bola (x, y)
     obs1_x, obs1_y, obs1_r, ...]  # 11..N: Obstáculos em trincas (x, y, raio)
    """
    def __init__(self):
        super().__init__('path_planner_real_node')

        self.radius = self.declare_parameter('robot_radius', ROBOT_RADIUS).value
        self.margin = self.declare_parameter('safety_margin', MARGIN).value
        self.max_speed = self.declare_parameter('max_linear_speed', 0.15).value
        self.max_angular = self.declare_parameter('max_angular_speed', 0.8).value

        self.bb = Blackboard()
        self.params_apf = {
            "k_atr": 1.5, "k_rep": 0.08, "d0": 0.35,
            "raio_robo": self.radius, "velocidade_max": 0.25, "dt": 0.05
        }
        self.planner = None
        self.last_msg_time = None
        self.world_data = None

        #canal do ROS que recebe todas as informações necessárias
        self.create_subscription(
            Float64MultiArray, '/planning/world_state', self.world_state_callback, 10
        )

        #publishers dos dois arrays para a rotina de andar (walking_routine)
        self.positions_pub = self.create_publisher(Path, '/planning/positions_array', 10)
        self.velocities_pub = self.create_publisher(Float64MultiArray, '/planning/velocities_array', 10)

        self.create_timer(0.05, self.process_planning_cycle)  # 20 Hz

    def world_state_callback(self, msg: Float64MultiArray):
        data = list(msg.data)
        #precisa de pelo menos 11 floats básicos + múltiplo de 3 para os obstáculos
        if len(data) < 11 or (len(data) - 11) % 3 != 0:
            self.get_logger().warn(
                f"[PathPlanner] Erro ao receber dados do mundo real. "
                f"Recebidos {len(data)} valores (mínimo esperado: 11)."
            )
            self.world_data = None
            return

        if not all(math.isfinite(v) for v in data):
            self.get_logger().warn("[PathPlanner] Valores NaN ou infinitos detectados no pacote!")
            self.world_data = None
            return

        field_bounds = (data[0], data[1], data[2], data[3])
        me_pos = (data[4], data[5])
        me_yaw = data[6]
        goal_pos = (data[7], data[8])
        ball_pos = (data[9], data[10])

        obstacles = []
        for i in range(11, len(data), 3):
            if data[i + 2] > 0:
                obstacles.append({"x": data[i], "y": data[i + 1], "raio": data[i + 2]})

        self.world_data = {
            "field_bounds": field_bounds,
            "me_pos": me_pos,
            "me_yaw": me_yaw,
            "goal_pos": goal_pos,
            "ball_pos": ball_pos,
            "obstacles": obstacles
        }
        self.last_msg_time = time.monotonic()

    def process_planning_cycle(self):
        #verifica se recebeu todas as informações necessárias; se não, dá um alerta
        if self.world_data is None or self.last_msg_time is None:
            self.get_logger().warn(
                "[PathPlanner] Sem informações completas (campo, pose, alvo, bola) no canal /planning/world_state!",
                throttle_duration_sec=2.0
            )
            return

        if time.monotonic() - self.last_msg_time > 1.0:
            self.get_logger().warn(
                "[PathPlanner] Canal /planning/world_state parou de atualizar",
                throttle_duration_sec=2.0
            )
            return

        #alimenta o Blackboard e configura o HybridPlanner com os limites do campo
        bounds = self.world_data["field_bounds"]
        if self.planner is None or self.planner.limites != bounds:
            self.planner = HybridPlanner(
                self.bb, self.params_apf, limites=bounds, margem_seguranca=self.margin
            )

        me_pos = self.world_data["me_pos"]
        me_yaw = self.world_data["me_yaw"]
        goal_pos = self.world_data["goal_pos"]
        ball_pos = self.world_data["ball_pos"]
        obstacles = self.world_data["obstacles"]

        self.bb.set(TypeRef.ME_POS, me_pos)
        self.bb.set(TypeRef.BALL_POS, goal_pos)  #destino onde o robô deve chegar
        self.bb.set(TypeRef.GOL_POS, ball_pos)   #posição da bola salva no Blackboard
        self.bb.set(TypeRef.OBSTACLES, obstacles)
        self.bb.set(TypeRef.REFERENCE_POS, [Reference(o["x"], o["y"], i) for i, o in enumerate(obstacles)])

        #executa o algoritmo de Path Planning
        positions_array = self.planner.processar_ciclo()

        if not positions_array:
            self.get_logger().warn(
                "[PathPlanner] Falha ao calcular o caminho até o alvo",
                throttle_duration_sec=1.0
            )
            return

        #calcula o array contendo as velocidades (dx, dy, psi) para cada ponto
        velocities_array = compute_simulated_velocities(
            positions_array, me_yaw, self.max_speed, self.max_angular
        )

        #publica os dois arrays para a rotina de andar (walking_routine)
        self.publish_arrays(positions_array, velocities_array)

    def publish_arrays(self, positions_array, velocities_array):
        now_msg = self.get_clock().now().to_msg()

        #posições
        path_msg = Path()
        path_msg.header.stamp = now_msg
        path_msg.header.frame_id = "map"
        for x, y in positions_array:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.positions_pub.publish(path_msg)

        # velocidades (dx, dy, psi)
        vel_msg = Float64MultiArray()
        for dx, dy, psi in velocities_array:
            vel_msg.data.extend([float(dx), float(dy), float(psi)])
        self.velocities_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()