#!/usr/bin/env python3
# coding=utf-8
"""Nó unificado de Path Planning para Simulação e Debug."""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray

from ..path_planning.hybridplanner import Blackboard, HybridPlanner, TypeRef, Reference

DEFAULT_FIELD_BOUNDS = (-4.3, 4.3, -2.8, 2.8)
ROBOT_RADIUS = 0.12
MARGIN = 0.05


def compute_simulated_velocities(path, initial_yaw, max_linear=0.5, max_angular=1.2):
    """
    A partir do array de posições do path planning, simula o movimento entre os pontos
    e retorna um array de tuplas (dx, dy, psi) para cada ponto da trajetória.
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

        #erro angular normalizado [-pi, pi]
        error_yaw = math.atan2(math.sin(desired_yaw - sim_yaw), math.cos(desired_yaw - sim_yaw))
        psi_vel = max(-max_angular, min(max_angular, 2.5 * error_yaw))

        #velocidade escalar baseada na lei de controle da simulação
        dist_step = math.hypot(world_dx, world_dy)
        dist_goal = math.dist(curr_pos, goal)
        speed = min(max_linear, 2.0 * dist_step, 2.0 * max(0.0, dist_goal))

        #projeta a velocidade no referencial local do robô (dx frontal, dy lateral)
        dx_local = speed * math.cos(error_yaw) if abs(error_yaw) < 0.8 else 0.0
        dy_local = speed * math.sin(error_yaw) if abs(error_yaw) < 0.8 else 0.0

        velocities.append((dx_local, dy_local, psi_vel))
        #supõe que o robô alinhou com o segmento na simulação para o próximo ponto
        sim_yaw = desired_yaw

    #ultimo ponto
    velocities.append((0.0, 0.0, 0.0))
    return velocities


class PathPlannerSimNode(Node):
    def __init__(self):
        super().__init__('path_planner_sim_node')

        radius = self.declare_parameter('robot_radius', ROBOT_RADIUS).value
        margin = self.declare_parameter('safety_margin', MARGIN).value
        self.max_speed = self.declare_parameter('max_linear_speed', 0.5).value
        self.max_angular = self.declare_parameter('max_angular_speed', 1.2).value
        self.field_bounds = tuple(self.declare_parameter('field_bounds', list(DEFAULT_FIELD_BOUNDS)).value)

        self.bb = Blackboard()
        params_apf = {
            "k_atr": 1.5, "k_rep": 0.08, "d0": 0.35,
            "raio_robo": radius, "velocidade_max": 0.25, "dt": 0.05
        }
        self.planner = HybridPlanner(
            self.bb, params_apf, limites=self.field_bounds, margem_seguranca=margin
        )

        #armazena as mensagens recebidas
        self.inputs = {}
        self.required_keys = ('robot', 'ball', 'goal', 'obstacles')

        #subscribers da Simulação / Debug
        self.create_subscription(PoseStamped, '/simulation/robot_pose', lambda m: self.receive('robot', m), 10)
        self.create_subscription(PointStamped, '/simulation/ball_position', lambda m: self.receive('ball', m), 10)
        self.create_subscription(PointStamped, '/simulation/goal_position', lambda m: self.receive('goal', m), 10)
        self.create_subscription(MarkerArray, '/simulation/obstacles', lambda m: self.receive('obstacles', m), 10)

        #publishers dos 2 Arrays + Debug Webots
        self.positions_pub = self.create_publisher(Path, '/planning/positions_array', 10)
        self.velocities_pub = self.create_publisher(Float64MultiArray, '/planning/velocities_array', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.05, self.update_loop)  # 20 Hz

    def receive(self, key, msg):
        self.inputs[key] = (msg, time.monotonic())
        #na simulação, se não houver tópico separado de 'goal', usa a própria bola como destino
        if key == 'ball' and 'goal' not in self.inputs:
            self.inputs['goal'] = (msg, time.monotonic())

    def check_inputs_valid(self):
        missing = [k for k in self.required_keys if k not in self.inputs]
        if missing:
            self.get_logger().warn(
                f"[SIM/DEBUG ALERTA] Aguardando informações obrigatórias no ROS: {missing}",
                throttle_duration_sec=2.0
            )
            return False

        now_mono = time.monotonic()
        expired = [k for k, (_, t) in self.inputs.items() if now_mono - t > 1.5]
        if expired:
            self.get_logger().warn(
                f"[SIM/DEBUG ALERTA] Dados expirados (>1.5s) nos canais: {expired}",
                throttle_duration_sec=2.0
            )
            return False
        return True

    def update_loop(self):
        if not rclpy.ok():
            return

        #verifica se todas as informações necessárias chegaram; se não, emite alerta e para
        if not self.check_inputs_valid():
            self.cmd_vel_pub.publish(Twist())
            return

        robot_msg = self.inputs['robot'][0]
        ball_msg = self.inputs['ball'][0]
        goal_msg = self.inputs['goal'][0]
        obs_msg = self.inputs['obstacles'][0]

        me_pos = (robot_msg.pose.position.x, robot_msg.pose.position.y)
        ball_pos = (ball_msg.point.x, ball_msg.point.y)
        goal_pos = (goal_msg.point.x, goal_msg.point.y)

        q = robot_msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        obstacles = [
            {"x": m.pose.position.x, "y": m.pose.position.y, "raio": max(m.scale.x, m.scale.y) / 2.0}
            for m in obs_msg.markers if max(m.scale.x, m.scale.y) > 0
        ]

        #alimenta o Blackboard e roda o algoritmo híbrido (Campo Vetorial + Theta*)
        self.bb.set(TypeRef.ME_POS, me_pos)
        self.bb.set(TypeRef.BALL_POS, goal_pos)  # HybridPlanner usa BALL_POS como alvo final
        self.bb.set(TypeRef.GOL_POS, ball_pos)
        self.bb.set(TypeRef.OBSTACLES, obstacles)
        self.bb.set(TypeRef.REFERENCE_POS, [Reference(o["x"], o["y"], i) for i, o in enumerate(obstacles)])

        positions_array = self.planner.processar_ciclo()

        if not positions_array:
            self.get_logger().warn("[SIM/DEBUG ALERTA] Nenhum caminho válido encontrado!", throttle_duration_sec=1.0)
            self.cmd_vel_pub.publish(Twist())
            return

        #cria o array de velocidades (dx, dy, psi) para cada ponto supondo a cinemática da simulação
        velocities_array = compute_simulated_velocities(
            positions_array, yaw, self.max_speed, self.max_angular
        )

        #publica os dois arrays e o comando imediato para o simulador
        self.publish_outputs(positions_array, velocities_array, robot_msg.header)

    def publish_outputs(self, positions_array, velocities_array, header):
        #publica as posições como Path
        path_msg = Path()
        path_msg.header = header
        for x, y in positions_array:
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.positions_pub.publish(path_msg)

        #publica as velocidades como Float64MultiArray
        vel_msg = Float64MultiArray()
        for dx, dy, psi in velocities_array:
            vel_msg.data.extend([float(dx), float(dy), float(psi)])
        self.velocities_pub.publish(vel_msg)

        #publica o comando de velocidade imediato para o simulador (primeiro ponto do array)
        cmd = Twist()
        cmd.linear.x = float(velocities_array[0][0])
        cmd.linear.y = float(velocities_array[0][1])
        cmd.angular.z = float(velocities_array[0][2])
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd_vel_pub.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()