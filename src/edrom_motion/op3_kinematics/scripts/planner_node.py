#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose
from op3_kinematics.srv import SolveIK # IMPORTANTE: use o nome do seu pacote C++ aqui

class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')

        # Cria um cliente para o serviço de IK que criamos em C++
        self.ik_client = self.create_client(SolveIK, 'solve_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço de IK não disponível, esperando novamente...')

        # Parâmetros da trajetória
        self.duration = 2.0  # Duração do movimento em segundos
        self.step_height = 0.03  # Altura do passo de 5cm
        self.start_pose = np.array([0.0, -0.05, 0.0]) # Posição inicial
        self.end_pose = np.array([0.1, -0.05, 0.0])   # Posição final

        # Timer para o loop de controle (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.start_time = self.get_clock().now()
        
        self.get_logger().info("Planejador de trajetória iniciado. Executando um passo.")

    def timer_callback(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Calcula a fração do tempo decorrido
        t_fraction = elapsed_time / self.duration
        if t_fraction > 1.0:
            t_fraction = 1.0

        # Interpolação Linear para X e Y
        current_x = self.start_pose[0] + (self.end_pose[0] - self.start_pose[0]) * t_fraction
        current_y = self.start_pose[1] + (self.end_pose[1] - self.start_pose[1]) * t_fraction
        
        # Interpolação com Seno para Z (para criar o arco)
        current_z = self.start_pose[2] + self.step_height * np.sin(np.pi * t_fraction)

        # Monta a requisição do serviço de IK
        request = SolveIK.Request()
        request.leg_id = 'direita'
        request.target_pose.position.x = current_x
        request.target_pose.position.y = current_y
        request.target_pose.position.z = current_z
        request.target_pose.orientation.w = 1.0 # Pé reto

        # Chama o serviço de IK de forma assíncrona
        self.ik_client.call_async(request)

        # Para o timer quando o movimento terminar
        if t_fraction >= 1.0:
            self.get_logger().info("Trajetória concluída.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    planner_node = TrajectoryPlannerNode()
    rclpy.spin(planner_node)
    planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()