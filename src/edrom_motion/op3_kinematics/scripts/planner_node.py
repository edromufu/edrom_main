#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose
from op3_kinematics.srv import SolveIK 

class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')

        self.ik_client = self.create_client(SolveIK, 'solve_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço de IK não disponível, esperando novamente...')

        # Parâmetros da trajetória
        self.duration = 2.0  
        self.step_height = 0.03  
        self.start_pose = np.array([0.0, -0.05, 0.0]) 
        self.end_pose = np.array([0.1, -0.05, 0.0])   

        self.timer = self.create_timer(0.02, self.timer_callback)
        self.start_time = self.get_clock().now()
        
        self.get_logger().info("Planejador de trajetória iniciado. Executando um passo.")

    def timer_callback(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        t_fraction = elapsed_time / self.duration
        if t_fraction > 1.0:
            t_fraction = 1.0

        current_x = self.start_pose[0] + (self.end_pose[0] - self.start_pose[0]) * t_fraction
        current_y = self.start_pose[1] + (self.end_pose[1] - self.start_pose[1]) * t_fraction
        current_z = self.start_pose[2] + self.step_height * np.sin(np.pi * t_fraction)

        request = SolveIK.Request()
        request.leg_id = 'direita'
        request.target_pose.position.x = current_x
        request.target_pose.position.y = current_y
        request.target_pose.position.z = current_z
        request.target_pose.orientation.w = 1.0 # Pé reto

        self.ik_client.call_async(request)

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