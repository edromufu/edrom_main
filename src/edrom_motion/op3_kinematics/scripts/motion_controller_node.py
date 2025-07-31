#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from op3_kinematics.srv import SolveIK
import numpy as np

class MotionControllerNode(Node):
    def __init__(self):
        super().__init__('motion_controller_node')
        
        # Cliente para o serviço de IK em C++
        self.ik_client = self.create_client(SolveIK, 'solve_ik')
        while not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Serviço de IK C++ não disponível, esperando...')

        # Publisher para enviar os ângulos desejados para o nó do hardware
        self.goal_joint_pub = self.create_publisher(JointState, '/goal_joint_states', 10)

        # Subscriber para receber o alvo do pé
        self.target_sub = self.create_subscription(
            PoseStamped, '/ik_target_pose', self.target_callback, 10)
        
        # Variáveis para o planejamento da trajetória
        self.planning_timer = None
        self.start_pose_vec = None
        self.end_pose_vec = None
        self.duration = 1.0  # Duração do movimento em segundos
        self.step_height = 0.03 # Altura do passo em metros
        self.start_time = None
            
        self.get_logger().info("Controlador de Movimento com Planejamento iniciado.")
        self.get_logger().info("Aguardando alvo em /ik_target_pose...")

    def target_callback(self, msg: PoseStamped):
        """Callback chamada quando um novo alvo final é recebido."""
        self.get_logger().info("Novo alvo recebido! Iniciando planejamento de trajetória.")
        
        # Para este exemplo, a posição inicial é fixa.
        # Numa versão mais avançada, leríamos a posição atual do robô.
        self.start_pose_vec = np.array([0.0, -0.05, 0.0])
        self.end_pose_vec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        # Inicia o timer de planejamento
        self.start_time = self.get_clock().now()
        if self.planning_timer is not None and not self.planning_timer.is_canceled():
            self.planning_timer.cancel()
        self.planning_timer = self.create_timer(0.02, self.planning_step) # 50Hz

    def planning_step(self):
        """Executado a cada passo do timer para calcular e comandar um ponto da trajetória."""
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        t_fraction = elapsed_time / self.duration
        if t_fraction > 1.0:
            t_fraction = 1.0

        # Interpolação Linear para X e Y
        current_x = self.start_pose_vec[0] + (self.end_pose_vec[0] - self.start_pose_vec[0]) * t_fraction
        current_y = self.start_pose_vec[1] + (self.end_pose_vec[1] - self.start_pose_vec[1]) * t_fraction
        
        # Interpolação com Seno para Z para criar o arco
        current_z = self.start_pose_vec[2] + self.step_height * np.sin(np.pi * t_fraction)

        # Monta e envia a requisição de IK para este ponto intermediário
        request = SolveIK.Request()
        request.leg_id = 'direita'
        request.target_pose.position.x = current_x
        request.target_pose.position.y = current_y
        request.target_pose.position.z = current_z
        request.target_pose.orientation.w = 1.0

        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_response_callback)

        # Para o timer quando o movimento terminar
        if t_fraction >= 1.0:
            self.get_logger().info("Trajetória concluída.")
            self.planning_timer.cancel()

    def ik_response_callback(self, future):
        """Recebe o resultado da IK e envia para o hardware."""
        try:
            response = future.result()
            if response.success:
                # Publica a mensagem JointState que o dynamixel_node escuta
                self.goal_joint_pub.publish(response.result_joint_state)
            # Não mostramos aviso de falha aqui para não poluir o log durante a trajetória
        except Exception as e:
            self.get_logger().error(f"Chamada de serviço de IK falhou durante a trajetória: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MotionControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()