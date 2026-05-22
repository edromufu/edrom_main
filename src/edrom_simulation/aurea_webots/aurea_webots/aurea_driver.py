import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient # Importar a classe ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from controller import Robot, Keyboard

# Importar a definição da sua Action de chute
from aurea_kick.action import Kick 

class AureaJointController:
    def init(self, webots_node, properties):
        """
        Esta função é chamada uma vez quando o controlador é iniciado.
        """
        # --- Configuração Inicial (ROS e Webots) ---
        rclpy.init(args=None)
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())
        
        self.__node = rclpy.create_node('aurea_keyboard_controller')
        self.__node.get_logger().info("Controlador de Juntas e Teclado para Aurea iniciado.")

        # --- Parâmetros de Controle ---
        self.WALK_VELOCITY = 0.1
        self.TURN_VELOCITY = 0.21 # Aumentei um pouco para a rotação ser mais visível

        # --- Estado do Controlador ---
        self.__walking_active = False
        self.__previous_key = -1
        self.__last_twist_cmd = Twist()

        # --- Configuração dos Motores (semelhante ao original) ---
        joint_names = [
            'head_pan', 'head_tilt', 'l_sho_pitch', 'l_sho_roll', 'l_el',
            'r_sho_pitch', 'r_sho_roll', 'r_el', 'r_hip_yaw', 'r_hip_roll', 
            'r_hip_pitch', 'r_knee', 'r_ank_pitch', 'r_ank_roll', 'l_hip_yaw', 
            'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch', 'l_ank_roll', 
        ]
        self.__motors = {name: self.__robot.getDevice(name) for name in joint_names}
        
        #mCamera = self.__robot.getDevice("camera")
        #mCamera.enable(self.__timestep)

        # --- Habilitar o Teclado ---
        self.__keyboard = self.__robot.getKeyboard()
        self.__keyboard.enable(self.__timestep)
        self.__node.get_logger().info("Teclado habilitado.")
        self.__node.get_logger().info("- ESPAÇO: Ativa/desativa o modo de caminhada.")
        self.__node.get_logger().info("- WASD: Move o robô.")
        self.__node.get_logger().info("- Q/E: Gira (Anti-horário/Horário).")
        self.__node.get_logger().info("- C: Chuta com a perna direita.")
        self.__node.get_logger().info("- X: Parada de emergência.")

        # --- Publishers e Subscribers ---
        self.__node.create_subscription(JointState, '/goal_joint_states', self.__joint_command_callback, 1)
        self.__cmd_vel_pub = self.__node.create_publisher(Twist, '/cmd_vel', 10)
        self.__stop_pub = self.__node.create_publisher(Empty, '/stop_walking', 10)

        # --- Action Client para o Chute ---
        self.__kick_action_client = ActionClient(self.__node, Kick, '/kick')
        self.__node.get_logger().info("Aguardando o servidor de ação '/kick'...")
        self.__kick_action_client.wait_for_server()
        self.__node.get_logger().info("Servidor de ação '/kick' encontrado.")

        self.__node.get_logger().info("Controlador pronto.")


    def __joint_command_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.__motors:
                self.__motors[name].setPosition(msg.position[i])

    def __send_kick_goal(self):
        self.__node.get_logger().info("Enviando objetivo de chute para a perna direita...")
        goal_msg = Kick.Goal()
        goal_msg.leg_id = 'direita'

        # Envia o objetivo de forma assíncrona
        self._send_goal_future = self.__kick_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.__kick_feedback_callback)

        # Adiciona um callback para quando a meta for aceita/rejeitada
        self._send_goal_future.add_done_callback(self.__kick_goal_response_callback)

    def __kick_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.__node.get_logger().info('Objetivo de chute rejeitado :(')
            return

        self.__node.get_logger().info('Objetivo de chute aceito :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.__kick_get_result_callback)

    def __kick_get_result_callback(self, future):
        result = future.result().result
        self.__node.get_logger().info(f'Resultado do chute: {result.success}')

    def __kick_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.__node.get_logger().info(f'Feedback do chute: {feedback.status}')

    def __check_keyboard_and_publish_cmd(self):
        key = self.__keyboard.getKey()
        
        # --- Ativação, Parada e Chute (ações de um toque) ---
        if key != self.__previous_key:
            if key == ord(' '):
                self.__walking_active = not self.__walking_active
                self.__node.get_logger().info(f"Modo de caminhada {'ATIVADO' if self.__walking_active else 'DESATIVADO'}.")
                if not self.__walking_active:
                    self.__stop_pub.publish(Empty())
                    self.__cmd_vel_pub.publish(Twist())
                    self.__last_twist_cmd = Twist()
            
            elif key == ord('C'):
                self.__send_kick_goal()
            
            elif key == ord('X'):
                if self.__walking_active:
                    self.__node.get_logger().info("Comando de PARADA enviado.")
                    self.__stop_pub.publish(Empty())
                    self.__cmd_vel_pub.publish(Twist())
                    self.__last_twist_cmd = Twist()
                    self.__walking_active = False
        
        self.__previous_key = key

        # --- Lógica de Movimento Contínuo ---
        if not self.__walking_active:
            return

        cmd = Twist()
        if key in [ord('W'), ord('S'), ord('A'), ord('D'), ord('Q'), ord('E'), -1]:
            if key == ord('W'): cmd.linear.x = self.WALK_VELOCITY
            elif key == ord('S'): cmd.linear.x = -self.WALK_VELOCITY
                
            if key == ord('A'): cmd.linear.y = self.WALK_VELOCITY
            elif key == ord('D'): cmd.linear.y = -self.WALK_VELOCITY
                
            # Q: Anti-horário (positivo em Z)
            if key == ord('Q'): cmd.angular.z = self.TURN_VELOCITY
            # E: Horário (negativo em Z)
            elif key == ord('E'): cmd.angular.z = -self.TURN_VELOCITY

            if cmd != self.__last_twist_cmd:
                self.__cmd_vel_pub.publish(cmd)
                self.__last_twist_cmd = cmd

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.__check_keyboard_and_publish_cmd()