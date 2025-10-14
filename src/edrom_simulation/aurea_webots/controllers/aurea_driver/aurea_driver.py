import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from controller import Robot

class AureaJointController:  # Mudei o nome da classe para ser mais genérico
    def init(self, webots_node, properties):
        """
        Esta função é chamada uma vez quando o controlador é iniciado.
        """
        # Inicializa o rclpy para que possamos criar um nó ROS 2
        rclpy.init(args=None)

        # Inicializa a API do robô do Webots
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())
        
        # Cria o nó ROS 2 com um nome mais genérico
        self.__node = rclpy.create_node('aurea_joint_controller')
        self.__node.get_logger().info("Controlador de Juntas para Aurea iniciado.")

        # --- LISTA COMPLETA DE MOTORES DO ROBÔ ---
        # Esta é a sua lista, agora controlando o corpo inteiro.
        # Garanta que estes nomes são EXATAMENTE os mesmos do campo "name"
        # dos seus dispositivos RotationalMotor no Webots.
        joint_names = [
            'l_sho_pitch', 'l_el', 'l_knee',
            'r_sho_pitch', 'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 
            'r_knee', 'r_ank_pitch', 'r_ank_roll'
            # Se tiver mais motores, adicione-os aqui.
        ]
        
        # Usamos um dicionário para armazenar os motores.
        self.__motors = {}
        for name in joint_names:
            motor = self.__robot.getDevice(name)
            if motor is None:
                # Mensagem de erro mais genérica
                self.__node.get_logger().error(f"Motor '{name}' não foi encontrado no robô!")
            else:
                self.__motors[name] = motor
                self.__node.get_logger().info(f"Motor '{name}' encontrado com sucesso.")

        # --- SUBSCRIBER ROS 2 ---
        # O subscriber escuta no tópico '/goal_joint_states' que você definiu.
        self.__node.create_subscription(
            JointState,
            '/goal_joint_states',
            self.__joint_command_callback,
            1  # QoS (Quality of Service)
        )
        self.__node.get_logger().info("Aguardando comandos de posição no tópico '/goal_joint_states'...")

    def __joint_command_callback(self, msg):
        """
        Esta função é chamada toda vez que uma mensagem JointState chega.
        """
        # Itera sobre os nomes e posições recebidos na mensagem
        for i, name in enumerate(msg.name):
            # Verifica se o nome da junta recebida corresponde a um motor que conhecemos
            if name in self.__motors:
                position = msg.position[i]
                
                # Comanda o motor correspondente no Webots para ir para a posição
                self.__motors[name].setPosition(position)
                
                # Opcional: Log para confirmar o comando
                # self.__node.get_logger().info(f"Movendo junta '{name}' para {position:.2f} rad")
            else:
                # Alerta se receber um comando para uma junta que não está na nossa lista
                self.__node.get_logger().warn(f"Recebido comando para uma junta desconhecida: '{name}'")

    def step(self):
        """
        Esta função é chamada repetidamente em cada passo da simulação.
        """
        # Processa qualquer chamada pendente do ROS 2 (essencial para o subscriber funcionar)
        rclpy.spin_once(self.__node, timeout_sec=0)
