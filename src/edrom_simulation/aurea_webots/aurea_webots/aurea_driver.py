import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from controller import Robot

class AureaJointController:
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

        joint_names = [
            'head_pan', 'head_tilt',
            'l_sho_pitch', 'l_sho_roll', 'l_el',
            'r_sho_pitch', 'r_sho_roll', 'r_el',
            'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee', 'r_ank_pitch', 'r_ank_roll',
            'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch', 'l_ank_roll', 
        ]
        
        # Usamos um dicionário para armazenar os motores.
        self.__motors = {}
        for name in joint_names:
            motor = self.__robot.getDevice(name)
            if motor is None:
                self.__node.get_logger().error(f"Motor '{name}' não foi encontrado no robô!")
            else:
                self.__motors[name] = motor
                self.__node.get_logger().info(f"Motor '{name}' encontrado com sucesso.")

        mCamera = self.__robot.getDevice("camera")
        mCamera.enable(self.__timestep)
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
                
            else:
                # Alerta se receber um comando para uma junta que não está na nossa lista
                self.__node.get_logger().warn(f"Recebido comando para uma junta desconhecida: '{name}'")

    def step(self):
        """
        Esta função é chamada repetidamente em cada passo da simulação.
        """
        # Processa qualquer chamada pendente do ROS 2 
        rclpy.spin_once(self.__node, timeout_sec=0)
