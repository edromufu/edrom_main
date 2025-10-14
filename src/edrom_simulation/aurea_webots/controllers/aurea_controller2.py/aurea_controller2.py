import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState  # Mudamos a mensagem para JointState
from controller import Robot

class ArmPositionController:
    def init(self, webots_node, properties):
        """
        Esta função é chamada uma vez quando o controlador é iniciado.
        """
        # Inicializa o rclpy para que possamos criar um nó ROS 2
        rclpy.init(args=None)

        # Inicializa a API do robô do Webots
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())
        
        # Cria o nó ROS 2
        # O nome do nó agora é mais descritivo para o braço
        self.__node = rclpy.create_node('aurea_arm_position_controller')
        self.__node.get_logger().info("Controlador de Posição do Braço para Aurea iniciado.")

        # --- OBTENÇÃO DOS MOTORES DO BRAÇO ---
        # !!! IMPORTANTE: Substitua estes nomes pelos nomes exatos dos MOTORES
        # definidos no seu robô no Webots (campo "name").
        joint_names = [
            'l_sho_pitch',   # Exemplo para o ombro
            'l_el'        # Exemplo para o cotovelo
        ]
        
        # Usamos um dicionário para armazenar os motores. É mais flexível.
        self.__motors = {}
        for name in joint_names:
            motor = self.__robot.getDevice(name)
            if motor is None:
                self.__node.get_logger().error(f"Motor do braço '{name}' não foi encontrado no robô!")
            else:
                self.__motors[name] = motor
                self.__node.get_logger().info(f"Motor '{name}' encontrado com sucesso.")

        # --- SUBSCRIBER ROS 2 ---
        # Criamos um subscriber que escuta no tópico '/arm_joint_commands'
        # Quando uma mensagem chega, a função __joint_command_callback é chamada.
        self.__node.create_subscription(
            JointState,
            'arm_joint_commands',
            self.__joint_command_callback,
            1  # QoS (Quality of Service)
        )
        self.__node.get_logger().info("Aguardando comandos de posição no tópico '/arm_joint_commands'...")

    def __joint_command_callback(self, msg):
        """
        Esta função é chamada toda vez que uma mensagem JointState chega.
        """
        # Itera sobre os nomes e posições recebidos na mensagem
        for i, name in enumerate(msg.name):
            # Verifica se o nome da junta recebida corresponde a um motor que conhecemos
            if name in self.__motors:
                position = msg.position[i]
                
                # A mágica acontece aqui: comanda o motor para a posição desejada
                self.__motors[name].setPosition(position)
                
                self.__node.get_logger().info(f"Comando recebido: Mover junta '{name}' para {position:.2f} rad")
            else:
                # Alerta se receber um comando para uma junta que não está na nossa lista
                self.__node.get_logger().warn(f"Recebido comando para uma junta desconhecida: '{name}'")

    def step(self):
        """
        Esta função é chamada repetidamente em cada passo da simulação.
        """
        # Processa qualquer chamada pendente do ROS 2 (essencial para o subscriber funcionar)
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        # A lógica de controle foi movida para o callback,
        # então o método 'step' fica bem simples.
