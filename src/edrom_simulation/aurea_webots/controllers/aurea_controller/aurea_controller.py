#!/usr/bin/env python3
import rclpy
from controller import Robot
from sensor_msgs.msg import JointState

class AureaWebotsController:
    def __init__(self, ros_node=None):
        # Inicializa a API do Robô do Webots
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Inicializa o nó ROS2
        self.node = ros_node if ros_node is not None else rclpy.create_node('aurea_webots_controller')

        # --- Nomes das Juntas (devem ser idênticos aos do URDF/PROTO) ---
        self.joint_names = [
            "head_pan", "head_tilt",
            "l_sho_pitch", "l_sho_roll", "l_el",
            "r_sho_pitch", "r_sho_roll", "r_el",
            "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
            "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll"
        ]

        # --- Inicialização dos Dispositivos ---
        self.motors = {}
        self.position_sensors = {}
        for name in self.joint_names:
            # Motores
            motor = self.robot.getDevice(name)
            motor.setPosition(float('inf')) # Modo de controle de velocidade
            motor.setVelocity(0.0)
            self.motors[name] = motor
            
            # Sensores de Posição (Encoders)
            sensor_name = name + "_sensor" # Webots geralmente adiciona "_sensor"
            sensor = self.robot.getDevice(sensor_name)
            sensor.enable(self.timestep)
            self.position_sensors[name] = sensor

        # --- Configuração dos Tópicos ROS2 ---
        # Publicador para o estado atual das juntas (para o RViz)
        self.joint_state_publisher = self.node.create_publisher(JointState, '/joint_states', 10)
        
        # Subscriber para receber os comandos de ângulos desejados
        self.joint_goal_subscriber = self.node.create_subscription(
            JointState, '/goal_joint_states', self.goal_callback, 10)
        
        self.node.get_logger().info("Controlador do robô 'Aurea' para Webots iniciado.")

    def goal_callback(self, msg: JointState):
        """Callback para mover os motores quando um alvo é recebido."""
        for i, name in enumerate(msg.name):
            if name in self.motors:
                self.motors[name].setPosition(msg.position[i])

    def step(self):
        """Avança um passo na simulação e publica o estado dos sensores."""
        # Avança a simulação do Webots
        if self.robot.step(self.timestep) == -1:
            return False # Simulação encerrou
        
        # Cria a mensagem de estado das juntas
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Lê a posição de cada sensor e a adiciona à mensagem
        positions = []
        for name in self.joint_names:
            positions.append(self.position_sensors[name].getValue())
        msg.position = positions

        # Publica a mensagem para o RViz e outros nós
        self.joint_state_publisher.publish(msg)
        
        return True

def main(args=None):
    rclpy.init(args=args)
    # Este nó é apenas para inicializar o controlador
    # A lógica principal roda dentro da classe
    node = rclpy.create_node('aurea_controller_starter')
    controller = AureaWebotsController(ros_node=node)
    
    # Loop principal que roda a cada passo da simulação
    while controller.step():
        rclpy.spin_once(node, timeout_sec=0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
