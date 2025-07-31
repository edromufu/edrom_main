#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
from sensor_msgs.msg import JointState
import numpy as np
import os

# --- OFFSETS DA POSE EM T (SUBSTITUA PELOS SEUS VALORES MEDIDOS) ---
T_POSE_ENCODER_OFFSETS = {
    'r_hip_yaw': 2045,
    'r_hip_roll': 2005,
    'r_hip_pitch': 2108,
    'r_knee': 1977,
    'r_ank_pitch': 2004,
    'r_ank_roll': 2047,
}

# --- PASSO 1: IDENTIFICAR OS MOTORES INVERTIDOS ---
# Adicione o NOME da junta que tem a rotação invertida a esta lista.
# Use um 'set' para uma busca mais rápida.
INVERTED_JOINT_NAMES = {'r_hip_pitch'} # Exemplo: Apenas o motor do joelho é invertido

class DynamixelNode(Node):
    def __init__(self):
        super().__init__('dynamixel_node')
        # ... (Configurações dos motores como antes) ...
        self.PROTOCOL_VERSION    = 2.0
        self.BAUDRATE            = 1000000
        self.DEVICENAME          = '/dev/ttyUSB0'
        self.ADDR_TORQUE_ENABLE  = 64
        self.ADDR_GOAL_POSITION  = 116
        self.ADDR_PRESENT_POSITION = 132
        self.DXL_MAX_POSITION_VALUE = 4095

        # Mapeamento de nomes de junta para IDs
        self.joint_name_to_id = {
            "r_hip_yaw": 6, "r_hip_roll": 8, "r_hip_pitch": 10,
            "r_knee": 12, "r_ank_pitch": 14, "r_ank_roll": 16
        }
        self.id_to_joint_name = {v: k for k, v in self.joint_name_to_id.items()}

        # Inicializa a comunicação com a SDK
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        if not self.portHandler.openPort() or not self.portHandler.setBaudRate(self.BAUDRATE):
            self.get_logger().error("Falha ao conectar aos Dynamixels.")
            rclpy.shutdown()
            return

        self.get_logger().info("Porta serial dos Dynamixels aberta com sucesso.")
        self.enable_all_torques()

        # Subscriber para receber os ângulos desejados
        self.goal_joint_state_sub = self.create_subscription(
            JointState, '/goal_joint_states', self.goal_callback, 10)

    # --- PASSO 2: MODIFICAR A FUNÇÃO DE CONVERSÃO ---
    def angle_to_position(self, angle_rad, joint_name):
        """Converte um ângulo do ROS (radianos) para um valor de encoder absoluto."""
        
        # Se o nome da junta estiver na nossa lista de inversão, inverte o sinal do ângulo.
        if joint_name in INVERTED_JOINT_NAMES:
            angle_rad = -angle_rad

        # O resto da lógica de conversão continua igual
        angle_as_ticks = (angle_rad / (2 * np.pi)) * self.DXL_MAX_POSITION_VALUE
        offset_ticks = T_POSE_ENCODER_OFFSETS.get(joint_name, 2048)
        final_position = int(offset_ticks + angle_as_ticks)
        
        return max(0, min(final_position, self.DXL_MAX_POSITION_VALUE))

    def goal_callback(self, msg: JointState):
        """Recebe uma mensagem JointState e comanda os motores para as posições."""
        self.get_logger().info("Recebido novo alvo de juntas para o hardware.")
        for i, name in enumerate(msg.name):
            if name in self.joint_name_to_id:
                dxl_id = self.joint_name_to_id[name]
                # A função de conversão agora lida com a inversão automaticamente
                goal_pos = self.angle_to_position(msg.position[i], name)
                
                res, err = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_GOAL_POSITION, goal_pos)
                if res != 0 or err != 0:
                    self.get_logger().warn(f"Falha ao mover {name} (ID: {dxl_id})")
                else:
                    self.get_logger().info(f"Movendo {name} (ID: {dxl_id}) para a posição de encoder {goal_pos}")

    def enable_all_torques(self):
        for dxl_id in self.joint_name_to_id.values():
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, 1)

    def on_shutdown(self):
        self.get_logger().info("Desabilitando torques e fechando porta serial.")
        for dxl_id in self.joint_name_to_id.values():
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, 0)
        self.portHandler.closePort()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()