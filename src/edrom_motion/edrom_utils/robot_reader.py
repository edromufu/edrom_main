#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    GroupSyncRead,
    COMM_SUCCESS,
    DXL_LOBYTE,
    DXL_HIBYTE,
    DXL_LOWORD,
    DXL_HIWORD
)

ADDR_MX_TORQUE_ENABLE = 64
ADDR_MX_GOAL_POSITION = 116
LEN_MX_GOAL_POSITION = 4
ADDR_MX_PRESENT_POSITION = 132
LEN_MX_PRESENT_POSITION = 4
CENTER_VALUE = 2048 

class DirectController(Node):
    def __init__(self):
        super().__init__('direct_controller')

        # --- CONFIGURAÇÃO ---
        usb_port = "/dev/ttyUSB0" # VERIFIQUE SE É USB0 OU USB1
        baud_rate = 1000000
        self.amplitude = 5.0 * (math.pi / 180.0) 
        self.frequency = 0.5 
        self.start_time = time.time()

        self.joint_names = [
            "l_sho_pitch", 
            "r_sho_pitch", 
            "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
            "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll"
        ]

        self.motors_config = {
            "l_sho_pitch": {"id": 7,  "protocol": 2.0, "inverted": False, "calibration_offset": 0.0},
            "r_sho_pitch": {"id": 8,  "protocol": 2.0, "inverted": True,  "calibration_offset": 0.0},
            "r_hip_yaw":   {"id": 10, "protocol": 2.0, "inverted": False, "calibration_offset": -0.02},
            "r_hip_roll":  {"id": 12, "protocol": 2.0, "inverted": True,  "calibration_offset": 0.0},
            "r_hip_pitch": {"id": 14, "protocol": 2.0, "inverted": False, "calibration_offset": 0.0},
            "r_knee":      {"id": 16, "protocol": 2.0, "inverted": True,  "calibration_offset": -0.0},
            "r_ank_pitch": {"id": 18, "protocol": 2.0, "inverted": True,  "calibration_offset": 0.0},
            "r_ank_roll":  {"id": 20, "protocol": 2.0, "inverted": False, "calibration_offset": -0.01},
            "l_hip_yaw":   {"id": 9,  "protocol": 2.0, "inverted": False, "calibration_offset": 0.0},
            "l_hip_roll":  {"id": 11, "protocol": 2.0, "inverted": True,  "calibration_offset": -0.0},
            "l_hip_pitch": {"id": 13, "protocol": 2.0, "inverted": True,  "calibration_offset": 0.02},
            "l_knee":      {"id": 15, "protocol": 2.0, "inverted": False, "calibration_offset": -0.25},
            "l_ank_pitch": {"id": 17, "protocol": 2.0, "inverted": True,  "calibration_offset": -0.02},
            "l_ank_roll":  {"id": 19, "protocol": 2.0, "inverted": False, "calibration_offset": -0.0}
        }

        # Inicialização da porta
        self.portHandler = PortHandler(usb_port)
        self.packetHandlerV2 = PacketHandler(2.0)
        self.groupSyncWriteV2 = GroupSyncWrite(self.portHandler, self.packetHandlerV2, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
        self.groupSyncReadV2  = GroupSyncRead(self.portHandler, self.packetHandlerV2, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)

        if not self.portHandler.openPort():
            self.get_logger().error(f"Não foi possível abrir a porta {usb_port}")
            exit()
        
        if not self.portHandler.setBaudRate(baud_rate):
            self.get_logger().error(f"Erro ao setar baudrate {baud_rate}")
            exit()

        # Ativar motores
        for name in self.joint_names:
            mid = self.motors_config[name]['id']
            # Tenta ligar o torque e verifica se houve resposta
            res, err = self.packetHandlerV2.write1ByteTxRx(self.portHandler, mid, ADDR_MX_TORQUE_ENABLE, 1)
            if res != COMM_SUCCESS:
                self.get_logger().warn(f"Motor {mid} ({name}) não respondeu ao comando de Torque.")
            
            self.groupSyncReadV2.addParam(mid)

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states_feedback', 10)
        
        # Timers
        self.timer_read = self.create_timer(0.04, self.read_joints_callback) # 25Hz para teste mais estável
        self.timer_write = self.create_timer(0.04, self.control_loop_callback)
        
        self.get_logger().info("Nó iniciado! Verifique se o tópico /joint_states_feedback tem dados.")

    def control_loop_callback(self):
        elapsed = time.time() - self.start_time
        target_rad = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed)
        self.groupSyncWriteV2.clearParam()
        
        for name in self.joint_names:
            config = self.motors_config[name]
            pos_to_send = (target_rad * (-1.0 if config['inverted'] else 1.0)) + config['calibration_offset']
            goal_value = int(CENTER_VALUE + (pos_to_send * (4095.0 / (2.0 * math.pi))))
            
            param_goal = [DXL_LOBYTE(DXL_LOWORD(goal_value)), DXL_HIBYTE(DXL_LOWORD(goal_value)), 
                          DXL_LOBYTE(DXL_HIWORD(goal_value)), DXL_HIBYTE(DXL_HIWORD(goal_value))]
            self.groupSyncWriteV2.addParam(config['id'], param_goal)
        
        self.groupSyncWriteV2.txPacket()

    def read_joints_callback(self):
        dxl_comm_result = self.groupSyncReadV2.txRxPacket()
        
        # Se falhar, vamos imprimir o porquê
        if dxl_comm_result != COMM_SUCCESS:
            # self.get_logger().debug(f"Falha na leitura: {self.packetHandlerV2.getTxRxResult(dxl_comm_result)}")
            pass # Não damos return aqui para tentar publicar o que for possível

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for name in self.joint_names:
            config = self.motors_config[name]
            mid = config['id']
            
            if self.groupSyncReadV2.isAvailable(mid, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION):
                raw_value = self.groupSyncReadV2.getData(mid, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
                
                # Conversão
                rad = (float(raw_value) - CENTER_VALUE) * (2.0 * math.pi / 4095.0)
                rad -= config['calibration_offset']
                if config['inverted']: rad *= -1.0
                
                msg.name.append(name)
                msg.position.append(rad)

        # SE TIVER PELO MENOS UM MOTOR LIDO, PUBLICA
        if len(msg.name) > 0:
            self.joint_state_pub.publish(msg)
        else:
            # Se cair aqui, nenhum motor está a responder na porta serial
            self.get_logger().warn("Leitura falhou: Nenhum motor disponível no bus.", throttle_duration_sec=2.0)

    def destroy_node(self):
        self.portHandler.closePort()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DirectController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()