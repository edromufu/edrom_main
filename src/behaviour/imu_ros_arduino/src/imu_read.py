#!/usr/bin/env python3
#coding=utf-8

'''
Leitor de IMU para ROS 2.
Recebe dados via serial e os publica nos tópicos do ROS.
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
from geometry_msgs.msg import Vector3
import os
import sys
import threading

# A linha abaixo não é a forma recomendada de importar módulos em ROS 2.
# O setup.py deve ser configurado para tratar os módulos corretamente.
# edrom_dir = '/home/' + os.getlogin() + '/edromufu/src/'
# sys.path.append(edrom_dir + 'behaviour/transitions_and_states/src')
from behaviour.transitions_and_states.src.behaviour_parameters import BehaviourParameters

class ImuReader(Node):

    def __init__(self):
        # Inicializa o nó ROS 2
        super().__init__('imu_node')
        self.get_logger().info('Nó ImuReader iniciado.')

        self.parameters = BehaviourParameters()
        
        # Configuração QoS para comunicação
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Melhor esforço para dados de sensor
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Criando os publishers para os dados do acelerômetro, giroscópio e roll
        self.accel_pub = self.create_publisher(Vector3, self.parameters.imuAccelTopic, qos_profile)
        self.gyro_pub = self.create_publisher(Vector3, self.parameters.imuGyroTopic, qos_profile)
        self.roll_pub = self.create_publisher(Vector3, self.parameters.imuRollTopic, qos_profile)

        # Inicializando as mensagens
        self.accel_msg = Vector3()
        self.gyro_msg = Vector3()
        self.roll_msg = Vector3()

        # Obtendo o parâmetro da porta serial. Em ROS 2, `get_param` é um método do nó.
        # Se você definiu o parâmetro, pode acessá-lo. Ex: `self.declare_parameter('port', '/dev/ttyUSB0')`
        # self.port = self.get_parameter('port').get_parameter_value().string_value
        
        # O código original usa uma porta fixa, vamos manter para a conversão.
        self.port = '/dev/ttyUSB0'

        # Inicializando a conexão serial
        try:
            self.imu = serial.Serial(self.port, 115200)
            self.get_logger().info(f'Conexão serial com {self.port} estabelecida.')
        except serial.SerialException as e:
            self.get_logger().error(f'Falha ao abrir a porta serial {self.port}: {e}')
            # Se a conexão falhar, o nó não deve continuar.
            raise SystemExit

    def run(self):
        try:
            while rclpy.ok():
                if self.imu.in_waiting > 0:
                    imu_output = self.imu.readline().decode().strip()
                    data = imu_output.split('|')
                    
                    if len(data) == 7:
                        try:
                            # Convertendo os valores para float
                            rollC, AcX, AcY, AcZ, GyX, GyY, GyZ = map(float, data)

                            if -200 < rollC < 200:
                                # Publicando os dados do acelerômetro
                                self.accel_msg.x = AcX
                                self.accel_msg.y = AcY
                                self.accel_msg.z = AcZ
                                self.accel_pub.publish(self.accel_msg)

                                # Publicando os dados do giroscópio
                                self.gyro_msg.x = GyX
                                self.gyro_msg.y = GyY
                                self.gyro_msg.z = GyZ
                                self.gyro_pub.publish(self.gyro_msg)

                                # Publicando o valor de roll
                                self.roll_msg.x = rollC
                                self.roll_pub.publish(self.roll_msg)
                        except (ValueError, IndexError) as e:
                            self.get_logger().warning(f"Dados do serial inválidos: {e}")
                    else:
                        self.get_logger().warning("Linha de dados incompleta.")

        except Exception as e:
            self.get_logger().error(f"Erro ao ler dados da IMU: {e}")
        finally:
            self.imu.close()
            self.get_logger().info('Porta serial fechada.')

def main(args=None):
    rclpy.init(args=args)
    
    imu_reader = ImuReader()
    
    # Cria uma thread separada para a leitura da porta serial
    serial_thread = threading.Thread(target=imu_reader.run)
    serial_thread.daemon = True
    serial_thread.start()

    try:
        rclpy.spin(imu_reader)
    except KeyboardInterrupt:
        pass
    finally:
        imu_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()