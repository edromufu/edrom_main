#!/usr/bin/env python3
# coding=utf-8

'''
Nó ROS2 que recebe dados da IMU (acelerômetro, giroscópio, roll)
e interpreta esses dados para determinar o estado de queda do robô (em pé, caído).
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import os
import sys

edrom_dir = '/home/' + os.getlogin() + '/edromufu/src/'
sys.path.append(edrom_dir + 'behaviour/transitions_and_states/src')
from behaviour_parameters import BehaviourParameters

class FallInterpreter(Node):

    def __init__(self):
        """
        Construtor:
        - Inicializa o nó ROS2.
        - Cria os subscribers e o publisher.
        - Define e inicializa variáveis do código.
        """
        # 1. Inicializa a classe base Node
        super().__init__('fall_interpreter_node')
        self.get_logger().info('Nó interpretador de queda iniciado.')

        self.parameters = BehaviourParameters()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Melhor esforço para dados de sensor
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 2. Cria os subscribers para os tópicos da IMU
        self.accel_sub = self.create_subscription(
            Vector3,
            self.parameters.imuAccelTopic,
            self.callback_sensor_accel,
            qos_profile)

        self.gyro_sub = self.create_subscription(
            Vector3,
            self.parameters.imuGyroTopic,
            self.callback_sensor_gyro,
            qos_profile)
            
        self.roll_sub = self.create_subscription(
            Vector3, # Mantido como Vector3 conforme o código original
            self.parameters.imuRollTopic,
            self.callback_sensor_roll,
                qos_profile)
        
        # 3. Cria o publisher para o estado de queda
        self.fall_pub = self.create_publisher(PoseStamped, self.parameters.fallStateTopic, qos_profile)

        # Variáveis de estado do sistema de detecção de queda
        self.fallState = self.parameters.up
        self.countFalled = 0
        self.accel_data = Vector3()
        self.gyro_data = Vector3()
        self.roll = 0.0


    def getValues(self):
        """
        -> Output: Estado da queda
        """
        return self.fallState

    def callback_sensor_accel(self, msg):
        self.accel_data = msg
    
        self.evaluate_fall()

    def callback_sensor_gyro(self, msg):
        self.gyro_data = msg

    def callback_sensor_roll(self, msg):
        self.roll = msg.x

    def evaluate_fall(self):        
        # Assume que yGravitySecurity existe em seus parâmetros
        yGravitySecurity = self.parameters.yGravitySecurity if hasattr(self.parameters, 'yGravitySecurity') else self.parameters.xGravitySecurity

        if abs(self.accel_data.x) > self.parameters.xGravitySecurity or abs(self.accel_data.y) > yGravitySecurity:
            self.countFalled += 1
        else:
            if self.fallState != self.parameters.up:
                self.fallState = self.parameters.up
                # Publica o novo estado "em pé"
                self.publish_fall_state()
            self.countFalled = 0

        if self.countFalled > self.parameters.timerCountLimit:
            previous_state = self.fallState
            
            if self.accel_data.x < self.parameters.xSensorBack:
                self.fallState = self.parameters.back
            elif self.accel_data.x > self.parameters.xSensorFront:
                self.fallState = self.parameters.front
            elif self.accel_data.y < self.parameters.ySensorRight:
                self.fallState = self.parameters.right
            elif self.accel_data.y > self.parameters.ySensorLeft:
                self.fallState = self.parameters.left

            # Publica apenas se o estado de queda mudou
            if self.fallState != previous_state:
                self.get_logger().info(f"Estado de queda detectado: {self.fallState}")
                self.publish_fall_state()

    def publish_fall_state(self):
        fall_msg = PoseStamped()
        
        # 4. Obtém o timestamp atual do relógio do nó ROS2
        fall_msg.header.stamp = self.get_clock().now().to_msg()
        fall_msg.header.frame_id = "base_link" # Boa prática adicionar um frame_id

        # Atribui os dados à mensagem 
        fall_msg.pose.position.x = self.accel_data.x
        fall_msg.pose.position.y = self.accel_data.y
        fall_msg.pose.position.z = self.accel_data.z
        fall_msg.pose.orientation.x = self.gyro_data.x
        fall_msg.pose.orientation.y = self.gyro_data.y
        fall_msg.pose.orientation.z = self.gyro_data.z
        fall_msg.pose.orientation.w = self.roll # Usando o w para o roll como no original

        # 5. Publica a mensagem
        self.fall_pub.publish(fall_msg)

def main(args=None):
    rclpy.init(args=args)
    fall_interpreter_node = FallInterpreter()
    try:
        rclpy.spin(fall_interpreter_node)
    except KeyboardInterrupt:
        pass
    finally:
        fall_interpreter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()