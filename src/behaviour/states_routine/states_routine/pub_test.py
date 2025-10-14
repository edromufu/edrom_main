#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from modularized_bhv_msgs.msg import StateMachineMsg, CurrentStateMsg
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub_state_machine = self.create_publisher(
            StateMachineMsg, '/sensor_observer/state_machine_vars', qos_profile)
        
        #self.pub_current_state = self.create_publisher(
        #    CurrentStateMsg, '/transitions_and_states/state_machine', qos_profile)

        # publica a cada 1 segundo
        self.timer = self.create_timer(1.0, self.publish_messages)



    def publish_messages(self):
        # Mensagem de queda (exemplo: caiu de costas)
        fall_msg = StateMachineMsg()
        fall_msg.ball_position = "center"
        fall_msg.ball_close = False
        fall_msg.ball_found = False
        fall_msg.fall_state = "back"
        fall_msg.hor_motor_out_of_center = "ok"
        fall_msg.head_kick_check = False

        # Mensagem de estado (getting_up)
        state_msg = CurrentStateMsg()
        state_msg.current_state = "getting_up"

        self.pub_state_machine.publish(fall_msg)
        self.pub_current_state.publish(state_msg)

        self.get_logger().info(f"Mensagens publicadas: fall_state={fall_msg.fall_state}, current_state={state_msg.current_state}")

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
