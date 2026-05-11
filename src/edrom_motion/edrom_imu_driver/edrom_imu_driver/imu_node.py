import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import numpy as np

class ImuSerialDriver(Node):
    def __init__(self):
        super().__init__('imu_serial_driver')
        
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
            self.get_logger().info(f"Conectado ao ESP32 em {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Falha ao conectar na serial: {e}")
            self.ser = None

        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if self.ser is None or not self.ser.is_open:
            return

        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                data = line.split(',')
                
                if len(data) == 6:
                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.frame_id

                    # Aceleração (m/s^2)
                    msg.linear_acceleration.x = float(data[0])
                    msg.linear_acceleration.y = float(data[1])
                    msg.linear_acceleration.z = float(data[2])

                    # Giroscópio (rad/s)
                    msg.angular_velocity.x = float(data[3])
                    msg.angular_velocity.y = float(data[4])
                    msg.angular_velocity.z = float(data[5])
                    
                    msg.orientation_covariance[0] = -1.0

                    self.publisher_.publish(msg)
            
            except ValueError:
                pass 
            except Exception as e:
                self.get_logger().warn(f"Erro no loop: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()