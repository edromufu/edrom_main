import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image 
from controller import Robot

'''
Controller que se conecta com o Webots, lê os dados dos sensores e publica em tópicos ROS.
'''

class AureaSensors:
    def init(self, webots_node, properties):

        # Inicializa o ROS2
        rclpy.init(args=None)

        # Pega o robô do Webots
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Cria o nó ROS2
        self.__node = rclpy.create_node('aurea_sensors')
        self.__node.get_logger().info("Class AureaSensors initialized.")

        # Inicializa o acelerômetro
        self.__accel = self.__robot.getDevice('imu_accel')
        if self.__accel is None:
            self.__node.get_logger().error("Accel not found.")
            return
        self.__accel.enable(self.__timestep)

        # Inicializa o gyro
        self.__gyro = self.__robot.getDevice('imu_gyro')
        if self.__gyro is None:
            self.__node.get_logger().error("Gyro not found.")
            return
        self.__gyro.enable(self.__timestep)

        
        # Inicializa a câmera
        self.__camera = self.__robot.getDevice("camera_front")
        if self.__camera is None:
            self.__node.get_logger().error("Câmera not found.")
            return
        self.__camera.enable(self.__timestep)
        
        # Cria publisher
        self.__accel_pub = self.__node.create_publisher(Vector3, '/imu/accel', 10)
        self.__node.get_logger().info("Accel publishing in /imu/accel.")

        self.__gyro_pub = self.__node.create_publisher(Vector3, '/imu/gyro', 10)
        self.__node.get_logger().info("Gyro publishing in /imu/gyro")

        
        self.__camera_pub = self.__node.create_publisher(Image, '/camera/image', 10)
        self.__node.get_logger().info("Câmera publishing in /camera/image.")
        
    def step(self):
        """
        Esta função é chamada repetidamente em cada passo da simulação.
        """
        # Processa callbacks ROS2
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        if self.__camera is not None:
            image = self.__camera.getImage()
            if image is not None:
                msg = Image()
                msg.data = image
                msg.height = self.__camera.getHeight()
                msg.width = self.__camera.getWidth()
                msg.encoding = "bgra8"  # Webots retorna BGRA por padrão
                msg.is_bigendian = 0
                msg.step = msg.width * 4
                self.__camera_pub.publish(msg)
        
        if self.__accel is not None:
            values = self.__accel.getValues()
            msg = Vector3()
            msg.x, msg.y, msg.z = values
            self.__accel_pub.publish(msg)
        
        if self.__gyro is not None:
            values = self.__gyro.getValues()
            msg = Vector3()
            msg.x, msg.y, msg.z = values
            self.__gyro_pub.publish(msg)
