import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

# Apenas as importações essenciais
from soccer_vision_2d_msgs.msg import Ball, BallArray

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        self.bridge = CvBridge()
        
        # Configuração do pipeline de detecção do OpenCV (>= 4.7.0)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        # Inscrição no tópico da câmera
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        
        # Publicador
        self.aruco_pub = self.create_publisher(
            BallArray, 
            '/vision/aruco_detections', 
            10
        )

    def image_callback(self, msg):
        try:
            # CORREÇÃO 1 e 2: Utilizar bgr8 para o debug funcionar e .copy() para liberar a memória
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()
        except Exception as e:
            self.get_logger().error(f'Falha no cv_bridge: {e}')
            return

        corners, ids, rejected = self.detector.detectMarkers(cv_image)

        # Checagem extra de segurança para tuplas/arrays vazios retornados em algumas versões do OpenCV
        if ids is not None and len(ids) > 0:
            out_msg = BallArray()
            out_msg.header = msg.header 
            
            # Desenha as bordas na cópia alocada da imagem
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            for i in range(len(ids)):
                id_aruco = float(ids[i][0])
                canto = corners[i][3]
                    
                u_centro = float(np.mean(canto[:, 0]))
                v_centro = float(np.mean(canto[:, 1]))
                
                bola = Ball()
                
                # CORREÇÃO 3: Atribuição direta sem necessidade de novos objetos em memória
                bola.center.x = u_centro
                bola.center.y = v_centro
                
                # Hack de calibração alocando os atributos diretamente
                #bola.confidence.known = True
                bola.confidence.confidence = float(id_aruco) 
                
                out_msg.balls.append(bola)

            # Publica a mensagem no tópico
            self.aruco_pub.publish(out_msg)
            
        # Bloco de debug visual
        cv2.imshow('Camera ROS 2 - Aruco Debug', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows() 
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()