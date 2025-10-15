#!/usr/bin/env python3
# coding=utf-8

class BehaviourParameters:

    def __init__(self):
        self.visionwiseParameters()
        self.behaviourwiseParameters()
        self.movementwiseParameters()

    def movementwiseParameters(self):
        # Tópico do ROS da cabeça
        self.headPositionsTopic = '/u2d2_comm/data2head'

        # Parâmetros dos motores da cabeça
        self.lookingLeftRad = 0.35
        self.lookingRightRad = -0.35
        self.minVerRad2Kick = -1.22

        self.maxSpeedLinearX = 0.05
        self.maxSpeedAngularZ = 0.03

    def behaviourwiseParameters(self):
        # Relacionado à número de vezes que uma variável deve extrapolar certo valor para resetar
        self.timerCountLimit = 20

        self.idle_duration = 1.0  # Duração mínima em segundos para o estado idle_march

        # Tópicos do ROS IMU
        self.imuAccelTopic = '/behaviour/imu_accel'
        self.imuGyroTopic = '/behaviour/imu_gyro'
        self.imuRollTopic = '/behaviour/imu_roll'  # Novo tópico para o roll

        # Parâmetros de avaliação de queda nos três eixos
        self.xGravitySecurity = 6  # Valor absoluto para detecção de queda no eixo x
        self.xSensorFront = 6  # Limite para queda de frente
        self.xSensorBack = -6  # Limite para queda de costas
        self.ySensorLeft = 6  # Limite para queda sobre o lado esquerdo
        self.ySensorRight = -6  # Limite para queda sobre o lado direito

        # Retornos possíveis da interpretação de posição relativa da bola
        self.left = 'Left'
        self.right = 'Right'
        self.center = 'Center'
        self.top = 'Top'
        self.bottom = 'Bottom'

        # Retornos possíveis da interpretação de queda
        self.front = 'Front'
        self.up = 'Up'
        self.back = 'Back'
        self.left_fall = 'Left'  # Novo estado para queda à esquerda
        self.right_fall = 'Right'  # Novo estado para queda à direita

        # Tópico de conversa entre ros_packer e state_machine_receiver
        self.stateMachineTopic = '/sensor_observer/state_machine_vars'

        # Tópico do estado atual da máquina de estados
        self.currentStateTopic = '/transitions_and_states/state_machine'

        # Tópico para publicação do estado de queda
        self.fallStateTopic = '/behaviour/fall_state'  # Novo tópico para o estado de queda

    def visionwiseParameters(self):
        # Tópico do ROS
        self.vision2BhvTopic = '/vision/vision_inference'

        # Parâmetros da câmera
        self.cameraWidth = 640
        self.cameraHeight = 480

        # Parâmetros de interpretação da câmera
        self.xCenterLeftLimit = 4 * self.cameraWidth / 10
        self.xCenterRightLimit = 6 * self.cameraWidth / 10
        self.yCenterBottomLimit = 6 * self.cameraHeight / 10
        self.yCenterTopLimit = 4 * self.cameraHeight / 10

        self.closeSize = 80 * 80
