# coding=utf-8
import cv2 as cv
import numpy as np

class FieldGenerator():

    # Atributos que definem as dimensões e características do campo.
    fieldLenght = 675       # Comprimento do campo
    fieldWidth = 450        # Largura do campo
    goalDepth = 60          # Profundidade do gol
    goalWidth = 260         # Largura do gol
    goalAreaDepth = 60      # Profundidade da área do gol
    goalAreaWidth = 300     # Largura da área do gol
    penaltySpot = 135       # Ponto de Penalty
    centralCircle = 110     # Diâmetro do Círculo central
    padding = 100           # Margem ao redor do campo de futebol.
    penaltyMArkDist = 135   # Distância da marca do penalty
    penaltyAreaDepth = 0    # Profundidade da área de Penalty
    penaltyAreaWidth = 0    # Largura da área de Penalty
    lineWidth = 5           # Largura da linha

    # Possuem uma tupla contendo as coordenadas (x, y), um nome e um código.
    # 0 = meio do campo, 1 = marca de penalty, 2 = L, 3 = T, 4 = trave esquerda, 5 = trave direita, 6 = círculo central
    nwField = [(padding,padding),'nwField',2]
    neField = [(padding+fieldLenght,padding),'neField',2]
    swField = [(padding,padding+fieldWidth),'swField',2]
    seField = [(padding+fieldLenght,padding+fieldWidth),'seField',2]

    middleN = [(int(padding+fieldLenght/2),padding),'midleN',3]
    middleS = [(int(padding+fieldLenght/2),padding+fieldWidth),'midleS',3]
    middle = [(int(padding+fieldLenght/2),int(padding+fieldWidth/2)),'middle',0]

    NCenterCircle = [(int(padding+fieldLenght/2),int(padding+(fieldWidth+centralCircle)/2)),'upCenterCircle',6]
    SCenterCircle = [(int(padding+fieldLenght/2),int(padding+(fieldWidth-centralCircle)/2)),'downCenterCircle',6]

    nwLGoalArea = [(padding,int(padding+fieldWidth/2-goalAreaWidth/2)),'nwLGoalArea',3]
    neLGoalArea = [(padding+goalAreaDepth,int(padding+fieldWidth/2-goalAreaWidth/2)),'neLGoalArea',2]
    swLGoalArea = [(padding,int(padding+fieldWidth/2+goalAreaWidth/2)),'swLGoalArea',3]
    seLGoalArea = [(padding+goalAreaDepth,int(padding+fieldWidth/2+goalAreaWidth/2)),'seLGoalArea',2]

    nwRGoalArea = [(padding+fieldLenght-goalAreaDepth,int(padding+fieldWidth/2-goalAreaWidth/2)),'nwRGoalArea',2]
    neRGoalArea = [(padding+fieldLenght,int(padding+fieldWidth/2-goalAreaWidth/2)),'neRGoalArea',3]
    swRGoalArea = [(padding+fieldLenght-goalAreaDepth,int(padding+fieldWidth/2+goalAreaWidth/2)),'swRGoalArea',2]
    seRGoalArea = [(padding+fieldLenght,int(padding+fieldWidth/2+goalAreaWidth/2)),'seRGoalArea',3]

    nwLGoal = [(padding-goalDepth,int(padding+fieldWidth/2-goalWidth/2)),'nwLGoal']
    neLGoal = [(padding,int(padding+fieldWidth/2-goalWidth/2)),'neLGoal',4]
    seLGoal = [(padding,int(padding+fieldWidth/2+goalWidth/2)),'seLGoal',5]

    nwRGoal = [(padding+fieldLenght,int(padding+fieldWidth/2-goalWidth/2)),'nwRGoal',5]
    swRGoal = [(padding+fieldLenght,int(padding+fieldWidth/2+goalWidth/2)),'swRGoal',4]
    seRGoal = [(padding+fieldLenght+goalDepth,int(padding+fieldWidth/2+goalWidth/2)),'seRGoal']

    LPenaltyMark = [(padding+penaltyMArkDist,int(padding+fieldWidth/2)),'LPenaltyMark',1]
    RPenaltyMark = [(padding+fieldLenght-penaltyMArkDist,int(padding+fieldWidth/2)),'RPenaltyMark',1]

    nwLPenaltyArea = [(padding,int(padding+fieldWidth/2-penaltyAreaWidth/2)),'nwLPenaltyArea',3]
    neLPenaltyArea = [(padding+penaltyAreaDepth,int(padding+fieldWidth/2-penaltyAreaWidth/2)),'neLPenaltyArea',2]
    swLPenaltyArea = [(padding,int(padding+fieldWidth/2+penaltyAreaWidth/2)),'swLPenaltyArea',3]
    seLPenaltyArea = [(padding+penaltyAreaDepth,int(padding+fieldWidth/2+penaltyAreaWidth/2)),'seLPenaltyArea',2]

    nwRPenaltyArea = [(padding+fieldLenght-penaltyAreaDepth,int(padding+fieldWidth/2-penaltyAreaWidth/2)),'nwRPenaltyArea',2]
    neRPenaltyArea = [(padding+fieldLenght,int(padding+fieldWidth/2-penaltyAreaWidth/2)),'neRPenaltyArea',3]
    swRPenaltyArea = [(padding+fieldLenght-penaltyAreaDepth,int(padding+fieldWidth/2+penaltyAreaWidth/2)),'swRPenaltyArea',2]
    seRPenaltyArea = [(padding+fieldLenght,int(padding+fieldWidth/2+penaltyAreaWidth/2)),'seRPenaltyArea',3]

    fieldIntersections = [
        nwField, neField, swField, seField,
        middleN, middleS,
        nwLGoalArea, neLGoalArea, swLGoalArea, seLGoalArea,
        nwRGoalArea, neRGoalArea, swRGoalArea, seRGoalArea,
        neLGoal, seLGoal, nwRGoal, swRGoal,
        LPenaltyMark,RPenaltyMark,
        middle,
        NCenterCircle, SCenterCircle
    ]

    # --- ADIÇÃO: Coordenadas das 4 posições de início legais ---
    startPos1 = [(padding + 100), (padding + fieldWidth/2), 0]
    startPos2 = [(padding + 200), (padding + fieldWidth/4), 0]
    startPos3 = [(padding + 200), (padding + 3*fieldWidth/4), 0]
    startPos4 = [(int(padding + fieldLenght/2 - 80)), (int(padding + fieldWidth/2)), 0]
    allStartPos = [startPos1, startPos2, startPos3, startPos4]

    @staticmethod
    def generate():
        field = np.zeros((FieldGenerator.padding*2+FieldGenerator.fieldWidth,
                          FieldGenerator.padding*2+FieldGenerator.fieldLenght), dtype=np.uint8)
        
        cv.rectangle(field, FieldGenerator.nwField[0], FieldGenerator.seField[0], 255, 1)
        cv.line(field, FieldGenerator.middleN[0], FieldGenerator.middleS[0], 255, 1)
        cv.circle(field, FieldGenerator.middle[0], int(FieldGenerator.centralCircle/2), 255, 1)
        cv.rectangle(field, FieldGenerator.nwLGoalArea[0], FieldGenerator.seLGoalArea[0], 255, 1)
        cv.rectangle(field, FieldGenerator.nwRGoalArea[0], FieldGenerator.seRGoalArea[0], 255, 1)
        cv.rectangle(field, FieldGenerator.nwLGoal[0], FieldGenerator.seLGoal[0], 255, 1)
        cv.rectangle(field, FieldGenerator.nwRGoal[0], FieldGenerator.seRGoal[0], 255, 1)
        cv.rectangle(field, FieldGenerator.nwLPenaltyArea[0], FieldGenerator.seLPenaltyArea[0], 255, 1)
        cv.rectangle(field, FieldGenerator.nwRPenaltyArea[0], FieldGenerator.seRPenaltyArea[0], 255, 1)

        field = cv.dilate(field, np.ones((FieldGenerator.lineWidth,FieldGenerator.lineWidth)), iterations=1)
        return field
        
    @staticmethod
    def drawInField(field):
        coloredField = cv.cvtColor(field.astype('uint8'),cv.COLOR_GRAY2BGR)
        for intersection in FieldGenerator.fieldIntersections:
            if len(intersection) < 3: continue
            if intersection[2] == 2: color = [0,255,0]
            elif intersection[2] == 3: color = [0,0,255]
            elif intersection[2] in (4,5): color = [255,0,0]
            else: color = [139,0,139]
            cv.circle(coloredField,intersection[0],5,color,-1) # -1 para preencher o círculo
        return coloredField
    
    @staticmethod
    def drawParticles(coloredField, particles, drawFov=False, fov=0, minRange=0, maxRange=3, neckAngle=0):
        for particle in particles:
            # A chamada para drawParticle permanece a mesma
            coloredField = FieldGenerator.drawParticle(coloredField,(particle[0],particle[1],particle[2],neckAngle),fov,minRange,maxRange,drawFov=drawFov)
        return coloredField
    
    @staticmethod
    def drawParticle(coloredField, particle, fov, minRange, maxRange, drawFov=True, color=[255,0,0], robo=False):
        size = 3 if robo else 2
        
        # --- CORREÇÃO: Converte coordenadas float para int ANTES de usar no OpenCV ---
        center_x = int(particle[0])
        center_y = int(particle[1])
        
        # Usa as coordenadas convertidas para desenhar o círculo
        cv.circle(coloredField, (center_x, center_y), size, color, size)

        if drawFov and robo:
            # O código original para desenhar o FOV é complexo e depende de radianos e graus.
            # O importante é que os pontos finais das linhas e o centro da elipse
            # já estão sendo convertidos para int(), o que está correto.
            # A lógica original do FOV pode ser mantida.
            
            # Desenha as linhas do FOV
            p3_rad = np.deg2rad(particle[3]) # neck_angle
            p2_rad = np.deg2rad(particle[2]) # body_angle
            
            # O FOV da câmera é relativo ao ângulo total (corpo + pescoço)
            total_angle_rad = p2_rad + p3_rad
            fov_rad = fov # Assumindo que fov já foi convertido para radianos

            lim_left = total_angle_rad + fov_rad/2
            lim_right = total_angle_rad - fov_rad/2

            p_start_left = (int(center_x + minRange * np.cos(lim_left)), int(center_y + minRange * np.sin(lim_left)))
            p_end_left = (int(center_x + maxRange * np.cos(lim_left)), int(center_y + maxRange * np.sin(lim_left)))
            
            p_start_right = (int(center_x + minRange * np.cos(lim_right)), int(center_y + minRange * np.sin(lim_right)))
            p_end_right = (int(center_x + maxRange * np.cos(lim_right)), int(center_y + maxRange * np.sin(lim_right)))

            cv.line(coloredField, p_start_left, p_end_left, [150,0,0], 1)
            cv.line(coloredField, p_start_right, p_end_right, [150,0,0], 1)
            
            # Desenha os arcos do FOV (usando o centro já convertido para int)
            start_angle_deg = np.rad2deg(lim_right)
            end_angle_deg = np.rad2deg(lim_left)

            cv.ellipse(coloredField, (center_x, center_y), 
                    (minRange, minRange), 0,
                    start_angle_deg, end_angle_deg, 
                    [150,0,0], 1)
            cv.ellipse(coloredField, (center_x, center_y), 
                    (maxRange, maxRange), 0,
                    start_angle_deg, end_angle_deg,
                    [150,0,0], 1)

        # Desenha a linha de direção do corpo do robô
        end_x = int(center_x + size * 5 * np.cos(np.deg2rad(particle[2])))
        end_y = int(center_y + size * 5 * np.sin(np.deg2rad(particle[2])))
        cv.line(coloredField, (center_x, center_y), (end_x, end_y), color, 2)

        return coloredField