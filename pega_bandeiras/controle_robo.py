import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from scipy.spatial.transform import Rotation as R

from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum

# O ROBO TRANSITA ENTRE OS 16 ESTADOS POSSIVEIS:
class ESTADOS(Enum):
    EXPLORANDO = 1
    BANDEIRA_ENCONTRADA = 2
    INDO_PARA_BANDEIRA = 3
    POSICIONANDO_NA_BANDEIRA = 4
    DESVIANDO = 5
    ABRINDO_GARRA = 6
    PEGANDO_BANDEIRA = 7
    FECHANDO_GARRA = 8
    VOLTANDO_PARA_BASE = 9
    LEVANTANDO_GARRA = 10
    DESVIANDO_VOLTANDO_PARA_BASE = 11
    PARADO = 12
    ABAIXANDO_GARRA = 13
    ABRINDO_GARRA_FINAL = 14
    DANDO_RE = 15
    FECHANDO_GARRA_FINAL = 16


class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')

        # Publisher para comando de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher para comando da garra
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)
        self.create_subscription(
            Image, '/robot_cam/colored_map', self.camera_callback, 10)

        # Utilizado para converter imagens ROS -> OpenCV
        self.bridge = CvBridge()

        # Timer para enviar comandos continuamente
        self.timer = self.create_timer(0.1, self.move_robot)

        # Contador para evitar loop infinito
        self.maximo_loops = 90
        self.contador = self.maximo_loops

        # Estado interno
        self.obstaculo_a_frente = False             # Define se existe algo a menos de uma certa distancia usando o LIDAR
        self.direcao_explorando = 0                 # Define a direcao para a qual esta desviano no momento para o estado EXPLORANDO
        self.direcao_obstaculo = 0                  # 1 -> direita e -1 -> esquerda
        self.percentual_bandeira = -1               # Diz a porcentagem que a bandeira ocupa na camera
        self.menor_distancia_frente_esquerda = -1   # Angulo da menor distancia a frente esquerda
        self.menor_distancia_esquerda = -1          # Angulo da menor distancia a esquerda
        self.menor_distancia_frente_direita = -1    # Angulo da menor distancia a frente direita
        self.menor_distancia_direita = -1           # Angulo da menor distancia a direita
        self.bandeira_a_frente = False              # Vira true quando a bandeira esta no centro da camera
        self.estado_atual = ESTADOS.EXPLORANDO      # Estado atual do robo na maquina de estados
        self.chegou_na_bandeira = False             # Vira true quando uma certa portagem da bandeira ocupa a tela
        self.giro_desvio = 0                        # Define se o robo esta girando para desvio de algum obstaculo
        self.val_menor_distancia_lados = -1         # Valor da menor distancia encontrada ao lado do robo

        self.const_dist_obstaculo = 0.8             # Constante que define a distancia minima para um obstaculo

        self.pos_x_bandeira = -1                    # Posicao horizontal da bandeira na tela
        self.pos_x_mastro = -1                      # Posicao horizontal do mastro da bandeira na tela
        self.distancia_frente = None                # Menor distancia que o robo enxerga num angulo de 30°
        self.ultima_posx_bandeira = -1              # Ultima posicao da bandeira vista na tela (caso saia da camera)
        self.centro_x = -1                          # Centro horizontal da imagem (tamanho horizontal dividido por 2)
        self.dx = 20                                # Tolerancia em pixels para considerar a bandeira no centro da imagem
        self.dx_mastro = 3                          # Tolerancia em pixels para considerar o mastro da bandeira no centro da imagem

        self.contador_garra = -1                    # Contador utilizado em alguns momentos para movimentos da garra
        self.bandeira_coletada = 0                  # Variavel que define se a bandeira esta coletada ou nao

        self.base_x = 0                             # Coordenada x da base (posicao inicial do robo)
        self.base_y = 0                             # Coordenada y da base (posicao inicial do robo)
        self.base_salvada = False                   # Variavel auxiliar para definir se a coordenada da base ja foi salva

    # FUNCAO QUE RECEBE OS DADOS DO LIDAR
    def scan_callback(self, msg: LaserScan):
        # Verifica uma faixa estreita ao redor de 0° (frente)
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        indices_frente_direita = list(range(330, 360))      # Indices que estao ate 30° a direita
        indices_frente_esquerda = list(range(0, 31))        # Indices que estao ate 30° a esquerda
        indices_direita = list(range(240, 360))             # Indices que estao ate 120° a direita
        indices_esquerda = list(range(0, 120))              # Indices que estao ate 120° a esquerda

        distancias_frente_esquerda = [msg.ranges[i] for i in indices_frente_esquerda]   # Distancias para ate 30° a esquerda
        distancias_frente_direita = [msg.ranges[i] for i in indices_frente_direita]     # Distancias para ate 30° a direita

        #Loop que calcula o angulo da menor distancia a esquerda
        dist_ladoesq = 100000
        self.menor_distancia_esquerda = -1
        for i in range(0 + self.bandeira_coletada,120):
            if msg.ranges[i] < dist_ladoesq:
                dist_ladoesq = msg.ranges[i]
                self.menor_distancia_esquerda = i  

        #Loop que calcula a menor distancia a direita
        dist_ladodir = 100000
        self.menor_distancia_direita = -1
        for i in range(240,360 - self.bandeira_coletada):
            if msg.ranges[i] < dist_ladodir:
                dist_ladodir = msg.ranges[i]
                self.menor_distancia_direita = i

        if dist_ladodir < dist_ladoesq:
            self.val_menor_distancia_lados = dist_ladodir
        else:
            self.val_menor_distancia_lados = dist_ladoesq

        #Loop que calcula o angulo da menor distancia a frente esquerda
        dist_esq = 100000
        self.menor_distancia_frente_esquerda = -1
        for i in range(0 + self.bandeira_coletada,30):
            if msg.ranges[i] < dist_esq:
                dist_esq = msg.ranges[i]
                self.menor_distancia_frente_esquerda = i
        
        #Loop que calcula a menor distancia a frente direita
        dist_dir = 100000
        self.menor_distancia_frente_direita = -1
        for i in range(330,360 - self.bandeira_coletada):
            if msg.ranges[i] < dist_dir:
                dist_dir = msg.ranges[i]
                self.menor_distancia_frente_direita = i

        # Caso observe obstaculos em ambos os lados:
        if distancias_frente_direita and dist_dir < self.const_dist_obstaculo and distancias_frente_esquerda and dist_esq < self.const_dist_obstaculo:
            # Se a menor distancia estiver a esquerda:
            if dist_esq < dist_dir:
                # Define que o obstaculo esta a esquerda
                self.direcao_obstaculo = -1
                self.distancia_frente = dist_esq
            # Caso a menor distancia estiver a direita
            else:
                # Define que o obstaculo esta a direita
                self.direcao_obstaculo = 1
                self.distancia_frente = dist_dir
            #Define que existe obstaculo a frente
            self.obstaculo_a_frente = True

        # Caso so hajam obstaculos a esquerda
        elif distancias_frente_esquerda and dist_esq < self.const_dist_obstaculo:
            # Define que o obstaculo esta a esquerda
            self.distancia_frente = dist_esq
            self.obstaculo_a_frente = True
            self.direcao_obstaculo = -1

        # Caso so hajam obstaculos a direita
        elif distancias_frente_direita and dist_dir < self.const_dist_obstaculo:
            # Define que o obstaculo esta a direita
            self.distancia_frente = dist_dir
            self.obstaculo_a_frente = True
            self.direcao_obstaculo = 1

        # Caso nao hajam obstaculos
        else:
            # Define a variavel que indica se existem obstaculos como False
            self.obstaculo_a_frente = False

    def imu_callback(self, msg: Imu):
        pass

    # FUNCAO QUE RECEBE OS VALORES DA ODOMETRIA
    def odom_callback(self, msg: Odometry):
        # Recebe as coordenadas x e y da odometria
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Recebe a orientacao inicial do robo
        q = msg.pose.pose.orientation

        # Calcula o angulo a partir da orientacao
        theta = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # Salva as coordenadas nas variaveis internas do robo
        self.x = x
        self.y = y
        self.theta = theta

        # Caso seja a primeira medicao da odometria, salva a posicao como a base
        if not self.base_salvada:
            self.base_x = x
            self.base_y = y
            self.base_theta = theta
            self.base_salvada = True
            print(self.base_x, self.base_y)

    # Funcao que abre a garra 
    def abrir_garra(self):
        msg = Float64MultiArray()
        msg.data = [0.0, -0.06, 0.06]
        self.gripper_pub.publish(msg)

    # Funcao que fecha a garra
    def fechar_garra(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.gripper_pub.publish(msg)

    # Funcao que levanta a garra
    def levantar_garra(self):
        msg = Float64MultiArray()
        msg.data = [-0.3, 0.0, 0.0]
        self.gripper_pub.publish(msg)

    # Funcao que abaixa a garra
    def abaixar_garra(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.gripper_pub.publish(msg)

    # FUNCAO QUE RECEBE OS DADOS DO LIDAR
    def camera_callback(self, msg: Image):
        # Converte mensagem ROS para imagem OpenCV (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Obtem o tamanho da imagem e determina o centro horizontal da imagem
        h, w = frame.shape[:2]
        self.centro_x = w // 2

        # Cor da bandeira (BGR)
        target_color = np.array([227, 73, 0])

        # Máscara
        mask = cv2.inRange(frame, target_color, target_color)

        # Quantos pixels da imagem pertencem à bandeira
        pixels_bandeira = cv2.countNonZero(mask)
        area_imagem = h * w
        self.percentual_bandeira = pixels_bandeira / area_imagem

        # Detecta contornos
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # Define a variavel bandeira_a_frente como True caso haja algum contorno da cor da bandeira na imagem
        self.bandeira_a_frente = len(contours) > 0

        # Valores padrão
        if self.pos_x_bandeira != None: self.ultima_posx_bandeira = self.pos_x_bandeira
        self.pos_x_bandeira = None
        self.area_bandeira = self.percentual_bandeira

        # Caso exista algum contorno da bandeira
        if contours:
            # Seleciona apenas o maior blob
            maior_contorno = max(contours, key=cv2.contourArea)

            M = cv2.moments(maior_contorno)

            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Define a coordenada x da bandeira sendo o centro do maior blob
                self.pos_x_bandeira = cx

            # Calcula o quanto a bandeira ocupa na imagem
            area_blob = cv2.contourArea(maior_contorno)

            # Define que chegou na bandeira caso essa ocupe mais de 2,5% da imagem
            self.chegou_na_bandeira = self.percentual_bandeira > 0.02
        
        # Caso esteja no estado POSICIONANDO_NA_BANDEIRA
        # Calcula a coordenada x do mastro da bandeira considerando apenas a parte inferior da imagem
        self.pos_x_mastro = -1
        # Define limite inferior (apenas a parte inferior da imagem)
        y_limite = int(h * 2/3)

        # Cria uma máscara apenas da região inferior
        mask_inferior = np.zeros_like(mask)
        mask_inferior[y_limite:h, :] = mask[y_limite:h, :]

        # Quantos pixels da imagem pertencem à bandeira na região inferior
        pixels_bandeira = cv2.countNonZero(mask_inferior)
        area_imagem_inferior = (h - y_limite) * w
        self.percentual_bandeira = pixels_bandeira / area_imagem_inferior

        # Detecta contornos apenas na região inferior
        contours, _ = cv2.findContours(
            mask_inferior,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:
            # Seleciona apenas o maior blob
            maior_contorno = max(contours, key=cv2.contourArea)

            # Metade da altura da imagem
            y_meio = h // 2

            # Mantém apenas os pontos do contorno na metade inferior
            pontos_inferiores = maior_contorno[maior_contorno[:, 0, 1] >= y_meio]

            if len(pontos_inferiores) > 2:  # necessário para formar um contorno
                M = cv2.moments(pontos_inferiores)

                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # Define a coordenada x do mastro sendo o centro do maior blob
                    self.pos_x_mastro = cx
    
    # FUNCAO QUE DEFINE A MAQUINA DE ESTADOS DO ROBO E A SUA MOVIMENTACAO PARA CADA ESTADO
    def move_robot(self):

        # Printa o estado atual do robo no terminal
        if self.estado_atual == ESTADOS.EXPLORANDO: self.get_logger().info("EXPLORANDO")
        elif self.estado_atual == ESTADOS.BANDEIRA_ENCONTRADA: self.get_logger().info("BANDEIRA_ENCONTRADA") 
        elif self.estado_atual == ESTADOS.INDO_PARA_BANDEIRA: self.get_logger().info("INDO_PARA_BANDEIRA") 
        elif self.estado_atual == ESTADOS.POSICIONANDO_NA_BANDEIRA: self.get_logger().info("POSICIONANDO_NA_BANDEIRA")
        elif self.estado_atual == ESTADOS.DESVIANDO: self.get_logger().info("DESVIANDO")
        elif self.estado_atual == ESTADOS.ABRINDO_GARRA: self.get_logger().info("ABRINDO_GARRA")
        elif self.estado_atual == ESTADOS.PEGANDO_BANDEIRA: self.get_logger().info("PEGANDO_BANDEIRA")
        elif self.estado_atual == ESTADOS.FECHANDO_GARRA: self.get_logger().info("FECHANDO_GARRA")
        elif self.estado_atual == ESTADOS.LEVANTANDO_GARRA: self.get_logger().info("LEVANTANDO_GARRA")
        elif self.estado_atual == ESTADOS.VOLTANDO_PARA_BASE: self.get_logger().info("VOLTANDO_PARA_BASE")
        elif self.estado_atual == ESTADOS.DESVIANDO_VOLTANDO_PARA_BASE: self.get_logger().info("DESVIANDO_VOLTANDO_PARA_BASE")
        elif self.estado_atual == ESTADOS.ABAIXANDO_GARRA: self.get_logger().info("ABAIXANDO_GARRA")
        elif self.estado_atual == ESTADOS.ABRINDO_GARRA_FINAL: self.get_logger().info("ABRINDO_GARRA_FINAL")
        elif self.estado_atual == ESTADOS.DANDO_RE: self.get_logger().info("DANDO_RE")
        elif self.estado_atual == ESTADOS.FECHANDO_GARRA_FINAL: self.get_logger().info("FECHANDO_GARRA_FINAL")
        elif self.estado_atual == ESTADOS.PARADO: self.get_logger().info("PARADO")


        twist = Twist()


        # CASO ESTEJA NO ESTADO "EXPLORANDO"
        # Nao sabe onde a bandeira esta, e anda aleatoriamente desviando dos obstaculos ate encontrar
        if self.estado_atual == ESTADOS.EXPLORANDO:
            # Anda pra frente enquanto nao encontrar obstaculos e salva a direcao como zero (frente)
            if not self.obstaculo_a_frente:
                self.direcao_explorando = 0
                twist.linear.x = 0.5  # Move para frente
            # Caso ja esteja desviando para um direcao, continua para ela ate nao haver mais obstaculos
            # Isso evita que o robo fique preso num canto realizando um movimento de vai e volta para os lados
            elif self.direcao_explorando != 0:

                if self.distancia_frente < self.const_dist_obstaculo * 0.63:
                    twist.linear.x = -0.5
                else:
                    twist.angular.z = self.direcao_explorando * 0.3
            # Quando encontra um obstaculo pela primeita vez, gira para desviar e salva a direcao para a qual esta desviando
            else:
                self.direcao_explorando = self.direcao_obstaculo
                twist.angular.z = self.direcao_obstaculo * 0.3  # Gira em torno do proprio eixo

            # Muda para o estado "BANDEIRA_ENCONTRADA" quando reconhece a bandeira
            if self.bandeira_a_frente:
                self.estado_atual = ESTADOS.BANDEIRA_ENCONTRADA


        # CASO ESTEJA NO ESTADO "BANDEIRA_ENCONTRADA"
        # Percebeu visualmente na camera algum pixel da bandeira
        elif self.estado_atual == ESTADOS.BANDEIRA_ENCONTRADA:

            # Muda para o estado "POSICIONANDO_NA_BANDEIRA" quando percebe que chegou na bandeira
            if self.chegou_na_bandeira and self.obstaculo_a_frente:
                self.estado_atual = ESTADOS.POSICIONANDO_NA_BANDEIRA

            # Volta para o estado "EXPLORANDO" caso perca a bandeira no processo
            if self.pos_x_bandeira == None:
                self.estado_atual = ESTADOS.EXPLORANDO

            # Vira para a direita caso a bandeira esteja sendo vista na esquerda do centro da imagem
            elif self.pos_x_bandeira < self.centro_x - self.dx:
                twist.angular.z = 0.3  # Gira em torno do proprio eixo
                twist.linear.x = 0.1
            # Vira para a esquerda caso a bandeira esteja sendo vista na direita do centro da imagem
            elif self.pos_x_bandeira > self.centro_x + self.dx:
                twist.angular.z = -0.3  # Gira em torno do proprio eixo
                twist.linear.x = 0.1
            # Caso a bandeira ja esteja no centro da image, vai para o estado "INDO_PARA_BANDEIRA"
            elif self.pos_x_bandeira <= self.centro_x + self.dx and self.pos_x_bandeira >= self.centro_x - self.dx:
                self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA

        
        # CASO ESTEJA NO ESTADO "INDO_PARA_BANDEIRA"
        # A bandeira esta em vista e ele vai reto em direcao a bandeira
        elif self.estado_atual == ESTADOS.INDO_PARA_BANDEIRA:

            # Muda para o estado "POSICIONANDO_NA_BANDEIRA" quando percebe que chegou na bandeira
            if self.chegou_na_bandeira and self.obstaculo_a_frente:
                self.estado_atual = ESTADOS.POSICIONANDO_NA_BANDEIRA

            # Caso nao tenham obstaculos a frente
            elif not self.obstaculo_a_frente:
                # Caso a bandeira saia do centro da imagem, volta para o estado "BANDEIRA_ENCONTRADA"
                if self.pos_x_bandeira != None and not (self.pos_x_bandeira < self.centro_x + self.dx and self.pos_x_bandeira > self.centro_x - self.dx):
                    self.estado_atual = ESTADOS.BANDEIRA_ENCONTRADA
                    self.giro_desvio = 0                # Tambem define o giro como zero
                    self.contador = self.maximo_loops   # Reseta o contador de loop infinito
                # Caso giro_desvio seja diferente de zero, esta vindo do estado "DESVIANDO"
                # Portanto, deve continuar girando na direcao contraria ao desvio ate enxergar a bandeira novamente
                if self.giro_desvio != 0:
                    # Gira ate encontrar a bandeira novamente e decremente o contador
                    twist.angular.z = self.giro_desvio * 0.3
                    self.contador = self.contador - 1

                    # Caso o contador zere, volta para o estado de "EXPLORANDO"
                    if self.contador == 0:
                        self.giro_desvio = 0
                        self.estado_atual = ESTADOS.EXPLORANDO
                        self.contador = self.maximo_loops

                # Caso contrario, continua andando reto ate a bandeira
                else:
                    self.giro_desvio = 0
                    twist.linear.x = 0.5
            # Caso tenham obstaculos a frente, entra no estado "DESVIANDO"
            else:
                self.estado_atual = ESTADOS.DESVIANDO 

            # Caso nao tenha a bandeira em vista e tambem nao esteja voltando do desviando, volta para explorando
            if not self.bandeira_a_frente and self.giro_desvio == 0:
                self.estado_atual = ESTADOS.EXPLORANDO
        
        
        # CASO ESTEJA NO ESTADO "DESVIANDO"
        # O robo detectou um obstaculo a frente e precisa desviar
        elif self.estado_atual == ESTADOS.DESVIANDO:
            
            # Caso o obstaculo nao esteja mais a frente
            if not self.obstaculo_a_frente:

                # Caso o obstaculo esteja a esquerda
                if self.direcao_obstaculo == -1:
                    # Se a bandeira estiver a vista a direita, volta para o estado de "INDO_PARA_BANDEIRA"
                    if self.pos_x_bandeira != None and self.pos_x_bandeira > self.centro_x + self.dx :
                        self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA
                    # Avalia se a menor distancia a esquerda, ainda esta a menos de 90°, vai pra frente
                    elif self.menor_distancia_esquerda < 90:
                        twist.linear.x = 0.5
                    # Caso ja esteja a mais de 90°
                    else:
                        self.direcao_obstaculo = 0          # Define que nao existe mais obstaculo
                        self.giro_desvio = 1                # Define o giro de desvio para a direita
                        twist.angular.z = 0.3               # Comeca a girar
                        self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA      # Volta para o estado "INDO_PARA_BANDEIRA"

                # Caso o obstaculo esteja a direita
                elif self.direcao_obstaculo == 1:
                    # Se a bandeira estiver a vista a esquerda, volta para o estado de "INDO_PARA_BANDEIRA"
                    if self.pos_x_bandeira != None and self.pos_x_bandeira < self.centro_x - self.dx :
                        self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA
                    # Avalia se a menor distancia a direita, ainda esta a menos de 90°, vai pra frente
                    elif self.menor_distancia_direita > 270:
                        twist.linear.x = 0.5
                    # Caso ja esteja a mais de -90°
                    else:
                        self.direcao_obstaculo = 0          # Define que nao existe mais obstaculo
                        self.giro_desvio = -1               # Define o giro de desvio para a esquerda
                        twist.angular.z = -0.3              # Comeca a girar
                        self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA      # Volta para o estado "INDO_PARA_BANDEIRA"
                        
            # Caso o obstaculo ainda esteja a frente
            else:
                # Gira para o lado contrario ao lado que o obstaculo esta
                if self.direcao_obstaculo == -1:
                    twist.angular.z = -0.3
                elif self.direcao_obstaculo == 1:
                    twist.angular.z = 0.3
                    

        # CASO ESTEJA NO ESTADO "POSICIONANDO_NA_BANDEIRA"
        # O robo esta na frente da bandeira e ajeita para pega-la
        elif self.estado_atual == ESTADOS.POSICIONANDO_NA_BANDEIRA:

            # Caso aconteca de perder o mastro de vista, volta para o estado "EXPLORANDO"
            if self.pos_x_mastro == -1:
                self.estado_atual = ESTADOS.EXPLORANDO
            # Calcula o angulo que faz com a bandeira para garantir que a barra nao bata
            elif self.direcao_obstaculo == -1 and np.cos(self.menor_distancia_frente_esquerda * np.pi /180) * self.distancia_frente < 0.55:
                twist.linear.x = -0.1
            # Calcula o angulo que faz com a bandeira para garantir que a barra nao bata
            elif self.direcao_obstaculo == 1 and np.cos(self.menor_distancia_frente_direita * np.pi /180) * self.distancia_frente < 0.55:
                twist.linear.x = -0.1
            # Caso o mastro esteja a esquerda do centro da imagem, gira para a direita ate alinhar
            elif self.pos_x_mastro < self.centro_x - self.dx_mastro:
                twist.angular.z = 0.1  # Gira em torno do proprio eixo
            # Caso o mastro esteja a direita do centro da imagem, gira para a esquerda ate alinhar
            elif self.pos_x_mastro > self.centro_x + self.dx_mastro:
                twist.angular.z = -0.1  # Gira em torno do proprio eixo
            # Caso esteja distante da bandeira, anda para frente
            elif self.distancia_frente > 0.7:
                twist.linear.x = 0.1
            # Caso esteja muito proximo da bandeira, anda para tras
            elif self.distancia_frente < 0.5:
                twist.linear.x = -0.1
            # Caso o mastro esteja no centro da imagem, fica parado
            else:
                self.estado_atual = ESTADOS.ABRINDO_GARRA
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
        
        # CASO ESTEJA NO ESTADO "ABRINDO_GARRA"
        # Um contador e utilizado de 10 a 0 para garantir que o robo tenha tempo de realizar a determinada funcao
        # Quando o contador zera, o robo passa para o proximo estado
        # Essa logica e utilizada varias vezes daqui pra frente
        elif self.estado_atual == ESTADOS.ABRINDO_GARRA:
            if self.contador_garra == -1:
                self.contador_garra = 10
            elif self.contador_garra == 0:
                self.contador_garra = -1
                # Quando o timer acabar, vai para o estado de pegar a bandeira
                self.estado_atual = ESTADOS.PEGANDO_BANDEIRA    
            else:
                self.contador_garra -= 1
            self.abrir_garra()
        
        # CASO ESTEJA NO ESTADO "PEGANDO_BANDEIRA"
        # Coloca as garas em volta da bandeira para pega-la
        elif self.estado_atual == ESTADOS.PEGANDO_BANDEIRA:
            # Se ainda nao esta perto suficiente, chega mais proximo da bandeira
            if self.distancia_frente > 0.45:                
                twist.linear.x = 0.1
            # Quando esta proximo suficiente, entra no modo de fechar a garra
            else:
                self.estado_atual = ESTADOS.FECHANDO_GARRA  
        
        # CASO ESTEJA NO ESTADO "FECHANDO_GARRA"
        # Fecha a garra, segurando a bandeira
        elif self.estado_atual == ESTADOS.FECHANDO_GARRA:
            if self.contador_garra == -1:
                self.contador_garra = 10
            elif self.contador_garra == 0:
                self.contador_garra = -1
                #Quando o timer acabar, entra no estado para levantar a garra
                self.estado_atual = ESTADOS.LEVANTANDO_GARRA
            else:
                self.contador_garra -= 1
            self.fechar_garra()

        # CASO ESTEJA NO ESTADO "LEVANTANDO_GARRA"
        # Levanta a bandeira pega, podendo levar de volta para a base
        elif self.estado_atual == ESTADOS.LEVANTANDO_GARRA:
            if self.contador_garra == -1:
                self.contador_garra = 10
            elif self.contador_garra == 0:
                self.contador_garra = -1
                # Quando o timer acaba, entra no estado de voltar para a base
                self.estado_atual = ESTADOS.VOLTANDO_PARA_BASE
                self.bandeira_coletada = 10
            else:
                self.contador_garra -= 1
            self.levantar_garra()

        # CASO ESTEJA NO ESTADO "VOLTANDO_PARA_BASE"
        # A posicao da base foi salva no comeco
        # Com a bandeira pega, volta para a base relacionando sua posicao atual com a posicao da base
        elif self.estado_atual == ESTADOS.VOLTANDO_PARA_BASE:
            # Entra no modo de desvio se encontrar um obstaculo no caminho
            if self.obstaculo_a_frente:
                self.estado_atual = ESTADOS.DESVIANDO_VOLTANDO_PARA_BASE
            else:
                # Relaciona sua posicao atual com a da base nos eixos X e Y
                dx = self.base_x - self.x
                dy = self.base_y - self.y
                # Calcula o modulo para saber a distancia entre ele e a base e armazena em uma variavel
                distancia = np.sqrt(dx**2 + dy**2)
                # Caso tenha chego na base
                if distancia < 0.7:
                    # O robo para de se mexer 
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    # Entra no estado de abaixar a garra
                    self.estado_atual = ESTADOS.ABAIXANDO_GARRA
                    return
                # Calcula o angulo com a base e armazena em uma variavel
                angulo_base = np.arctan2(dy, dx)
                # Calcula o erro angular 
                erro_angular = np.arctan2(
                    np.sin(angulo_base - self.theta),
                    np.cos(angulo_base - self.theta)
                )
                if self.val_menor_distancia_lados < 0.3:
                    twist.linear.x = 0.5
                # Primeiro gira, depois anda
                elif abs(erro_angular) > 0.1:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.3 * erro_angular/abs(erro_angular)
                else:
                    twist.linear.x = 0.5
                    twist.angular.z = 0.0

        # CASO ESTEJA NO ESTADO "DESVIANDO_VOLTANDO_PARA_BASE"
        # Desvia do obstaculo da mesma forma que desvia procurando a bandeira
        elif self.estado_atual == ESTADOS.DESVIANDO_VOLTANDO_PARA_BASE:
            dx = self.base_x - self.x
            dy = self.base_y - self.y

            distancia = np.sqrt(dx**2 + dy**2)

            # Caso tenha chego na base
            if distancia < 0.7:
                # O robo para de se mexer
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                # Entra no estado de abaixar a garra
                self.estado_atual = ESTADOS.ABAIXANDO_GARRA
                return

            # Caso o obstaculo nao esteja mais a frente
            if not self.obstaculo_a_frente:

                # Caso o obstaculo esteja a esquerda
                if self.direcao_obstaculo == -1:
                    # Vai pra frente enquanto o obstaculo esta a menos de 90°
                    if self.menor_distancia_esquerda < 90:
                        twist.linear.x = 0.5
                    # Caso ja esteja a mais de 90°
                    else:
                        self.direcao_obstaculo = 0          # Define que nao existe mais obstaculo
                        self.giro_desvio = 1                # Define o giro de desvio para a direita
                        twist.angular.z = 0.3               # Comeca a girar
                        self.estado_atual = ESTADOS.VOLTANDO_PARA_BASE      # Volta para o estado de voltar para a base

                # Caso o obstaculo esteja a direita
                elif self.direcao_obstaculo == 1:
                    # Vai pra frente enquanto o obstaculo esta a mais de -90°
                    if self.menor_distancia_direita > 270:
                        twist.linear.x = 0.5
                    # Caso ja esteja a mais de -90°
                    else:
                        self.direcao_obstaculo = 0          # Define que nao existe mais obstaculo
                        self.giro_desvio = -1               # Define o giro de desvio para a esquerda
                        twist.angular.z = -0.3              # Comeca a girar
                        self.estado_atual = ESTADOS.VOLTANDO_PARA_BASE      # Volta para o estado de voltar para a base
                        
            # Caso o obstaculo ainda esteja a frente
            else:
                # Gira para o lado contrario ao lado que o obstaculo esta
                if self.direcao_obstaculo == -1:
                    twist.angular.z = -0.3
                elif self.direcao_obstaculo == 1:
                    twist.angular.z = 0.3

        # CASO ESTEJA NO ESTADO "ABAIXANDO_GARRA"
        # Depois que chegou na base, abaixa a bandeira para coloca-la
        elif self.estado_atual == ESTADOS.ABAIXANDO_GARRA:
            if self.contador_garra == -1:
                self.contador_garra = 10
            elif self.contador_garra == 0:
                self.contador_garra = -1
                # Quando o timer acaba, entra no estado de abrir a garra
                self.estado_atual = ESTADOS.ABRINDO_GARRA_FINAL
            else:
                self.contador_garra -= 1
            self.abaixar_garra()
        
        # CASO ESTEJA NO ESTADO "ABRINDO_GARRA_FINAL"
        # Com a bandeira colocada, abre a garra para solta-la
        elif self.estado_atual == ESTADOS.ABRINDO_GARRA_FINAL:
            if self.contador_garra == -1:
                self.contador_garra = 10
            elif self.contador_garra == 0:
                self.contador_garra = -1
                # Quando o timer acaba, entra no estado de dar re
                self.estado_atual = ESTADOS.DANDO_RE
            else:
                self.contador_garra -= 1
            self.abrir_garra()
        
        # CASO ESTEJA NO ESTADO "DANDO_RE"
        # Com a bandeira devidamente posicionada na base, da re para poder fechar a garra e terminar a rotina
        elif self.estado_atual == ESTADOS.DANDO_RE:
            self.bandeira_coletada = 0
            # Vai para tras ate a distancia com a bandeira atingir um valor minimo
            if self.distancia_frente < 0.7:
                twist.linear.x = -0.1
            else:
                self.estado_atual = ESTADOS.FECHANDO_GARRA_FINAL

        # CASO ESTEJA NO ESTADO "FECHANDO_GARRA_FINAL"
        # Fecha a garra para concluir sua rotina
        elif self.estado_atual == ESTADOS.FECHANDO_GARRA_FINAL:
            if self.contador_garra == -1:
                self.contador_garra = 10
            elif self.contador_garra == 0:
                self.contador_garra = -1
                # Quando o timer acaba, entra no estado de parado
                self.estado_atual = ESTADOS.PARADO
            else:
                self.contador_garra -= 1
            self.fechar_garra()

        # CASO ESTEJA NO ESTADO "PARADO"
        # Com a rotina terminada, o robo para
        elif self.estado_atual == ESTADOS.PARADO:
            twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()