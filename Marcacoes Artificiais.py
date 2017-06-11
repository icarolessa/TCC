#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Importante Bibliotecas
import rospy
import actionlib
import numpy as np
import cv2
import cv
import zbar
import tf
import math

# Reimportando bibliotecas
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3, Twist
from tf import *#TransformListener, tf_conversions
from tf.transformations import * #euler_from_quaternion

bridge = CvBridge()
border2 = 5

# Sistema de Navegacao por Marcacoes Artificiais
class MarcacaoArtificial():
    # Inicialização de variáveis
    destino = None
    qrcodelido = None
    leitura_ativada = False
    goal = None
    
    # Função para fazer o reconhecimento das marcações artificiais
    def callback(self, msg):
        
        # Convertendo a mensagem ROS para o padrão OpenCV
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Criando um retângulo vermelho
        cv2.rectangle(cv2_img, (290, 290), (500, 500), (0,0,255), 2)
        
        # Usando método da Região de Interesse
        tela_ori = cv2_img[200:590, 200:590]
        qr_tela = cv2_img[290:500, 290:500]
        
        # Condição para o funcionamento da câmera
        if self.leitura_ativada is True:
            # Declarando e definindo a variável do escaneamento
            scanner = zbar.ImageScanner()
            scanner.parse_config('enable')
            
            # Convertendo a imagem para a escala de cinza
            gray = cv2.cvtColor(qr_tela, cv2.COLOR_BGR2GRAY, dstCn=0)
            
            # Passando as posições dos pixels da imagem para um vetor
            frame = np.array(gray)
            # Definindo a altura e largura da imagem
            width, height = frame.shape[1], frame.shape[0]

            raw = frame.tostring()
            
            # Chamando a biblioteca Zbar para ler a imagem
            image = zbar.Image(width, height, 'Y800', raw)
            
            # Passando o escaner a procura das marcações artificiais
            scanner.scan(image)
            
            qrc = None
            
            # Laço para armazenar os símbolos encontrados
            for symbol in image:
                qrc = symbol.data
            
            # Condição para quando for diferente de vazio, armazenar o símbolo em uma variável
            if qrc != None:
                self.qrcodelido = qrc
        
        # Desativando a câmera
        self.leitura_ativada = False
        
        # Visão do robô
        cv2.imshow("Sistema de Navegação por Marcações Artificiais",tela_ori)
        cv2.waitKey(1)

    # Função para atualizar Odometria
    def atualiza_odom(self, msg):
        odom.pose.pose.position.x = float(posicoes[1]) + 0.109509);
        odom.pose.pose.position.y = (float(posicoes[2]) - 1.60);
        __init__(self)
    
    # Função para fazer a comunicação com o nó da câmera
    def qr_main(self):
        rospy.Subscriber("/camera/image_raw", Image, self.callback)

    # Função para ler o arquivo de texto externo
    def lerArquivo(self, arquivo, numlinha):
        pontos = open(arquivo, "r")
        linhas = pontos.readlines()
        pontos.close()
        
        # Condição para retornar a quantidade de elementos da linha
        if numlinha < len(linhas):
            # Criando um vetor com os elementos da linha
            return linhas[numlinha]
        
        # Retorna vazio
        else:
            return None    
    
    # Função para buscar as marcações artificiais
    def __init__(self):
        
        # Criando um nó
        rospy.init_node('nav_tcc', anonymous=False)
        
        # Inicializando comunicação com o nó da rede ROS
        self.qr_main()
        
        # Quando acontecer algum erro ou o programa for cancelado
        rospy.on_shutdown(self.shutdown)
        
        # Declarando variável de ação
        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        # Mensagem de conexão
        rospy.loginfo("Conectando ao robô")
        
        # Espera até 5 segundos para ocorrer a conexão
        move_base.wait_for_server(rospy.Duration(5))

        numlinha = 0
        # Lendo o arquivo das posicoes das marcações artificiais
        arquivo = "/home/icaro/catkin_ws/src/tcc/posicoes/posicoes.txt"
        
        # Inicializando variaveis de ações e conexões
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'odom'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        
        # Enquanto o destino for diferente de exit
        while self.destino != "exit":
            # Solicita ao usuario um destino (laboratório)
            self.destino = raw_input("Escreva o laboratorio: ")
            
            # Se destino diferente de exit
            if self.destino != "exit":
                
                # Verifica se existe algum informação no arquivo de texto
                posicoes = self.lerArquivo(arquivo, numlinha)
                
                # Inicializando variáveis
                laboratorio = None
                numlinha = 0
                
                # Caso o arquivo de texto não esteja vazio
                # E o destino for diferente de laboratório... Isso quer dizer que se for igual o robô está no laboratório e não precisa andar
                while posicoes != None and laboratorio != self.destino:
                    
                    # Ler a linha do arquivo de posições novamente
                    posicoes = self.lerArquivo(arquivo, numlinha)
                    
                    # split() "quebra" os dados da linha e armazenando o nome do laboratório
                    # A quebra ocorre a cada espaço entre os dados.
                    laboratorio = posicoes.split(" ")
                    laboratorio = laboratorio[0]
                    
                    # Incrementa numlinha para que a próxima linha possa ser lida
                    numlinha = numlinha + 1
                    
                # Verificando se a posicao do qrcode existe
                if posicoes != None:
                    
                    # Faz a quebra das informações da linha
                    posicoes = posicoes.split(" ")
                    
                    # As informações quebradas na linha 177 foram armazenadas em posições diferentes representando X e Y
                    # Depois disso é feito o cálculo para a distância entre o robô e a marcação artificial
                    self.goal.target_pose.pose.position.x = (float(posicoes[1]) + 0.109509);
                    self.goal.target_pose.pose.position.y = (float(posicoes[2]) - 1.60);
                    self.goal.target_pose.pose.orientation.w = 1.0;
                    
                    # Move o robô para a posição da marcação artificial
                    move_base.send_goal(self.goal)
                    
                    # Instrução para verificar o tempo da tarefa
                    success = move_base.wait_for_result(rospy.Duration(60))
                    
                    # Condição para testar o sucesso da tarefa e ativação da câmera e chamada do método para ler a marcação
                    if not success:
                        move_base.cancel_self.goal()
                        rospy.loginfo("Falha na trajetoria")
        
                    else:
                        self.leitura_ativada = True
                        self.qr_main()
                
                # Condição para testar se a marcação existe ou se foi cadastrada e atualização da odometria
                # A atualização ocorre se e somente se a marcação for a requisitada
                if self.qrcodelido != self.destino:
                    print("Qr Code não encontrado ou não cadastrado")
                
                else:
                    print "Estou no laboratorio '%s'" % self.qrcodelido
                    atualiza_odom()
    
    # Desligando o robô
    def shutdown(self):
        rospy.loginfo("Robô desligado")

if __name__ == '__main__':
    try:
        MarcacaoArtificial()
	
    except rospy.ROSInterruptException:
        rospy.loginfo("Falha")
