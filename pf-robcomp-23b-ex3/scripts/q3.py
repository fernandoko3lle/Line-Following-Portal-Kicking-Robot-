#! /usr/bin/env python3
# -*- coding:utf-8 -*-

# from module_aruco import Aruco3d
# from module import ImageModule
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import random
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge,CvBridgeError
import numpy as np
import cv2
from sensor_msgs.msg import LaserScan
import time


""" 
	Para inicializar o mapa, em um terminal digite:
	roslaunch my_simulation corrida_de_obstaculos.launch

	Para utilizar a garra, em um terminal digite:
	roslaunch mybot_description mybot_control2.launch

	Para executar, em um novo terminal digite:
	rosrun pf-robcomp-23b-ex3 q3.py
"""

class Questao3():
	def __init__(self):
		self.rate = rospy.Rate(250) # 250 Hz
		self.point = Point()
		self.point_azul = Point()
		self.point_rosa = Point()
		self.point_amarelo = Point()
		self.start = Point(x = -1000)
		self.kernel = np.ones((5,5),np.uint8)
		self.tempoAleatorio = random.randint(0, 6)
		self.segundos = 0
		self.listaPortal = []
		self.tempoInicial = time.time()
		self.kp = 700
		self.pode_parar = False
		self.comparaContornos = {}
		self.cores = {
			"rosa": ((150,100,100),(180,255,255)),
			"azul": ((90, 100, 100),(110, 255, 255)),
			"amarelo":((25, 50, 70), (35, 255, 255))
		}

		# lower e upper por cor 
		self.lower_hsv_rosa = self.cores["rosa"][0]
		self.upper_hsv_rosa = self.cores["rosa"][1]

		self.lower_hsv_azul = self.cores["azul"][0]
		self.upper_hsv_azul = self.cores["azul"][1]

		self.lower_hsv_amarelo = self.cores["amarelo"][0]
		self.upper_hsv_amarelo = self.cores["amarelo"][1]
		
		# Subscribers
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('/camera/image/compressed',CompressedImage,self.image_callback,queue_size=1,buff_size = 2**24)
		self.odom_sub = rospy.Subscriber("/odom",Odometry,self.odom_callback)

        # Publishers
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist)
		self.ombro=rospy.Publisher("/joint1_position_controller/command",Float64,queue_size = 1)
		self.ombro.publish(-1.5)

		# Maquina de estados
		self.robot_state = "gira"
		self.robot_machine = {
			"gira": self.gira,
			"segue_linha_azul": self.segue_linha_azul,
			'segue_linha_rosa': self.segue_linha_rosa,
			"stop": self.stop,
			"derruba": self.derruba,
			"procura": self.procura
		}


	def image_callback(self, msg: CompressedImage) -> None:
		"""
		Callback function for the image topic
		"""
		try:
			cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		self.color_segmentation_azul(cv_image.copy()) # Processamento da imagem
		self.color_segmentation_rosa(cv_image.copy()) # Processamento da imagem
		self.color_segmentation_amarelo(cv_image.copy()) # Processamento da imagem

		# _, self.resultsAruco = self.detectaAruco(cv_image.copy()) # detecta aruco


	def color_segmentation_rosa(self,bgr: np.ndarray) -> None:

		print("entrou na segmentação rosa")

		hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

		# Define cortes 
		height, width = hsv.shape[:2]
		bottom_half = int(height/8)  # Corta a parte inferior (abaixo de 50%)
		trim_percentage = 0.2  # Corta 20% das extremidades

		trim_pixels = int(width * trim_percentage)
		left_trim = trim_pixels
		right_trim = width - trim_pixels

		# Aplicando o corte
		hsv_cortada = hsv[bottom_half:, left_trim:right_trim]

		# mascara rosa
		mask_rosa = cv2.inRange(hsv, self.lower_hsv_rosa, self.upper_hsv_rosa)
		mask_rosa = cv2.morphologyEx(mask_rosa,cv2.MORPH_OPEN, self.kernel)
		mask_rosa = cv2.morphologyEx(mask_rosa,cv2.MORPH_CLOSE, self.kernel)
		cv2.imshow('mask', mask_rosa)
		cv2.waitKey(1)

		# contorno
		contours_rosa,_ = cv2.findContours(mask_rosa, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		# centro rosa 
		if len(contours_rosa) > 0:
			# find contour with max area
			cnt_rosa = max(contours_rosa, key = lambda x: cv2.contourArea(x))
			
			# Find the center rosa
			M_rosa = cv2.moments(cnt_rosa)
			self.point_rosa.x = int(M_rosa['m10']/M_rosa['m00'])
			self.point_rosa.y = int(M_rosa['m01']/M_rosa['m00'])
			self.point_rosa.z = mask_rosa.shape[1] / 2
		else:
			self.point_rosa.x = -1
			self.point_rosa.y = -1
			self.point_rosa.z = -1


	def color_segmentation_azul(self,bgr: np.ndarray) -> None:

		print("entrou na segmentação azul")

		hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

		# Define cortes 
		height, width = hsv.shape[:2]
		bottom_half = int(height/8)  # Corta a parte inferior (abaixo de 50%)
		trim_percentage = 0.2  # Corta 20% das extremidades

		trim_pixels = int(width * trim_percentage)
		left_trim = trim_pixels
		right_trim = width - trim_pixels

		# Aplicando o corte
		hsv_cortada = hsv[bottom_half:, left_trim:right_trim]

		# mascara azul
		mask_azul = cv2.inRange(hsv, self.lower_hsv_azul, self.upper_hsv_azul)
		mask_azul = cv2.morphologyEx(mask_azul,cv2.MORPH_OPEN, self.kernel)
		mask_azul = cv2.morphologyEx(mask_azul,cv2.MORPH_CLOSE, self.kernel)
		cv2.imshow('mask', mask_azul)
		cv2.waitKey(1)

		# contorno
		contours_azul,_ = cv2.findContours(mask_azul, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		# centro azul 
		if len(contours_azul) > 0:
			# find contour with max area
			cnt_azul = max(contours_azul, key = lambda x: cv2.contourArea(x))
			
			# Find the center azul
			M_azul = cv2.moments(cnt_azul)
			self.point_azul.x = int(M_azul['m10']/M_azul['m00'])
			self.point_azul.y = int(M_azul['m01']/M_azul['m00'])
			self.point_azul.z = mask_azul.shape[1] / 2
		else:
			self.point_azul.x = -1
			self.point_azul.y = -1
			self.point_azul.z = -1

	
	def color_segmentation_amarelo(self,bgr: np.ndarray) -> None:

		hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

		# Define cortes 
		height, width = hsv.shape[:2]
		bottom_half = int(height/2)  # Corta a parte inferior (abaixo de 50%)
		trim_percentage = 0.2  # Corta 20% das extremidades

		trim_pixels = int(width * trim_percentage)
		left_trim = trim_pixels
		right_trim = width - trim_pixels

		# Aplicando o corte
		hsv_cortada = hsv[bottom_half:, left_trim:right_trim]

		# mascara amarela
		mask_amarelo = cv2.inRange(hsv, self.lower_hsv_amarelo, self.upper_hsv_amarelo)
		mask_amarelo = cv2.morphologyEx(mask_amarelo,cv2.MORPH_OPEN, self.kernel)
		mask_amarelo = cv2.morphologyEx(mask_amarelo,cv2.MORPH_CLOSE, self.kernel)
		# cv2.imshow('mask', mask_amarelo)
		# cv2.waitKey(1)
		rospy.loginfo("entrou na segmantação amarela")

		# contorno
		contours_amarelo,_ = cv2.findContours(mask_amarelo, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		# centro amarelo 
		if len(contours_amarelo) > 0:
			# find contour with max area
			cnt_amarelo = max(contours_amarelo, key = lambda x: cv2.contourArea(x))

			# Find the center amarelo
			M_amarelo = cv2.moments(cnt_amarelo)
			self.point_amarelo.x = int(M_amarelo['m10']/M_amarelo['m00'])
			self.point_amarelo.y = int(M_amarelo['m01']/M_amarelo['m00'])
			self.point_amarelo.z = mask_amarelo.shape[1] / 2
		else:
			self.point_amarelo.x = -1
			self.point_amarelo.y = -1
			self.point_amarelo.z = -1

	def odom_callback(self, data: Odometry):
		if self.start.x == -1000:
			self.start = data.pose.pose.position
		self.odom = data
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		self.z = data.pose.pose.position.z
		
		orientation_list = [data.pose.pose.orientation.x,
							data.pose.pose.orientation.y,
							data.pose.pose.orientation.z,
							data.pose.pose.orientation.w]
		self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)


		# print(f"ponto z rosa:{self.point_rosa.z}, ponto z azul{self.point_azul.z}")
		# print(f"vel_angular rosa: {self.twist.angular.z}")


		self.distance= np.sqrt((self.start.x - self.x)**2 + (self.start.y - self.y)**2)

	def get_error_azul(self): 
		self.err = self.point_azul.z - self.point_azul.x
		self.twist.angular.z = self.err / self.kp

		self.distance= np.sqrt((self.start.x - self.x)**2 + (self.start.y - self.y)**2)

	def get_error_rosa(self): 
		self.err = self.point_rosa.z - self.point_rosa.x
		self.twist.angular.z = self.err / self.kp

		self.distance= np.sqrt((self.start.x - self.x)**2 + (self.start.y - self.y)**2)

	def get_error_amarelo(self): 
		self.err = self.point_amarelo.z - self.point_amarelo.x
		self.twist.angular.z = self.err / self.kp

		# self.distancia_amarelo = np.sqrt((self.x - self.point_amarelo.x)**2 + (self.y - self.point_amarelo.y)**2)

	def conta_segundos(self):
		currentTime = time.time() - self.tempoInicial
		self.segundos = currentTime
		rospy.loginfo(currentTime)

	
	def gira(self):
		self.conta_segundos()
		self.twist.angular.z = 0.4
		if self.segundos >= self.tempoAleatorio:
			self.twist.angular.z = 0
			rospy.loginfo(f"giro aleatório de {self.tempoAleatorio} segundos concluído!")
			self.robot_state = 'procura'
		else:
			rospy.loginfo(f'girei {self.segundos} segundos de {self.tempoAleatorio} segundos')


	def procura(self):
		self.distanciaRosa = np.sqrt((self.point_rosa.x - self.x)**2 + (self.point_rosa.y - self.y)**2)
		self.distanciaAzul = np.sqrt((self.point_azul.x - self.x)**2 + (self.point_azul.y - self.y)**2)

		# Verifica se as pistas estão no campo de visão
		if self.point_rosa.x != -1 and self.point_azul.x != -1:
			if self.distanciaRosa < self.distanciaAzul:
				self.twist.angular.z = 0
				self.segue_linha_rosa()
			else:
				self.twist.angular.z = 0
				self.segue_linha_azul()
		elif self.point_rosa.x != -1:
			self.twist.angular.z = 0
			self.segue_linha_rosa()
		elif self.point_azul.x != -1:
			self.twist.angular.z = 0
			self.segue_linha_azul()
		else:
			# Caso nenhuma pista esteja no campo de visão, continue girando ou tomando outra ação
			self.twist.angular.z = 0.4


	def segue_linha_rosa(self):
		self.ombro.publish(-1.5)
		self.get_error_rosa()
		# self.get_error_azul()
		if self.point_amarelo.y > 10 and self.point_amarelo.y < 25:
			self.robot_state = 'derruba'
		else:
			self.twist.linear.x = 0.2
			if self.distance > 0.5:
				self.pode_parar = True

			if self.distance < 0.2 and self.pode_parar is True:
				self.twist.linear.x = 0
				self.twist.angular.z = 0
				self.robot_state = "stop"
		

	def segue_linha_azul(self):
		self.ombro.publish(-1.5)
		# self.get_error_rosa()
		self.get_error_azul()
		if self.point_amarelo.y > 10 and self.point_amarelo.y < 15:
			self.robot_state = 'derruba'
		else:
			self.twist.linear.x = 0.2
			if self.distance > 0.5:
				self.pode_parar = True

			if self.distance < 0.2 and self.pode_parar is True:
				self.twist.linear.x = 0
				self.twist.angular.z = 0
				self.robot_state = "stop"

	def derruba(self):
		self.ombro.publish(1)
		rospy.loginfo("achou portal")
		self.listaPortal.append((self.x, self.y, self.z))
		angulo_inicial = self.yaw
		angulo_alvo = angulo_inicial + np.pi / 2  # Gira 90 graus para a direita
		kp = 2.0  # Ganho proporcional (ajuste conforme necessário)

		while abs(self.yaw - angulo_alvo) > 0.05:
			erro = angulo_alvo - self.yaw
			velocidade_angular = kp * erro
			self.twist.angular.z = np.clip(velocidade_angular, -0.4, 0.4)
			self.cmd_vel_pub.publish(self.twist)
			self.rate.sleep()

		self.twist.angular.z = 0
		self.cmd_vel_pub.publish(self.twist)
		rospy.sleep(1)  # Aguarda um pouco antes de iniciar o retorno

		# Retornar à orientação inicial
		angulo_retorno = angulo_inicial
		while abs(self.yaw - angulo_retorno) > 0.05:
			erro = angulo_retorno - self.yaw
			velocidade_angular = kp * erro
			self.twist.angular.z = np.clip(velocidade_angular, -0.4, 0.4)
			self.cmd_vel_pub.publish(self.twist)
			self.rate.sleep("passei pelo portal")

		self.twist.angular.z = 0
		self.cmd_vel_pub.publish(self.twist)

		self.ombro.publish(1)
		rospy.loginfo("passei pelo portal")
		self.robot_state = 'procura'


	def stop(self):
		self.twist.linear.x = 0
		rospy.loginfo("FIM")
	
	def control(self) -> None:
		'''
		Esta função é chamada pelo menos em {self.rate} Hz.
		Esta função controla o robô.
		'''
		
		self.twist = Twist()
		print(f'self.robot_state: {self.robot_state}')
		self.robot_machine[self.robot_state]()

		self.cmd_vel_pub.publish(self.twist)

		self.rate.sleep()


def main():
	rospy.init_node('q3')
	control = Questao3()
	rospy.sleep(0.05)

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()