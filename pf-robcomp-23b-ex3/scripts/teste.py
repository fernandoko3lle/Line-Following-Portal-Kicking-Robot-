def segue_linha(self):
		self.get_error_rosa()
		self.get_error_azul()
		if self.point_amarelo.x != -1:
			self.distancia_amarelo = np.sqrt((self.x - self.point_amarelo.x)**2 + (self.y - self.point_amarelo.y)**2)
			print(self.distancia_amarelo)
			if self.distancia_amarelo > 1:
				# self.get_error_amarelo()
				self.twist.linear.x = 0.2
				if self.distance > 0.5:
					self.pode_parar = True

				if self.distance < 0.2 and self.pode_parar is True:
					self.twist.linear.x = 0
					self.twist.angular.z = 0
					self.robot_state = "stop"
			else:
				self.robot_state = "derruba"
		else:
			self.get_error_rosa()
			self.get_error_azul()
			self.twist.linear.x = 0.2
			if self.distance > 0.5:
				self.pode_parar = True

			if self.distance < 0.2 and self.pode_parar is True:
				self.twist.linear.x = 0
				self.twist.angular.z = 0
				self.robot_state = "stop"