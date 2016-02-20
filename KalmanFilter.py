import numpy as np
import matplotlib.pyplot as plt

class KalmanFilter:

	def __init__(self, A, P, R, Q, H, dimension):
		self.A = A
		self.P = P 
		self.x_k = np.array([0, 0, 0])
		self.kalmanGain = 0
		self.R = R #constant
		self.Q = Q #constant
		self.H = H
		self.dimensions = dimension

	def predictState(self, u_k, B):

		self.x_k = np.add(
			np.dot(self.A, self.x_k),
			np.dot(B, u_k))

		self.P = np.add(np.dot(
			np.dot(self.A, self.P),
			np.transpose(self.A)), self.Q)		

	def getKalmanGain(self):
		first = np.dot(self.P, np.transpose(self.H)) 
		second = np.linalg.inv(
			np.add(
				np.dot(
				np.dot(self.H, self.P), 
				np.transpose(self.H)),
				self.R))

		self.kalmanGain = np.dot(first, second)

	def update(self, z_k):
		residual = np.subtract(
					z_k, 
					np.dot(
						self.H,
						self.x_k))

		self.x_k = np.add(
			self.x_k, 
			np.dot(
				self.kalmanGain,
				residual))
		
		self.P = np.dot(
			np.subtract(
				np.identity(self.dimensions), 
				np.dot(
					self.kalmanGain, 
					self.H)),
			self.P)



