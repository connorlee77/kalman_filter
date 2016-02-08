import numpy as np
import matplotlib.pyplot as plt

class KalmanFilter:

	def __init__(self, A, P, dimension):
		self.A = A
		self.P = P
		self.v_k = 0
		self.kalmanGain = 0
		self.R = np.zeros((dimension, dimension))
		self.H = np.identity(dimension)
		self.dimensions = dimension

	def predictState(self, accel, time, Q):
		self.v_k = np.add(
			np.dot(self.A, v_k),
			np.dot(time, accel))

		self.P = np.add(np.dot(
			np.dot(self.A, self.P),
			np.transpose(self.A)), Q)		

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
		self.v_k = np.add(
			self.v_k, 
			np.dot(
				self.kalmanGain,
				np.subtract(
					z_k, 
					np.dot(
						self.H,
						self.v_k))))
		
		self.P = np.dot(
			np.subtract(
				np.identity(self.dimension), 
				np.dot(
					self.kalmanGain, self.H)),
			self.P)


if __name__ == '__main__':
	dimension = 3
	A = np.identity(dimension)
	P = np.identity(dimension)

	kf = KalmanFilter(A, P, dimension)

	### TODO ###
	# while(True):
	# 	kf.predictState(accel, timeDiff, Q)
	# 	kf.getKalmanGain()
	# 	kf.update(prediction)	if new velocity data comes in
