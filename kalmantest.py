'''Caltech UAV
#for specific definitions of our variables (A,Q,R,H, etc.) look at the google doc
An interesting note is that the noise of the acceleration data pretty much dictates 
how bad/good the predictions are. The noise of the velocity seems to have little effect 
on correct predictions.'''

import numpy as np
import matplotlib.pyplot as plt
import random
from math import sqrt
import pylab

'''Define and Implement the Kalman Filter'''
class KalmanFilter:

	def __init__(self, A, P, Q, R, H, x_k, dimension):
		self.A = A
		self.P = P
		self.x_k = x_k
		self.kalmanGain = 0
		self.R = R
		self.H = H
		self.Q = Q
		self.dimensions = dimension

	#in our case u_k represnets the input measured acceleration from the IMU
	#we use this measurement to predict the velocity
	def predictState(self, u_k, B):
		self.x_k = np.add(
			np.dot(self.A, self.x_k),
			np.dot(B, u_k))

		self.P = np.add(np.dot(
			np.dot(self.A, self.P),
			np.transpose(self.A)), self.Q)		

	#compute the Kalman Gain
	def getKalmanGain(self):
		first = np.dot(self.P, np.transpose(self.H)) 
		second = np.linalg.pinv(
			np.add(
				np.dot(
				np.dot(self.H, self.P), 
				np.transpose(self.H)),
				self.R))

		self.kalmanGain = np.dot(first, second)

	#in our case z_k represents the measuered velocity from OpenCV
	#we compare our predicted velocity to this
	def update(self, z_k):
		self.x_k = np.add(
			self.x_k, 
			np.dot(
				self.kalmanGain,
				np.subtract(
					z_k, 
					np.dot(
						self.H,
						self.x_k))))
		
		self.P = np.dot(
			np.subtract(
				np.identity(self.dimensions), 
				np.dot(
					self.kalmanGain, self.H)),
			self.P)


'''Define and create test data'''
#class to create test data
class Measure:
    def __init__(self, _trueVelocity, _noiseLevel):
        self.trueVelocity = _trueVelocity
        self.noiseLevel = _noiseLevel
    def SetVelocity(self, update):
        self.trueVelocity = update
    def GetVelocity(self):
        return self.trueVelocity
    def GetVelocityWithNoise(self):
        return random.gauss(self.GetVelocity(),self.noiseLevel)


numsteps = 500 #pretend this is number of seconds
noise = 0.1 #this changes the noise in the acceleration data, the filter only works with low noise

imu = Measure(1.25,0.75)

measuredState = []
trueState = []
measuredAccel = []
currVel = 0;

#create all data
#we make the velocity a sqrt function and "take the derivative" to get acceleration
#guassian noise is added to the data
for i in range(numsteps):
    newVel = sqrt(i)
    accel = newVel - currVel
    currVel = newVel
    measuredAccel.append(random.gauss(accel, noise))

    imu.SetVelocity(sqrt(i))
    measured = imu.GetVelocityWithNoise()
    measuredState.append(measured)
    trueState.append(imu.GetVelocity())

accelerations = measuredAccel
true_velocity = trueState
noisy_velocity = measuredState

'''Begin the Kalman Filter predictions using our test data'''

#we set our constants per the equations in the google doc
dimension = 1
A = np.identity(dimension)
P = np.identity(dimension)
R = np.zeros(dimension)
H = np.identity(dimension)
Q = np.zeros(dimension)
x_k = np.zeros(dimension) #initial guess

kf = KalmanFilter(A, P, Q, R, H, x_k, dimension)

i = 0
state = []
while(i < numsteps):
	B = 1 #this should be the change in time, here we will assume constant 1 second intervals between data readings
	kf.predictState(accelerations[i], B)
	kf.getKalmanGain()
	kf.update(noisy_velocity[i])
	i += 1
	state.append(kf.x_k)

state = np.transpose(np.array(state))


'''Plot the results'''
lines_measured = pylab.plot(measuredState, color = 'y')
lines_filtered = pylab.plot(state[0], color = "k")
lines_true = pylab.plot(trueState, color = "r")
lines_measured_accel = pylab.plot(measuredAccel, color = "b")
pylab.legend((lines_true[0], lines_filtered[0], lines_measured[0], lines_measured_accel[0]), ('true velocity', 'filtered velocity', 'measured velocity', 'measured acceleration'), loc = "lower right")
pylab.show()
