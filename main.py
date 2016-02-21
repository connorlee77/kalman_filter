import numpy as np
import matplotlib.pyplot as plt
import KalmanFilter as kf 
import data.fakeData as fakeData

if __name__ == '__main__':

	accelerations = fakeData.generateCurve((3, 200)) #fake data for testing purposes
	true_velocity, noisy_velocity = fakeData.genVelocity((3, 200)) #fake data for testing purposes

	time = time_between_steps #change in time between updates, do we need to update this every time or is it constant?
	time2 = time*time #time squared
	u_k = accelerations[:,i] #this is the measued acceleration from the IMU
	z_k = noisy_velocity[:,i] #this is the measued velocity from OpenCV

	dimension = 4
	A = np.matrix([[1,0,time,0],[0,1,0,time],[0,0,1,0],[0,0,0,1]])
	B = np.matrix([[time2,0],[0,time2],[time,0],[0, time]])
	H = np.matrix([[0,0,1,0],[0,0,0,1]])
	P = np.identity(dimension)
	Q = np.identity(dimension)
	R = np.identity(dimension)

	#tweak covariance matrices
	Q = np.dot(1,Q)
	R = np.dot(0.1, R)

	#create the Kalman Filter instance
	kf = kf.KalmanFilter(A, P, R, Q, H, dimension)

	i = 0
	'''our states should come out with 4 terms in the form Matrix([d_x], [d_y], [v_x],[v_y])'''
	state = []
	while(i < len(true_velocity[0])):
		kf.predictState(u_k, B)
		kf.getKalmanGain()
		kf.update(z_k)
		i += 1
		state.append(kf.x_k)

	state = np.transpose(np.array(state))

	#Plot our results
	for x in range(dimension):
		plt.plot(state[x],label='estimated velocity')
		#plt.plot(noisy_velocity[x], label='noisy velocity')
		plt.plot(true_velocity[x], label='true velocity')
		plt.plot(accelerations[x], label='measured acceleration')
		break
	plt.legend()
	plt.show()
