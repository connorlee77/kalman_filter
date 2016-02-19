import numpy as np
import matplotlib.pyplot as plt
import KalmanFilter as kf 
import data.fakeData as fakeData

if __name__ == '__main__':

	accelerations = fakeData.generateCurve((3, 200))
	true_velocity, noisy_velocity = fakeData.genVelocity((3, 200))


	dimension = 3
	A = np.identity(dimension)
	P = np.identity(dimension)
	Q = np.identity(dimension)
	R = np.identity(dimension)

	R = np.dot(0.1, R)

	kf = kf.KalmanFilter(A, P, R, Q, dimension)

	i = 0
	state = []
	while(i < len(true_velocity[0])):
		kf.predictState(accelerations[:,i], 1)
		kf.getKalmanGain()
		kf.update(noisy_velocity[:,i])
		i += 1
		state.append(kf.v_k)

	state = np.transpose(np.array(state))

	for x in range(dimension):
		plt.plot(state[x],label='estimated velocity')
		#plt.plot(noisy_velocity[x], label='noisy velocity')
		plt.plot(true_velocity[x], label='true velocity')
		plt.plot(accelerations[x], label='measured acceleration')
		break
	plt.legend()
	plt.show()