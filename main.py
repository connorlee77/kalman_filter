import numpy as np
import matplotlib.pyplot as plt
import KalmanFilter as kf 
import data.fakeData as fakeData

if __name__ == '__main__':

	accelerations = fakeData.generateCurve((3, 1000))
	true_velocity, noisy_velocity = fakeData.genVelocity((3, 1000))

	dimension = 3
	A = np.identity(dimension)
	P = np.identity(dimension)

	kf = kf.KalmanFilter(A, P, dimension)
	Q = np.array([[0,0,0],[0,0,0],[0,0,0]])
	### TODO ###
	i = 0
	state = []
	while(i < len(noisy_velocity[0])):
		kf.predictState(accelerations[:,i], 1, Q)
		kf.getKalmanGain()
		kf.update(noisy_velocity[:,1])
		i += 1
		state.append(kf.v_k)

	state = np.transpose(np.array(state))

	for x in range(dimension):
		plt.plot(state[x])

	plt.show()