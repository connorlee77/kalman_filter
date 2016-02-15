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
	Q = np.zeros(dimension)
	R = np.zeros(dimension)

	Q.fill(0.00001)
	R.fill(0.1)

	kf = kf.KalmanFilter(A, P, R, Q, dimension)

	i = 0
	state = []
	while(i < len(noisy_velocity[0])):
		kf.predictState(accelerations[:,i], 1)
		kf.getKalmanGain()
		kf.update(noisy_velocity[:,1])
		i += 1
		state.append(kf.v_k)

	state = np.transpose(np.array(state))

	for x in range(dimension):
		plt.plot(state[x])
		plt.plot(noisy_velocity[x])
		plt.plot(true_velocity[x])
		plt.plot(accelerations[x])

	plt.show()