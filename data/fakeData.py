import numpy as np 
import matplotlib.pyplot as plt 

def noise(x, sigma, dimension):
	return np.random.normal(x, sigma, dimension)

def generateCurve(dimension):

	def sqrtFunc(data, noise, vert_offset, scale):
		return np.multiply(scale, np.sqrt(data)) + noise + vert_offset

	data = []
	for x in range(dimension[0]):
		temp = np.linspace(0, dimension[1], dimension[1])
		data.append(temp)


	n = noise(0, 10, dimension)
	#n = np.zeros(dimension)
	data = np.array(data)

	transformedData = sqrtFunc(data, n, 0, 1)

	# for x in range(dimension[0]):
	# 	plt.plot(transformedData[x])

	# plt.show()

	return transformedData

def genVelocity(dimension):
	
	def velocity(data):
		return np.add(np.multiply(2 / 3.0, np.power(data, 1.5)), data)

	data = []
	for x in range(dimension[0]):
		temp = np.linspace(0, dimension[1], dimension[1])
		data.append(temp)

	n = noise(0, 10, dimension)
	data = np.array(data)
	vData = velocity(data)
	noisyV = np.add(vData, n)

	# for x in range(dimension[0]):
	# 	plt.plot(noisyV[x])

	# plt.show()

	return vData, noisyV


	
