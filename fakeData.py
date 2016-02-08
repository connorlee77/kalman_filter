import numpy as np 
import matplotlib.pyplot as plt 

def generateSigmoidCurve(dimension):

	def noise(x, sigma, dimension):
		return np.random.normal(x, sigma, dimension)

	def tan(data, noise, vert_offset, scale):
		return np.multiply(scale, np.sqrt(data)) + noise + vert_offset

	data = []
	for x in range(dimension[0]):
		temp = np.linspace(0, 1000, dimension[1])
		data.append(temp)


	noise = noise(20, 5, dimension)
	#noise = np.zeros(dimension)
	data = np.array(data)

	transformedData = tan(data, noise, 100, 0.5)

	# for x in range(dimension[0]):
	# 	plt.plot(transformedData[x])

	# plt.show()

	return transformedData

def genVelocity(dimension):
	


if __name__ == '__main__':

	data = generateSigmoidCurve((3, 1000))
