import numpy as np
import time
class Particle():
	"""class for particles in swarm"""
	def __init__(self):
		self.k = np.random.rand(3)
		self.v = np.random.rand(3)
		self.k[1] *= 0.001
		self.v[1] *= 0.001
		self.k[2] *= 0.01
		self.v[2] *= 0.01
		self.k[0] *= 0.9
		self.currErrorSum = 0
		self.bestLocalK = self.k
		self.minLocalError = np.inf
		self.move = 0

class PSO():
	"""swarm of particles"""
	def __init__(self,maxMoves ,numParticles, maxIter ,c1 ,c2 ,omega):
		self.numParticles = numParticles
		self.c1 = c1
		self.c2 = c2
		self.omega = omega
		self.maxIter = maxIter
		self.maxMoves = maxMoves
		self.swarm = [Particle() for _ in xrange(self.numParticles)]
		self.bestGlobalK = self.swarm[0].bestLocalK
		self.minGlobalError = np.inf
		self.currParticle = 0
		self.currIter = 0
		# For Debugging
		self.errors = []
		self.lastTime = time.time()

		
		