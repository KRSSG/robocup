##
## @file pid.py
## @brief Return velocity after applying PID
##

import numpy as np
import matplotlib.pyplot as plt
import time
dt = 0.001


##
## @brief      PID on velocity vx,vy
##
## @param      vX         velocity in x
## @param      vY         velocity in y
## @param      errorInfo  The error information
## @param      pso        Particle Swarm Optimiser object
## 
## @see        error.py
##
##
## @return     Velocity after PID
##
def pid(vX,vY,errorInfo,pso=None):
	errorPX = errorInfo.errorX
	errorPY = errorInfo.errorY
	errorIX = errorInfo.errorIX + errorPX
	errorIY = errorInfo.errorIY + errorPY
	errorDX = (errorPX - errorInfo.lastErrorX)/dt
	errorDY = (errorPY - errorInfo.lastErrorY)/dt
	errorX = np.array([errorPX,errorIX,errorDX])
	errorY = np.array([errorPY,errorIY,errorDY])
	if pso==None:
		k = np.array([3,0.00001,0.00001]) 		#define k
		deltaVX = errorX.dot(k)
		deltaVY = errorY.dot(k)

		vX = vX + deltaVX
		vY = vY + deltaVY
		return vX,vY

	# Optimiser (PSO)
	else:
		currIter = pso.currIter
		p = pso.currParticle
		currMove = pso.swarm[p].move
		maxMoves = pso.maxMoves
		numParticles = pso.numParticles
		maxIter = pso.maxIter

		k = pso.swarm[p].k
		# Disabling I & D in PID
		# k[1] = k[2] = 0
		# k[0] = 0
		

		deltaVX = errorX.dot(k)
		deltaVY = errorY.dot(k)

		vX = vX + deltaVX
		vY = vY + deltaVY

		errorMagnitude = np.linalg.norm(np.array([errorPX,errorPY]))
		pso.swarm[p].currErrorSum += errorMagnitude


		errorInfo.errorIX = errorInfo.errorIX + errorInfo.errorX
		errorInfo.errorIY = errorInfo.errorIY + errorInfo.errorY
		errorInfo.lastErrorX = errorInfo.errorX
		errorInfo.lastErrorY = errorInfo.errorY

		pso.swarm[p].move = (currMove + 1)%maxMoves
		pso.lastTime = time.time()
		pso.errors.append(errorMagnitude)

		if pso.swarm[p].move == 0:
			if pso.swarm[p].currErrorSum <= pso.swarm[p].minLocalError:
				pso.swarm[p].minLocalError = pso.swarm[p].currErrorSum
				pso.swarm[p].bestLocalK = k

			if pso.swarm[p].currErrorSum <= pso.minGlobalError:
				pso.minGlobalError = pso.swarm[p].currErrorSum
				pso.bestGlobalK = k

			pso.swarm[p].currErrorSum = 0 ##For next iteration
			r1 = np.random.rand()
			r2 = np.random.rand()
			pso.swarm[p].v = pso.omega*pso.swarm[p].v + pso.c1*r1*(pso.swarm[p].bestLocalK - k) + pso.c2*r2*(pso.bestGlobalK - k)
			pso.swarm[p].k = k + pso.swarm[p].v
			pso.currParticle = (p+1)%numParticles
			if pso.currParticle == 0:
				pso.currIter = currIter + 1

		# if pso.currIter == 20:
		# 	plt.plot(pso.errors)
		# 	plt.show()
		print("_______________________k = ",k)
		return vX,vY