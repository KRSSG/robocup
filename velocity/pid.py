##
## @file pid.py
## @brief Return velocity after applying PID
##
import rospy
import numpy as np
import matplotlib.pyplot as plt
import time
from utils.config import *
from utils.geometry import *

# try:
# 	f = open("pid_img/values/pid.txt",'a')
# except:
# 	f = open("pid_img/values/pid.txt",'w')




# pid_tu = rospy.Publisher('pid_c', pid_tune, queue_size=10)

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


class PID():
	def __init__(self, _errorInfo, _PSO = None):
		self.errorInfo = _errorInfo
		self.PSO = _PSO
		self.dt = 0.001
		self.i = 0
		self.k = np.array([5, 0, 0])
		self.errors_X = []
		self.errors_Y = []

	def pid(self, vX, vY):
		errorPX = self.errorInfo.errorX
		errorPY = self.errorInfo.errorY
		self.errors_X.append(errorPX)
		self.errors_Y.append(errorPY)
		errorIX = self.errorInfo.errorIX + errorPX
		errorIY = self.errorInfo.errorIY + errorPY
		errorDX = (errorPX - self.errorInfo.lastErrorX)/self.dt
		errorDY = (errorPY - self.errorInfo.lastErrorY)/self.dt
		errorX = np.array([errorPX,errorIX,errorDX])
		errorY = np.array([errorPY,errorIY,errorDY])
		if self.PSO==None:
			# k = np.array([0,0,0]) 		#define k
			# k = np.array([3.5,0.00001,0.0003])
			# print("PID applied")
			deltaVX = errorX.dot(self.k)
			deltaVY = errorY.dot(self.k)
			self.errorInfo.errorIX = self.errorInfo.errorIX + self.errorInfo.errorX
			self.errorInfo.errorIY = self.errorInfo.errorIY + self.errorInfo.errorY
			self.errorInfo.lastErrorX = self.errorInfo.errorX
			self.errorInfo.lastErrorY = self.errorInfo.errorY

			print(deltaVX, deltaVY, vX, vY)
			vX = vX + deltaVX
			vY = vY + deltaVY

			velocity = Vector2D(vX,vY)
			velocity_magnitude = velocity.abs(velocity)
			if velocity_magnitude > MAX_BOT_SPEED:
				velocity_angle = math.atan2(vY,vX)
				vX = MAX_BOT_SPEED*math.cos(velocity_angle)
				vY = MAX_BOT_SPEED*math.sin(velocity_angle)
				# print("________________Velocity Clipped________________")

			return vX,vY

		# Optimiser (PSO)
		else:
			####### TODO: Modify according to class structure.
			currIter = self.pso.currIter
			p = self.pso.currParticle
			currMove = self.pso.swarm[p].move
			maxMoves = self.pso.maxMoves
			numParticles = self.pso.numParticles
			maxIter = self.pso.maxIter

			k = self.pso.swarm[p].k
			# pid_tune = pid_tune()
			# pid_tune.pc = k[0]
			# pid_tune.ic = k[1]
			# pid_tune.dc = k[2]
			# pid_tu.publish(pid_tune)
			# print("pid constants ", k)

			deltaVX = errorX.dot(k)
			deltaVY = errorY.dot(k)

			vX = vX + deltaVX
			vY = vY + deltaVY

			errorMagnitude = np.linalg.norm(np.array([errorPX,errorPY]))
			self.pso.swarm[p].currErrorSum += errorMagnitude


			self.errorInfo.errorIX = self.errorInfo.errorIX + self.errorInfo.errorX
			self.errorInfo.errorIY = self.errorInfo.errorIY + self.errorInfo.errorY
			self.errorInfo.lastErrorX = self.errorInfo.errorX
			self.errorInfo.lastErrorY = self.errorInfo.errorY

			self.pso.swarm[p].move = (currMove + 1)%maxMoves
			self.pso.lastTime = time.time()
			self.pso.errors.append(errorPX)
			# print("Current Move", currMove)
			if self.pso.swarm[p].move == 0:
				if self.pso.swarm[p].currErrorSum <= self.pso.swarm[p].minLocalError:
					self.pso.swarm[p].minLocalError = self.pso.swarm[p].currErrorSum
					self.pso.swarm[p].bestLocalK = k

				if self.pso.swarm[p].currErrorSum <= self.pso.minGlobalError:
					self.pso.minGlobalError = self.pso.swarm[p].currErrorSum
					self.pso.bestGlobalK = k

				self.pso.swarm[p].currErrorSum = 0 ##For next iteration
				r1 = np.random.rand()
				r2 = np.random.rand()
				self.pso.swarm[p].v = self.pso.omega*self.pso.swarm[p].v + self.pso.c1*r1*(self.pso.swarm[p].bestLocalK - k) + self.pso.c2*r2*(self.pso.bestGlobalK - k)
				self.pso.swarm[p].k = k + self.pso.swarm[p].v
				self.pso.currParticle = (p+1)%numParticles
				# print("Particle #", self.pso.currParticle)
				if self.pso.currParticle == 0:
					self.pso.currIter = currIter + 1
					# print("Current Iteration #", self.pso.currIter)

			if self.pso.currIter%20 == 0:
				plt.plot(self.pso.errors)
				plt.savefig('pid_img/img/myfig_'+str(i)+'.png')
				# print("____________________________File Saved______________________________________",i)
				# if i%100 == 0:
				k_values = ','.join(map(str,self.pso.bestGlobalK))
				# f.write(k_values)	

				# np.save("pid_img/values/pid_"+str(i)+".txt",self.pso.bestGlobalK)
				self.i += 1



			return vX,vY

def pid(vX,vY,errorInfo,pso=None):
	global i,f
	errorPX = errorInfo.errorX
	errorPY = errorInfo.errorY
	errorIX = errorInfo.errorIX + errorPX
	errorIY = errorInfo.errorIY + errorPY
	errorDX = (errorPX - errorInfo.lastErrorX)/dt
	errorDY = (errorPY - errorInfo.lastErrorY)/dt
	errorX = np.array([errorPX,errorIX,errorDX])
	errorY = np.array([errorPY,errorIY,errorDY])
	if pso==None:
		# k = np.array([0,0,0]) 		#define k
		k = np.array([3.5,0.00001,0.0003])
		# print("PID applied")
		deltaVX = errorX.dot(k)
		deltaVY = errorY.dot(k)
		errorInfo.errorIX = errorInfo.errorIX + errorInfo.errorX
		errorInfo.errorIY = errorInfo.errorIY + errorInfo.errorY
		errorInfo.lastErrorX = errorInfo.errorX
		errorInfo.lastErrorY = errorInfo.errorY

		vX = vX + deltaVX
		vY = vY + deltaVY

		velocity = Vector2D(vX,vY)
		velocity_magnitude = velocity.abs(velocity)
		if velocity_magnitude > MAX_BOT_SPEED:
			velocity_angle = math.atan2(vY,vX)
			vX = MAX_BOT_SPEED*math.cos(velocity_angle)
			vY = MAX_BOT_SPEED*math.sin(velocity_angle)
			# print("________________Velocity Clipped________________")
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
		# pid_tune = pid_tune()
		# pid_tune.pc = k[0]
		# pid_tune.ic = k[1]
		# pid_tune.dc = k[2]
		# pid_tu.publish(pid_tune)
		# print("pid constants ", k)

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
		pso.errors.append(errorPX)
		# print("Current Move", currMove)
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
			# print("Particle #", pso.currParticle)
			if pso.currParticle == 0:
				pso.currIter = currIter + 1
				# print("Current Iteration #", pso.currIter)

		if pso.currIter%20 == 0:
			plt.plot(pso.errors)
			plt.savefig('pid_img/img/myfig_'+str(i)+'.png')
			# print("____________________________File Saved______________________________________",i)
			# if i%100 == 0:
			k_values = ','.join(map(str,pso.bestGlobalK))
			# f.write(k_values)	

			# np.save("pid_img/values/pid_"+str(i)+".txt",pso.bestGlobalK)
			i += 1



		return vX,vY

