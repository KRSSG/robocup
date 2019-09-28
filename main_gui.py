import os, pygame
from pygame.locals import *
from pygame.compat import geterror

if not pygame.font: print('Warning, fonts disabled')
if not pygame.mixer: print('Warning, sound disabled')

import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command

from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi
from utils.functions import *



main_dir = os.path.split(os.path.abspath(__file__))[0]
data_dir = os.path.join(main_dir, 'data')

print(os.getcwd())
pygame.init()
pygame.display.init()

pygame.font.init()

win = pygame.display.set_mode((800,500))
win.fill((255, 255, 255))



def send_command(team, bot_id, v_x,v_y, v_w, kick_power, dribble,speed,chip_power = 0):

	""" ,
	Publish the command packet
	team : 'True' if the team is yellow 
	"""
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	gr_command = gr_Robot_Command()
	final_command = gr_Commands()
	
	"""
	Set the command to each bot
	"""
	state = None
	#state=shared.get('state')
	rospy.wait_for_service('bsServer',)
	# getState = rospy.ServiceProxy('bsServer',bsServer)
	# try:
	# 	state = getState(state)
	# except rospy.ServiceException, e:
	# 	print("hello")
	# if state:
	# 	#print('lasknfcjscnajnstate',state.stateB.homePos)
	# 	state=state.stateB
	# 	kub = kubs.kubs(0,state,pub)
	# 	kub.update_state(state)

	# 	print(kub.kubs_id)	
	# 	angle=atan2(state.homePos[0].y,state.homePos[0].x)
	# 	#sprint('chal ja')
	# print fa,"-------------------- ",angle

	# v_x=0
	# #v_y=v*math.cos(fa-angle)/math.cos(fa)
	# v_y=v
	if(v_x>0):
		v_x+=speed
	elif(v_x<0):
		v_x-=speed

	if(v_y>0):
		v_y+=speed
	elif(v_y<0):
		v_y-=speed

	gr_command.id          = bot_id
	gr_command.wheelsspeed = 0
	gr_command.veltangent  = v_x
	gr_command.velnormal   = v_y
	gr_command.velangular  = v_w
	gr_command.kickspeedx  = kick_power
	gr_command.kickspeedz  = chip_power
	gr_command.spinner     = dribble

	final_command.timestamp      = rospy.get_rostime().secs
	final_command.isteamyellow   = team
	final_command.robot_commands = gr_command

	
	def debug():

		print 'botid: {}: [{}]\n'.format(bot_id, final_command.timestamp)
		print 'vel_x: {}\nvel_y: {}\nvel_w: {}\n'.format(v_x, v_y, v_w)
		print 'kick_power: {}\nchip_power: {}\ndribble_speed:{}\n\n'.format(kick_power, chip_power, dribble)
	

	pub.publish(final_command)


class button():
	def __init__(self, color, x,y,width,height, text=''):
		self.color = color
		self.x = x
		self.y = y
		self.width = width
		self.height = height
		self.text = text

	def draw(self,win,outline=None):
		#Call this method to draw the button on the screen
		if outline:
			pygame.draw.rect(win, outline, (self.x-2,self.y-2,self.width+4,self.height+4),0)
			
		pygame.draw.rect(win, self.color, (self.x,self.y,self.width,self.height),0)
		
		if self.text != '':
			font = pygame.font.SysFont('comicsans', 30)
			text = font.render(self.text, 1, (0,0,0))
			win.blit(text, (self.x + (self.width/2 - text.get_width()/2), self.y + (self.height/2 - text.get_height()/2)))

	def isOver(self, pos):
		#Pos is the mouse position or a tuple of (x,y) coordinates
		if pos[0] > self.x and pos[0] < self.x + self.width:
			if pos[1] > self.y and pos[1] < self.y + self.height:
				return True
			
		return False
def redrawWindow():
	win.fill((255,255,255))
	wButton.draw(win,(0,0,0))
	sButton.draw(win,(0,0,0))
	aButton.draw(win,(0,0,0))
	dButton.draw(win,(0,0,0))
	upButton.draw(win,(0,0,0))
	downButton.draw(win,(0,0,0))


wButton=button((255,255,0),150,200,80,80,'UP')
sButton=button((255,255,0),150,300,80,80,'DOWN')
aButton=button((255,255,0),60,300,80,80,'LEFT')
dButton=button((255,255,0),240,300,80,80,'RIGHT')
upButton=button((255,255,0),340,300,130,80,'SPEED UP')
downButton=button((255,255,0),490,300,145,80,'SPEED DOWN')
# COLOR_INACTIVE = pygame.Color('lightskyblue3')
# COLOR_ACTIVE = pygame.Color('dodgerblue2')

class InputBox:

	def __init__(self, x, y, w, h, text=''):
		self.rect = pygame.Rect(x, y, w, h)
		self.color = (120,120,120)
		self.text = text
		font = pygame.font.SysFont('comicsans', 30)
		self.txt_surface = font.render(self.text, 1, (0,0,0))
		self.active = False

	def handle_event(self, event):
		if event.type == pygame.MOUSEBUTTONDOWN:
			# If the user clicked on the input_box rect.
			if self.rect.collidepoint(event.pos):
				# Toggle the active variable.
				self.active = not self.active
			else:
				self.active = False
		if event.type == pygame.KEYDOWN:
			if self.active:
				if event.key == pygame.K_RETURN:
					print(self.text)
					self.text = ''
				elif event.key == pygame.K_BACKSPACE:
					self.text = self.text[:-1]
				else:
					self.text += event.unicode
				# Re-render the text.
				self.txt_surface = FONT.render(self.text, True, self.color)

	def update(self):
		# Resize the box if the text is too long.
		width = max(200, self.txt_surface.get_width()+10)
		self.rect.w = width

	def draw(self, screen):
		# Blit the text.
		screen.blit(self.txt_surface, (self.rect.x+5, self.rect.y+5))
		# Blit the rect.
		pygame.draw.rect(screen, self.color, self.rect, 2)


def main():
	rospy.init_node('node',anonymous=False)
	start_time = rospy.Time.now()
	start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
	
	power=False
	
	s=0
	input_box1 = InputBox(100, 100, 140, 32)
	input_box2 = InputBox(100, 300, 140, 32)
	input_boxes = [input_box1, input_box2]

	clock = pygame.time.Clock()
	
	going = True
	while going:
		clock.tick(60)
		redrawWindow()
		pygame.display.update()
		pygame.display.flip()
		for event in pygame.event.get():
			vx=0
			vy=0
			vw=0

			pos=pygame.mouse.get_pos()
			if event.type == pygame.QUIT:
				pygame.quit(); sys.exit();pygame.display.quit()
				quit()
			flag=[]
			pressed = pygame.key.get_pressed()
			if event.type == pygame.MOUSEBUTTONDOWN:
				if wButton.isOver(pos):
					flag+=["w"]
				if sButton.isOver(pos):
					flag+=["s"]
				if aButton.isOver(pos):
					flag+=["a"]
				if dButton.isOver(pos):
					flag+=["d"]
				if upButton.isOver(pos):
					flag+=["up"]
				if downButton.isOver(pos):
					flag+=["down"]

			if pressed[pygame.K_UP]: 
					s+=0.3
			if pressed[pygame.K_DOWN] :
					s=max(0,s-0.3)
			if pressed[pygame.K_LEFT]:
				vw+=1
			if pressed[pygame.K_RIGHT]:
				vw-=1
			if pressed[pygame.K_w] and pressed[pygame.K_d] or ( "w" in flag and "d" in flag):
				print("1")
				vy=0.4
				vx=0.4
			elif pressed[pygame.K_w] and pressed[pygame.K_a] or ("w" in flag and "a" in flag):
				print "2"
				vy=0.4
				vx=-0.4
			elif pressed[pygame.K_w] or flag==["w"]:
				print "3"
				vx=0
				vy=0.5
			elif pressed[pygame.K_d] or flag==["d"]:
				print "3"
				vx=0.5
				vy=0
			elif pressed[pygame.K_a] or flag==["a"]:
				print "3"
				vx=-0.5
				vy=0
			elif pressed[pygame.K_s] and pressed[pygame.K_d] or  ("s" in flag and "d" in flag):
				print "4"
				vx=0.4
				vy=-0.4
			elif pressed[pygame.K_s] and pressed[pygame.K_a] or ("s" in flag and "a" in flag):
				print "5"
				vx=-0.4
				vy=-0.4
			elif pressed[pygame.K_s] or flag==["s"]:
				print "6"
				vx=0
				vy=-0.5

			send_command(False, 0, vy,vx, vw, 0,0,s,False)

			
	pygame.display.quit()
	pygame.quit()

if __name__ == '__main__':
	main()


