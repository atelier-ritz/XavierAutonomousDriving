import pygame, time, math
from mathfx import sind, cosd

'''
PS3 Controllers can be turned on by presing PS button.
If you see one solid indicator light, it is on.
If you see four lights blinking it is off.
'''

class DualShock(object):
	''' direct plugin to Ubuntu '''
#    KEY = {
#        'CROSS': 0,
#        'CIRCLE': 1,
#        'TRIANGLE': 2,
#        'SQUARE': 3,
#        'L1': 4,
#        'R1': 5,
#        'L2': 6,
#        'R2': 7,
#        'SELE': 8,
#        'START': 9,
#        'PS': 10,
#        'L3': 11,
#        'R3': 12
#    }
#    AXIS = {
#        'L_X': 0,
#        'L_Y': 1,
#        'L2': 2,
#        'R_X': 3,
#        'R_Y': 4,
#        'R2': 5
#    }
    
	''' DirectInput mode using magic-NS '''
	KEY = {
		'SQUARE': 0,
		'CROSS': 1,
		'CIRCLE': 2,
		'TRIANGLE': 3,
		'L1': 4,
		'R1': 5,
		'L2': 6,
		'R2': 7,
		'SELECT': 8,
		'START': 9,
		'L3': 10,
		'R3': 11,
		'PS': 12
	}
	
	AXIS = {
		'L_X': 0,
		'L_Y': 1,
		'R_X': 2,
		'R_Y': 3,
	}

	def __init__(self):
		joystick = pygame.joystick.Joystick(0)
		joystick.init()
		self._name = joystick.get_name()
		self._numAxes = joystick.get_numaxes()
		self._numButtons = joystick.get_numbuttons()
		self.axis_data = {}
		self.button_data = {}
		self.initInputState()
		self.showInfo()

	def initInputState(self):
		for i in range(self._numAxes): 
			''' direct plugin to Ubuntu '''
#			self.axis_data[i] = 0.0
#			self.axis_data[2] = -1.0
#			self.axis_data[5] = -1.0
			''' DirectInput mode using magic-NS '''
			self.axis_data[i] = 0.0
		for i in range(self._numButtons): 
			self.button_data[i] = False
	
	def showInfo(self):
		print("===========================================")
		print('Name: {}'.format(self._name))
		print('Axes: {}'.format(self._numAxes))
		print('Buttons: {}'.format(self._numButtons))
		print("===========================================")

	def quit(self):
		pygame.joystick.quit()

	def update(self):
		for event in pygame.event.get():
			if event.type == pygame.JOYAXISMOTION:
				self.axis_data[event.axis] = round(event.value,2)
			elif event.type == pygame.JOYBUTTONDOWN:
				self.button_data[event.button] = True
			elif event.type == pygame.JOYBUTTONUP:
				self.button_data[event.button] = False

	def isPressed(self, keycode):
		return self.button_data[self.KEY[keycode]]

	def getButtonPressed(self):
		for key in self.KEY:
			if self.isPressed(key):
				return key		
	
	def getAngleLeft(self):
		rad = math.atan2(-self.axis_data[self.AXIS['L_Y']],self.axis_data[self.AXIS['L_X']])
		return math.degrees(rad)
	
	def getAngleRight(self):
		rad = math.atan2(-self.axis_data[self.AXIS['R_Y']],self.axis_data[self.AXIS['R_X']])
		return math.degrees(rad)
		
	def getMagniudeLeft(self):
		return math.sqrt(self.axis_data[self.AXIS['L_X']]**2 + self.axis_data[self.AXIS['L_Y']]**2)
		
	def getMagniudeRight(self):
		return math.sqrt(self.axis_data[self.AXIS['R_X']]**2 + self.axis_data[self.AXIS['R_Y']]**2)

	def getTiltLeft(self):
		azimuth = abs(self.getAngleLeft())
		magnitude = self.getMagniudeLeft()
		if magnitude == 0:
			return 90
		if azimuth < 45:
			magnitudeMax = 1 / cosd(azimuth)
		elif azimuth < 90:
			magnitudeMax = 1 / cosd(90-azimuth)
		elif azimuth < 135:
			magnitudeMax = 1 / cosd(azimuth-90)
		elif azimuth <= 180:
			magnitudeMax = 1 / cosd(180-azimuth)
		ratio = self.getMagniudeLeft() / magnitudeMax
		if ratio > 1: 
			ratio = 1
		return math.degrees(math.acos(ratio))

	def getTiltRight(self):
		azimuth = abs(self.getAngleRight())
		magnitude = self.getMagniudeRight()
		if magnitude == 0:
			return 90
		if azimuth < 45:
			magnitudeMax = 1 / cosd(azimuth)
		elif azimuth < 90:
			magnitudeMax = 1 / cosd(90-azimuth)
		elif azimuth < 135:
			magnitudeMax = 1 / cosd(azimuth-90)
		elif azimuth <= 180:
			magnitudeMax = 1 / cosd(180-azimuth)
		ratio = self.getMagniudeRight() / magnitudeMax
		if ratio > 1: 
			ratio = 1
		return math.degrees(math.acos(ratio))
		
if __name__ == "__main__":
	''' Run this script directly for testing '''
	import time
	import pygame
	pygame.init()
	pygame.joystick.init()
	joystick = pygame.joystick.Joystick(0)
	joystick.init()
	j = DualShock()
	while True:
		j.update()
		print(j.axis_data)
		print(j.button_data)
		print('LeftTilt:{:2.2f}, LeftMagnitude:{:2.2f}, LeftAngle:{:2.2f}'.format(j.getTiltLeft(),j.getMagniudeLeft(),j.getAngleLeft()))
		print('RightTilt:{:2.2f}, RightMagnitude:{:2.2f}, RightAngle:{:2.2f}'.format(j.getTiltRight(),j.getMagniudeRight(),j.getAngleRight()))
		time.sleep(0.1)
