import math
from mathfx import sind, cosd

import rospy
from sensor_msgs.msg import Joy

#receiving and processing messages from ROS package Joy

#Package: 
#Joy

#Nodes: 
#joy_node

#Published:
#joy (sensor_msgs/Joy)
#message type: sensor_msgs/Joy 

class DualShock(object):
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
		'CURSOR_X': 4,
		'CURSOR_Y': 5
	}

	def __init__(self):
		self.axis_data = [0.0,0.0,0.0,0.0,0.0,0.0]
		self.button_data = [0,0,0,0,0,0,0,0,0,0,0,0,0]
		rospy.init_node('joystick_listener')
		rospy.Subscriber("joy", Joy, self.callback)

	def callback(self,data):
		self.button_data = list(data.buttons)
		self.axis_data = list(data.axes)
	
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



