#!/usr/bin/python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import smbus2 as smbus

# ============================================================================
# Modeled Adafruit MCP4725 12-Bit 4-Channel DAC
# outputA - motor left speed		outputB - motor left dir
# outputC - motor right speed		outputD - motor right dir
# ============================================================================

# PCA9685 Default I2C address
MCP4728_ADDRESS = 0x60
PORT_NUM_I2C = 8  # Port 8 I2C bus of Xavier is used. Check status using 'sudo i2cdetect -y -r 8'

class MotorController(object):

	# Registers
	REG_WRITEDACEEPROM_SEQ = 0x50
	REG_WRITEDACEEPROM_A = 0x58
	REG_WRITEDACEEPROM_B = 0x5A
	REG_WRITEDACEEPROM_C = 0x5C
	REG_WRITEDACEEPROM_D = 0x5E
	
	WHEEL_DIAMETER = 0.2 # m
	DISTANCE_BETWEEN_WHEELS = 0.6 # m 

	def __init__(self, bus, mcp4728_address):
		self.bus = bus
		self.address = mcp4728_address
		self.clearAll()
		self.listener()

	# ============================================================================
	# ROS side
	# ============================================================================
	def listener(self):
		rospy.init_node('motor_controller')
		rospy.Subscriber("cmd_vel", Twist, self.callback)
		rospy.spin()
	
	def callback(self, data):
		dac_input_seq = self.joyOutput2DacInput(data.linear.x, data.angular.z) 
		self.setVoltageSeq(dac_input_seq) #[0-4095, 0/4095, 0-4095, 0/4095]
		
	def joyOutput2DacInput(self, vel_cmd, ang_vel_cmd):
		''' vel_cmd: [-1,1]. if vel_cmd = 1, the vehicle charges vel_cmd at full speed
			ang_vel_cmd: [-1,1]. if ang_vel_cmd = 1, the vehicle rotates counterclockwise (topview) at full speed '''
#		r = 0.5 * self.WHEEL_DIAMETER
#		R = 0.5 * self.DISTANCE_BETWEEN_WHEELS
#		wheel_left_ang_vel = 1 / r * (vel_cmd - ang_vel_cmd * R)
#		wheel_right_ang_vel = 1 / r * (vel_cmd + ang_vel_cmd * R)

		# just for debugging, no physical meaning
		wheel_left_ang_vel = vel_cmd - ang_vel_cmd
		wheel_right_ang_vel = vel_cmd + ang_vel_cmd
		wheel_left_spd, wheel_left_dir = self.preprocessDacInput(wheel_left_ang_vel)
		wheel_right_spd, wheel_right_dir = self.preprocessDacInput(wheel_right_ang_vel)

		return wheel_left_spd, wheel_left_dir, wheel_right_spd, wheel_right_dir
		
	def preprocessDacInput(self, val):
		''' val: [-1,1] '''
		if val < 0:
			absolute_value = - val
			direction = 0
		else:
			absolute_value = val
			direction = 4095
		absolute_value_12bit = int(4095 * absolute_value)
		return absolute_value_12bit, direction

	# ============================================================================
	# I2C side
	# ============================================================================
 	def formatDacInput(self,raw):
		''' convert a 12-bit integer to 2 bytes'''
		if raw > 4095:
			raw = 4095
		if raw < 0:
			raw = 0
		bytes = [(raw >> 8) & 0x0f, raw & 0xff]
		return bytes

	def clearAll(self):
		bytes = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
		self.bus.write_i2c_block_data(self.address, self.REG_WRITEDACEEPROM_SEQ, bytes)

	def setVoltageSeq(self, arrayVoltage):
		''' arrayVoltage: an array of 4 entries, each 0-4095 '''
		bytes = []
		for i in arrayVoltage:
			i = self.formatDacInput(i)
			bytes.extend(i)
		self.bus.write_i2c_block_data(self.address, self.REG_WRITEDACEEPROM_SEQ, bytes)

	def setVoltage(self, channel, voltage):
		''' channel: 0-3, voltage: 0-4095 '''
		bytes = self.formatDacInput(voltage)
		self.bus.write_i2c_block_data(self.address, self.REG_WRITEDACEEPROM_A + channel * 2, bytes)


if __name__ == '__main__':
	motor_controller_bus = smbus.SMBus(PORT_NUM_I2C)
	motor_controller = MotorController(motor_controller_bus, MCP4728_ADDRESS)


