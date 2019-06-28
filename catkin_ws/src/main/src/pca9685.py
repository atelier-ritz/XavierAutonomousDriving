#!/usr/bin/python
import rospy, time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import smbus2 as smbus

# ============================================================================
# PCA9685 12-Bit 16-Channel pwm
# ch0 - motor left speed		ch1 - motor left dir	ch2 - motor left brake
# ch3 - motor right speed		ch4 - motor right dir	ch5 - motor right brake
# ============================================================================

# PCA9685 Default I2C address
PCA9685_ADDRESS = 0x40
PORT_NUM_I2C = 8  # Port 8 I2C bus of Xavier is used. Check status using 'sudo i2cdetect -y -r 8'

class MotorController(object):

	# Registers/etc:
	PCA9685_ADDRESS    = 0x40
	MODE1              = 0x00
	MODE2              = 0x01
	SUBADR1            = 0x02
	SUBADR2            = 0x03
	SUBADR3            = 0x04
	PRESCALE           = 0xFE
	LED0_ON_L          = 0x06
	LED0_ON_H          = 0x07
	LED0_OFF_L         = 0x08
	LED0_OFF_H         = 0x09
	ALL_LED_ON_L       = 0xFA
	ALL_LED_ON_H       = 0xFB
	ALL_LED_OFF_L      = 0xFC
	ALL_LED_OFF_H      = 0xFD

	# Bits:
	RESTART            = 0x80
	SLEEP              = 0x10
	ALLCALL            = 0x01
	INVRT              = 0x10
	OUTDRV             = 0x04

	def __init__(self, bus, address):
		self.bus = bus
		self.address = address
		self.set_all_pwm(0, 0)
		self.bus.write_byte_data(self.address, self.MODE2, self.OUTDRV)
		self.bus.write_byte_data(self.address, self.MODE1, self.ALLCALL)
		time.sleep(0.005)  # wait for oscillator
		mode1 = self.bus.read_byte_data(self.address, self.MODE1)
		mode1 = mode1 & ~self.SLEEP  # wake up (reset sleep)
		self.bus.write_byte_data(self.address, self.MODE1, mode1)
		time.sleep(0.005)  # wait for oscillator
		self.listener()

	# ============================================================================
	# ROS side
	# ============================================================================
	def listener(self):
		rospy.init_node('motor_controller_pca_9685')
		rospy.Subscriber("cmd_vel", Twist, self.callback)
		rospy.spin()
	
	def callback(self, data):
		rawPwmArray = self.joy2pwm(data.linear.x, data.angular.z) 
		self.setPwm(rawPwmArray)
	
	def setPwm(self, pwmArray):
		'''0: spd_left, 1: dir_left, 2: brake_left, 3: spd_right, 4:dir_right, 5: brake_right'''
		for i in range(6):
			self.set_voltage(i,pwmArray[i])
		
	def joy2pwm(self, vel_cmd, ang_vel_cmd):
		''' vel_cmd: [-1,1]. if vel_cmd = 1, the vehicle charges vel_cmd at full speed
			ang_vel_cmd: [-1,1]. if ang_vel_cmd = 1, the vehicle rotates counterclockwise (topview) at full speed '''
#		r = 0.5 * self.WHEEL_DIAMETER
#		R = 0.5 * self.DISTANCE_BETWEEN_WHEELS
#		wheel_left_ang_vel = 1 / r * (vel_cmd - ang_vel_cmd * R)
#		wheel_right_ang_vel = 1 / r * (vel_cmd + ang_vel_cmd * R)

		# just for debugging, no physical meaning
		scale = 1 # the smaller the value, the slower the vehicle
		weight = 0.7 # the bigger the value, the slower turning
		wheel_left_ang_vel = scale * (weight * vel_cmd - (1-weight) * ang_vel_cmd) # range: [-1, 1]
		wheel_right_ang_vel = scale * (weight * vel_cmd + (1-weight) * ang_vel_cmd) # range: [-1, 1]
		wheel_left_spd, wheel_left_dir = self.preprocessDacInput(wheel_left_ang_vel)
		wheel_right_spd, wheel_right_dir = self.preprocessDacInput(wheel_right_ang_vel)
		return wheel_left_spd, wheel_left_dir, 0, wheel_right_spd, wheel_right_dir, 0

	def preprocessDacInput(self, val):
		''' val: [-1,1] '''
		if val < 0:
			absolute_value = -5 * val
			direction = 0
		else:
			absolute_value = 5 * val
			direction = 5
		return absolute_value, direction

	# ============================================================================
	# I2C side
	# ============================================================================
	def set_pwm_freq(self, freq_hz):
		"""Set the PWM frequency to the provided value in hertz."""
		prescaleval = 25000000.0    # 25MHz
		prescaleval /= 4096.0       # 12-bit
		prescaleval /= float(freq_hz)
		prescaleval -= 1.0
		logger.debug('Setting PWM frequency to {0} Hz'.format(freq_hz))
		logger.debug('Estimated pre-scale: {0}'.format(prescaleval))
		prescale = int(math.floor(prescaleval + 0.5))
		logger.debug('Final pre-scale: {0}'.format(prescale))
		oldmode = self._device.readU8(MODE1);
		newmode = (oldmode & 0x7F) | 0x10    # sleep
		self.bus.write_byte_data(self.address, self.MODE1, newmode)  # go to sleep
		self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
		self.bus.write_byte_data(self.address, self.MODE1, oldmode)
		time.sleep(0.005)
		self.bus.write_byte_data(self.address, self.MODE1, oldmode | 0x80)

	def set_pwm(self, channel, on, off):
		"""Sets a single PWM channel."""
		self.bus.write_byte_data(self.address, self.LED0_ON_L+4*channel, on & 0xFF)
		self.bus.write_byte_data(self.address, self.LED0_ON_H+4*channel, on >> 8)
		self.bus.write_byte_data(self.address, self.LED0_OFF_L+4*channel, off & 0xFF)
		self.bus.write_byte_data(self.address, self.LED0_OFF_H+4*channel, off >> 8)

	def set_all_pwm(self, on, off):
		"""Sets all PWM channels."""
		self.bus.write_byte_data(self.address, self.ALL_LED_ON_L, on & 0xFF)
		self.bus.write_byte_data(self.address, self.ALL_LED_ON_H, on >> 8)
		self.bus.write_byte_data(self.address, self.ALL_LED_OFF_L, off & 0xFF)
		self.bus.write_byte_data(self.address, self.ALL_LED_OFF_H, off >> 8)

	def set_voltage(self, channel, voltage):
		"""Sets a single PWM channel to a voltage. voltage:0-5V"""
		if voltage > 5: 
			voltage = 5
		if voltage < 0: 
			voltage = 0
		raw = int(4095/5*voltage)
		self.bus.write_byte_data(self.address, self.LED0_ON_L+4*channel, 0)
		self.bus.write_byte_data(self.address, self.LED0_ON_H+4*channel, 0)
		self.bus.write_byte_data(self.address, self.LED0_OFF_L+4*channel, raw & 0xFF)
		self.bus.write_byte_data(self.address, self.LED0_OFF_H+4*channel, raw >> 8)

	def set_all_voltage(self, voltage):
		"""Sets all PWM channels to the same voltage. voltage: 0-5V"""
		if voltage > 5: 
			voltage = 5
		if voltage < 0: 
			voltage = 0
		raw = int(4095/5*voltage)
		self.bus.write_byte_data(self.address, self.ALL_LED_ON_L, 0)
		self.bus.write_byte_data(self.address, self.ALL_LED_ON_H, 0)
		self.bus.write_byte_data(self.address, self.ALL_LED_OFF_L, raw & 0xFF)
		self.bus.write_byte_data(self.address, self.ALL_LED_OFF_H, raw >> 8)

if __name__ == '__main__':
	motor_controller_bus = smbus.SMBus(PORT_NUM_I2C)
	motor_controller = MotorController(motor_controller_bus, PCA9685_ADDRESS)

