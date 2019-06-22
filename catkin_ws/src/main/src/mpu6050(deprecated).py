#!/usr/bin/python
import rospy, tf, math
from tf.transformations import quaternion_about_axis
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import smbus2 as smbus
import numpy as np
# ============================================================================
# MPU6050 IMU
# ============================================================================

# MPU_6050 Default I2C address
MPU6050_ADDRESS = 0x68
PORT_NUM_I2C = 8  # Port 8 I2C bus of Xavier is used. Check status using 'sudo i2cdetect -y -r 8'
GRAVITIY_MS2 = 9.80665

class MPU6050(object):

	# Scale Modifiers
	ACCEL_SCALE_MODIFIER_2G = 16384.0
	ACCEL_SCALE_MODIFIER_4G = 8192.0
	ACCEL_SCALE_MODIFIER_8G = 4096.0
	ACCEL_SCALE_MODIFIER_16G = 2048.0

	GYRO_SCALE_MODIFIER_250DEG = 131.0
	GYRO_SCALE_MODIFIER_500DEG = 65.5
	GYRO_SCALE_MODIFIER_1000DEG = 32.8
	GYRO_SCALE_MODIFIER_2000DEG = 16.4

	# Pre-defined ranges
	ACCEL_RANGE_2G = 0x00
	ACCEL_RANGE_4G = 0x08
	ACCEL_RANGE_8G = 0x10
	ACCEL_RANGE_16G = 0x18

	GYRO_RANGE_250DEG = 0x00
	GYRO_RANGE_500DEG = 0x08
	GYRO_RANGE_1000DEG = 0x10
	GYRO_RANGE_2000DEG = 0x18

	# MPU-6050 Registers
	PWR_MGMT_1 = 0x6B
	PWR_MGMT_2 = 0x6C

	ACCEL_XOUT0 = 0x3B
	ACCEL_YOUT0 = 0x3D
	ACCEL_ZOUT0 = 0x3F

	TEMP_OUT0 = 0x41

	GYRO_XOUT0 = 0x43
	GYRO_YOUT0 = 0x45
	GYRO_ZOUT0 = 0x47

	ACCEL_CONFIG = 0x1C
	GYRO_CONFIG = 0x1B

	def __init__(self, bus, address):
		self.bus = bus
		self.address = address
		# Wake up the MPU-6050 since it starts in sleep mode
		self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
		try:
			self.talker()
		except rospy.ROSInterruptException: 
			pass

#	# ============================================================================
#	# ROS side
#	# ============================================================================
	def talker(self):
		rospy.init_node('imu_node')
		param_linkname = rospy.get_param("~frame_id", "base_link")
		param_frequency = rospy.get_param("~frequency", 10)

		pub = rospy.Publisher("imu", Imu, queue_size=10)
		br = tf.TransformBroadcaster()
		rate = rospy.Rate(param_frequency)
		msg = Imu()
		while not rospy.is_shutdown():
			msg.header.stamp = rospy.get_rostime()
			msg.header.frame_id = param_linkname
			
			accel = self.get_accel_data()
			gyro = self.get_gyro_data()

			msg.linear_acceleration.x = accel[0]
			msg.linear_acceleration.y = accel[1]
			msg.linear_acceleration.z = accel[2]
			msg.linear_acceleration_covariance[0] = -1

			msg.angular_velocity.x = gyro[0]
			msg.angular_velocity.y = gyro[1]
			msg.angular_velocity.z = gyro[2]
			msg.angular_velocity_covariance[0] = -1

			ref = np.array([0, 0, 1])
			accel_normalized = accel / np.linalg.norm(accel)
			axis = np.cross(accel_normalized, ref)
			angle = np.arccos(np.dot(accel_normalized, ref))
			quaternion = quaternion_about_axis(angle, axis)

			msg.orientation.x = quaternion[0]
			msg.orientation.y = quaternion[1]
			msg.orientation.z = quaternion[2]
			msg.orientation.w = quaternion[3]
			msg.orientation_covariance[0] = -1
			

			pub.publish(msg)

			def calcEuler(x,y,z):
				theta = math.atan( x / math.sqrt(y*y + z*z) )
				psi = math.atan( y / math.sqrt(x*x + z*z) )
				phi = math.atan( math.sqrt( x*x + y*y ) / z)
				return [theta, psi, phi]

			
			roll, pitch, yaw = calcEuler(accel_data['x'],accel_data['y'],accel_data['z'])
			
			br.sendTransform((0,0,0), tf.transformations.quaternion_from_euler(roll,pitch,yaw),rospy.Time.now(), "mpu6050", "map")

			rate.sleep()

	# ============================================================================
	# I2C side
	# ============================================================================
	def read_i2c_word(self, register):
		"""Read two i2c registers and combine them.
		register -- the first register to read from.
		Returns the combined read results.
		"""
		# Read the data from the registers
		high = self.bus.read_byte_data(self.address, register)
		low = self.bus.read_byte_data(self.address, register + 1)

		value = (high << 8) + low

		if (value >= 0x8000):
			return -((65535 - value) + 1)
		else:
			return value

	# MPU-6050 Methods

	def get_temp(self):
		"""Reads the temperature from the onboard temperature sensor of the MPU-6050.
		Returns the temperature in degrees Celcius.
		"""
		raw_temp = self.read_i2c_word(self.TEMP_OUT0)

		# Get the actual temperature using the formule given in the
		# MPU-6050 Register Map and Descriptions revision 4.2, page 30
		actual_temp = (raw_temp / 340.0) + 36.53

		return actual_temp

	def set_accel_range(self, accel_range):
		"""Sets the range of the accelerometer to range.
		accel_range -- the range to set the accelerometer to. Using a
		pre-defined range is advised.
		"""
		# First change it to 0x00 to make sure we write the correct value later
		self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

		# Write the new range to the ACCEL_CONFIG register
		self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

	def read_accel_range(self, raw = False):
		"""Reads the range the accelerometer is set to.
		If raw is True, it will return the raw value from the ACCEL_CONFIG
		register
		If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
		returns -1 something went wrong.
		"""
		raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

		if raw is True:
			return raw_data
		elif raw is False:
			if raw_data == self.ACCEL_RANGE_2G:
				return 2
			elif raw_data == self.ACCEL_RANGE_4G:
				return 4
			elif raw_data == self.ACCEL_RANGE_8G:
				return 8
			elif raw_data == self.ACCEL_RANGE_16G:
				return 16
			else:
				return -1

	def get_accel_data(self, g = False):
		"""Gets and returns the X, Y and Z values from the accelerometer.
		If g is True, it will return the data in g
		If g is False, it will return the data in m/s^2
		Returns a dictionary with the measurement results.
		"""
		x = self.read_i2c_word(self.ACCEL_XOUT0)
		y = self.read_i2c_word(self.ACCEL_YOUT0)
		z = self.read_i2c_word(self.ACCEL_ZOUT0)

		accel_scale_modifier = None
		accel_range = self.read_accel_range(True)

		if accel_range == self.ACCEL_RANGE_2G:
			accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
		elif accel_range == self.ACCEL_RANGE_4G:
			accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
		elif accel_range == self.ACCEL_RANGE_8G:
			accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
		elif accel_range == self.ACCEL_RANGE_16G:
			accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
		else:
			print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
			accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

		x = x / accel_scale_modifier
		y = y / accel_scale_modifier
		z = z / accel_scale_modifier

		if g is True:
			return x, y, z
		elif g is False:
			x = x * GRAVITIY_MS2
			y = y * GRAVITIY_MS2
			z = z * GRAVITIY_MS2
			return x, y, z

	def set_gyro_range(self, gyro_range):
		"""Sets the range of the gyroscope to range.
		gyro_range -- the range to set the gyroscope to. Using a pre-defined
		range is advised.
		"""
		# First change it to 0x00 to make sure we write the correct value later
		self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

		# Write the new range to the ACCEL_CONFIG register
		self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

	def read_gyro_range(self, raw = False):
		"""Reads the range the gyroscope is set to.
		If raw is True, it will return the raw value from the GYRO_CONFIG
		register.
		If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
		returned value is equal to -1 something went wrong.
		"""
		raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

		if raw is True:
			return raw_data
		elif raw is False:
			if raw_data == self.GYRO_RANGE_250DEG:
				return 250
			elif raw_data == self.GYRO_RANGE_500DEG:
				return 500
			elif raw_data == self.GYRO_RANGE_1000DEG:
				return 1000
			elif raw_data == self.GYRO_RANGE_2000DEG:
				return 2000
			else:
				return -1

	def get_gyro_data(self):
		"""Gets and returns the X, Y and Z values from the gyroscope.
		Returns the read values in a dictionary.
		"""
		x = self.read_i2c_word(self.GYRO_XOUT0)
		y = self.read_i2c_word(self.GYRO_YOUT0)
		z = self.read_i2c_word(self.GYRO_ZOUT0)

		gyro_scale_modifier = None
		gyro_range = self.read_gyro_range(True)

		if gyro_range == self.GYRO_RANGE_250DEG:
			gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
		elif gyro_range == self.GYRO_RANGE_500DEG:
			gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
		elif gyro_range == self.GYRO_RANGE_1000DEG:
			gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
		elif gyro_range == self.GYRO_RANGE_2000DEG:
			gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
		else:
			print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
			gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

		x = x / gyro_scale_modifier
		y = y / gyro_scale_modifier
		z = z / gyro_scale_modifier

		return x, y, z

	def get_all_data(self):
		"""Reads and returns all the available data."""
		temp = self.get_temp()
		accel = self.get_accel_data()
		gyro = self.get_gyro_data()

		return [accel, gyro, temp]


if __name__ == '__main__':
	imu_bus = smbus.SMBus(PORT_NUM_I2C)
	imu = MPU6050(imu_bus, MPU6050_ADDRESS)
#	while True:

#		print('temp',imu.get_temp())
#		accel_data = imu.get_accel_data()
#		print('accX',accel_data['x'])
#		print('accY',accel_data['y'])
#		print('accZ',accel_data['z'])
#		gyro_data = imu.get_gyro_data()
#		print('gyroX',gyro_data['x'])
#		print('gyroY',gyro_data['y'])
#		print('gyroZ',gyro_data['z'])


