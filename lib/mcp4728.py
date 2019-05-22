#!/usr/bin/python

# ============================================================================
# Modeled Adafruit MCP4725 12-Bit 4-Channel DAC
# ============================================================================

# PCA9685 Default I2C address
# MCP4728_ADDRESS = 0x60

def formatRaw(raw):
	''' convert a 12-bit integer to 2 bytes'''
	if (raw > 4095):
		raw = 4095
	if (raw < 0):
		raw = 0
	bytes = [(raw >> 8) & 0x0f, raw & 0xff]
	return bytes
		
class MCP4728:
	# Registers
	REG_WRITEDACEEPROM_SEQ = 0x50
	REG_WRITEDACEEPROM_A = 0x58
	REG_WRITEDACEEPROM_B = 0x5A
	REG_WRITEDACEEPROM_C = 0x5C
	REG_WRITEDACEEPROM_D = 0x5E

	def __init__(self, bus, address, debug=True):
		self.bus = bus
		self.address = address
		self.isDebug = debug
		self.clearAll()
	
	def formatRaw(self, raw):
		''' convert a 12-bit integer to 2 bytes'''
		if (raw > 4095):
			raw = 4095
		if (raw < 0):
			raw = 0
		bytes = [(i >> 8) & 0x0f, i & 0xff]
		return bytes
		
	def clearAll(self):
		bytes = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
		self.bus.write_i2c_block_data(self.address, self.REG_WRITEDACEEPROM_SEQ, bytes)

	def setVoltageSeq(self, arrayVoltage):
		''' arrayVoltage: an array of 4 entries, each 0-4095 '''
		bytes = []
		for i in arrayVoltage:
			i = formatRaw(i)
			bytes.extend(i)
		self.bus.write_i2c_block_data(self.address, self.REG_WRITEDACEEPROM_SEQ, bytes)

	def setVoltage(self, channel, voltage):
		''' channel: 0-3, voltage: 0-4095 '''
		bytes = formatRaw(voltage)
		self.bus.write_i2c_block_data(self.address, self.REG_WRITEDACEEPROM_A + channel * 2, bytes)
