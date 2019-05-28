#!/usr/bin/env python
from PyQt5 import uic
from PyQt5.QtCore import QTimer, Qt, pyqtSlot, QPoint
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt5.QtGui import QPixmap, QPainter, QImage
import smbus2 as smbus
from lib.mcp4728 import MCP4728
from lib.PS3Controller import DualShock
from lib.vision import Webcam
import time
import rospy

#=========================================================
# a class that handles the signal and callbacks of the GUI
#=========================================================

Ui_MainWindow, QtBaseClass = uic.loadUiType("mainwindow.ui")

class GUI(QMainWindow,Ui_MainWindow):
	def __init__(self):
		QMainWindow.__init__(self,None,Qt.WindowStaysOnTopHint)
		Ui_MainWindow.__init__(self)
		self.setupUi(self)
		self.setupROS()
		self.setupTimer()
		self.setupJoystick()
		self.setupDac()
		self.setupCameras()
		self.connectSignals()
		self.linkWidgets()

	def closeEvent(self,event):
		''' when terminating the app '''
		self.quitTimer()
		self.quitJoystick()
		self.quitDac()
		self.quitCameras()
		event.accept()

	def setupROS(self):
		rospy.init_node('ROS_listener')

	def setupTimer(self):
		''' run update() at 60 Hz '''
		self.timer = QTimer()
		self.timer.timeout.connect(self.update)
		self.timer.start(15) # msec
		
	def update(self):
		self.updateJoystick()
		
	def quitTimer(self):
		self.timer.stop()

	def setupJoystick(self):
		self.joystick = DualShock()
		self.isJoystickEnabled = False
	
	def updateJoystick(self):
		if self.isJoystickEnabled:
			self.lbl_joystick_button.setText(self.joystick.getButtonPressed())
			self.lbl_joystick_lstick.setText('{:2.2f}, {:2.2f}'.format(self.joystick.getAngleLeft(), self.joystick.getTiltLeft()))
			self.lbl_joystick_rstick.setText('{:2.2f}, {:2.2f}'.format(self.joystick.getAngleRight(), self.joystick.getTiltRight()))
			
	def quitJoystick(self):
		return
	
	def setupDac(self):
		PORT_NUM_I2C = 8  # Port 8 I2C bus of Xavier is used. Check status using 'sudo i2cdetect -y -r 8'
		DEFAULT_ADDRESS = 0x60
		bus = smbus.SMBus(PORT_NUM_I2C)
		self.dac = MCP4728(bus, DEFAULT_ADDRESS)
			
	def quitDac(self):
		self.dac.clearAll()

	def setupCameras(self):
		self.webcam = Webcam(1)
		self.webcam.changePixmap.connect(self.on_receive_frame)
		self.webcam2 = Webcam(2)
		self.webcam2.changePixmap.connect(self.on_receive_frame2)

	def quitCameras(self):
		return
			
	def connectSignals(self):
		''' connect widgets with callback functions '''
		self.btn_set_A.clicked.connect(self.on_btn_set_A)
		self.btn_camera.clicked.connect(lambda state: self.on_btn_camera(state))
		self.btn_camera2.clicked.connect(lambda state: self.on_btn_camera2(state))
		self.btn_joystick.clicked.connect(lambda state: self.on_btn_joystick(state))
		
	def linkWidgets(self):
		return

	#=====================================================
	# Callback Functions
	#=====================================================  
	def on_btn_set_A(self):
		self.dac.setVoltage(0, self.spb_vol_A.value())
	    
	def on_btn_camera(self, isButtonToggled):
		if isButtonToggled:
			print('<INFO> Camera is open.')
			self.webcam.start()
		else:
			print('<INFO> Camera is terminated.')
			self.webcam.stop()
			self.view_webcam.setText('Camera1')

	def on_btn_camera2(self, isButtonToggled):
		if isButtonToggled:
			print('<INFO> Camera2 is open.')
			self.webcam2.start()
		else:
			print('<INFO> Camera2 is terminated.')
			self.webcam2.stop()
			self.view_webcam2.setText('Camera2')
	
	def on_btn_joystick(self, isButtonToggled):
		if isButtonToggled:
			print('<INFO> Joystick debug starts.')
			self.isJoystickEnabled = True
		else:
			print('<INFO> Joystick debug ends.')
			self.isJoystickEnabled = False

	#=====================================================
	# PyQt signal Callback Functions
	#=====================================================  
	@pyqtSlot(QImage)
	def on_receive_frame(self, image):
		self.view_webcam.setPixmap(QPixmap.fromImage(image))

	@pyqtSlot(QImage)
	def on_receive_frame2(self, image):
		self.view_webcam2.setPixmap(QPixmap.fromImage(image))

			
		
