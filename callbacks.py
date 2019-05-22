#!/usr/bin/env python
from PyQt5 import uic
from PyQt5.QtCore import QTimer, Qt, pyqtSlot, QPoint
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt5.QtGui import QPixmap, QPainter, QImage
import smbus2 as smbus
import pygame
from lib.mcp4728 import MCP4728
from lib.PS3Controller import DualShock
from lib.vision import Webcam
import time

#=========================================================
# a class that handles the signal and callbacks of the GUI
#=========================================================

Ui_MainWindow, QtBaseClass = uic.loadUiType("mainwindow.ui")

class GUI(QMainWindow,Ui_MainWindow):
	def __init__(self):
		QMainWindow.__init__(self,None,Qt.WindowStaysOnTopHint)
		Ui_MainWindow.__init__(self)
		self.setupUi(self)
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

	def setupTimer(self):
		''' run update() at 60 Hz '''
		self.timer = QTimer()
		self.timer.timeout.connect(self.update)
		self.timer.start(15) # msec
		
	def quitTimer(self):
		self.timer.stop()
		
	def setupJoystick(self):
		self.isJoystickEnabled = False
		pygame.init()
		pygame.joystick.init()
		if pygame.joystick.get_count() < 1:
			print('<ERROR> Joystick is not connected!')
			return
		else:
			self.joystick = DualShock()
			
	def quitJoystick(self):
		pygame.joystick.quit()
		pygame.quit()
	
	def setupDac(self):
		PORT_NUM_I2C = 8  # Port 8 I2C bus of Xavier is used. Check status using 'sudo i2cdetect -y -r 8'
		DEFAULT_ADDRESS = 0x60
		bus = smbus.SMBus(PORT_NUM_I2C)
		self.dac = MCP4728(bus, DEFAULT_ADDRESS)
			
	def quitDac(self):
		self.dac.clearAll()

	def setupCameras(self):
		self.webcam = Webcam(0)
		self.webcam.streamStarted.connect(self.on_streamStarted)
		self.webcam.changePixmap.connect(self.on_receive_frame)
		self.webcam.finished.connect(self.on_webcam_terminated)
		self.webcam2 = Webcam(1)
		self.webcam2.streamStarted.connect(self.on_streamStarted2)
		self.webcam2.changePixmap.connect(self.on_receive_frame2)
		self.webcam2.finished.connect(self.on_webcam_terminated2)
	
	def quitCameras(self):
		self.webcam.stop()
		self.webcam2.stop()
		
	def update(self):
		self.updateJoystick()
	
	def updateJoystick(self):
		if self.isJoystickEnabled:
			try:
				self.joystick
			except:
				pass
			else:
				self.joystick.update()
				self.lbl_joystick_button.setText(self.joystick.getButtonPressed())
				self.lbl_joystick_lstick.setText('{:2.2f}, {:2.2f}'.format(self.joystick.getAngleLeft(), self.joystick.getTiltLeft()))
				self.lbl_joystick_rstick.setText('{:2.2f}, {:2.2f}'.format(self.joystick.getAngleRight(), self.joystick.getTiltRight()))
			
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
			self.btn_camera.setEnabled(False)
			self.webcam.start()
		else:
			self.webcam.stop()
	
	def on_btn_camera2(self, isButtonToggled):
		if isButtonToggled:
			print('<INFO> Camera2 is open.')
			self.btn_camera2.setEnabled(False)
			self.webcam2.start()
		else:
			self.webcam2.stop()
	
	def on_btn_joystick(self, isButtonToggled):
		if isButtonToggled:
			print('<INFO> Joystick is connected.')
			self.isJoystickEnabled = True
		else:
			print('<INFO> Joystick is disconnected.')
			self.isJoystickEnabled = False
			
	#=====================================================
    # PyQtslot Callback Functions
    #===================================================== 
	@pyqtSlot()
	def on_streamStarted(self):
		self.btn_camera.setEnabled(True)
		
	@pyqtSlot()
	def on_streamStarted2(self):
		self.btn_camera2.setEnabled(True)

	@pyqtSlot(QImage)
	def on_receive_frame(self, image):
		self.view_webcam.setPixmap(QPixmap.fromImage(image))
		
	@pyqtSlot(QImage)
	def on_receive_frame2(self, image):
		self.view_webcam2.setPixmap(QPixmap.fromImage(image))

	@pyqtSlot()
	def on_webcam_terminated(self):
		print('<INFO> Webcam is terminated.')
		self.view_webcam.setText('Camera1')

	@pyqtSlot()
	def on_webcam_terminated2(self):
		print('<INFO> Webcam2 is terminated.')
		self.view_webcam2.setText('Camera2')

		
