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
		self.setupCamera()
		self.connectSignals()
		self.linkWidgets()

	def closeEvent(self,event):
		''' when terminating the app '''
		self.quitTimer()
		self.quitJoystick()
		self.quitDac()
		self.quitCamera()
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
#		time.sleep(0.1) # need to wait before stop sending signals
		self.dac.clearAll()

	def setupCamera(self):
		self.webcam = Webcam(0)
		self.webcam.changePixmap.connect(self.on_receive_frame)
		self.webcam.finished.connect(self.on_webcam_terminated)
	
	def quitCamera(self):
		self.webcam.stop()
		
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
				# below is option	
#				if self.joystick.isPressed('CIRCLE'):		
#					print '<INFO> Circle is pressed!\r',
#				if self.joystick.isPressed('SQUARE'):		
#					print '<INFO> Square is pressed!\r',	
				self.lbl_joystick_button.setText(self.joystick.getButtonPressed())
				self.lbl_joystick_lstick.setText('{:2.2f}, {:2.2f}'.format(self.joystick.getAngleLeft(), self.joystick.getTiltLeft()))
				self.lbl_joystick_rstick.setText('{:2.2f}, {:2.2f}'.format(self.joystick.getAngleRight(), self.joystick.getTiltRight()))
#				val = int(self.joystick.getMagniudeLeft() * 2890)
#				self.dac.setVoltage(0, val)
			
	def connectSignals(self):
		''' connect widgets with callback functions '''
		self.btn_set_A.clicked.connect(self.on_btn_set_A)
		self.btn_camera.clicked.connect(lambda state: self.on_btn_camera(state))
		self.btn_joystick.clicked.connect(lambda state: self.on_btn_joystick(state))
		
	def linkWidgets(self):
		return

	#=====================================================
    # Callback Functions
    #=====================================================  
	def on_btn_set_A(self):
		self.dac.setVoltage(0, self.spb_vol_A.value())
		#mydac.setVoltageSeq()
	    #print self.spb_vol_A.value()
	    
	def on_btn_camera(self, isButtonToggled):
		if isButtonToggled:
			print('<INFO> Camera is open.')
			self.webcam.init()
			self.webcam.start()
		else:
			self.webcam.stop()
	
	def on_btn_joystick(self, isButtonToggled):
		if isButtonToggled:
			print('<INFO> Joystick is connected.')
			self.isJoystickEnabled = True
		else:
			print('<INFO> Joystick is disconnected.')
			self.isJoystickEnabled = False
	
	@pyqtSlot(QImage)
	def on_receive_frame(self, image):
		self.view_webcam.setPixmap(QPixmap.fromImage(image))
        
	@pyqtSlot()
	def on_webcam_terminated(self):
		print('<INFO> Webcam is terminated.')
		self.view_webcam.setText('Camera1')
		
		
