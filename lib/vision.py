import cv2, sys, re, time
import imgfilter
from PyQt5.QtCore import pyqtSignal, QMutexLocker, QMutex, QThread, Qt
from PyQt5.QtGui import QImage

class Webcam(QThread):
	changePixmap = pyqtSignal(QImage)
	streamStarted = pyqtSignal()
	
	def __init__(self, index, parent=None,):
		super(Webcam, self).__init__(parent)
		self.mutex = QMutex()
		self.id = index
		self.stopped = False

	def stop(self):
		with QMutexLocker(self.mutex):
			self.stopped = True
			
	def setup(self):
		self.stopped = False
		self.cap = cv2.VideoCapture(self.id)
		self.cap.set(3, 1280)
		self.cap.set(4, 720)
		if not self.cap.isOpened():
			print('Camera{} is not detected. End program.'.format(self.id))
			self.cap.release()
			return
		self.streamStarted.emit()	
	
	def run(self):
		self.setup()	
		while True:
			ret, img = self.cap.read()
			if ret:
				img = cv2.resize(img, (0,0), fx=0.5, fy=0.5) 
				rgbImage = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
				h, w, ch = rgbImage.shape
				bytesPerLine = ch * w
				convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
				p = convertToQtFormat.scaled(640, 360, Qt.KeepAspectRatio)
				self.changePixmap.emit(p)
			if self.stopped:
				self.cap.release()
				return

	def windowName(self):
		return 'CamID:{} (Click to print coordinate)'.format(self.id)
