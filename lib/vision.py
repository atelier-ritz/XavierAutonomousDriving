import cv2, sys, re, time
import imgfilter
from PyQt5.QtCore import pyqtSignal, QMutexLocker, QMutex, QThread, Qt
from PyQt5.QtGui import QImage
#=============================================================================================
# Mouse callback Functions
#=============================================================================================
#def showClickedCoordinate(event,x,y,flags,param):
#    # global mouseX,mouseY
#    if event == cv2.EVENT_LBUTTONDOWN:
#        # mouseX,mouseY = x,y
#        print('Clicked position  x: {} y: {}'.format(x,y))

class Webcam(QThread):
	changePixmap = pyqtSignal(QImage)
	
	def __init__(self, index, parent=None,):
		super(Webcam, self).__init__(parent)
		self.mutex = QMutex()
		self.id = index

	def init(self):
		self.stopped = False
		self.cap = cv2.VideoCapture(0)
		self.cap.set(3, 1280)
		self.cap.set(4, 720)
		if not self.cap.isOpened():
			print('Camera is not detected. End program.')
			self.cap.release()
			return
#		cv2.namedWindow(self.windowName(),16) # cv2.GUI_NORMAL = 16
#		cv2.setMouseCallback(self.windowName(), showClickedCoordinate)
	
	def stop(self):
		with QMutexLocker(self.mutex):
			self.stopped = True
			
	def run(self):
#		startTime = time.time()
#		timestamp = []
		while True:
#			timestamp.append(time.time() - startTime)
			ret, img = self.cap.read()
			if ret:
				img = cv2.resize(img, (0,0), fx=0.5, fy=0.5) 
				rgbImage = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
				h, w, ch = rgbImage.shape
				bytesPerLine = ch * w
				convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
				p = convertToQtFormat.scaled(640, 360, Qt.KeepAspectRatio)
				self.changePixmap.emit(p)
#				cv2.imshow(self.windowName(),img)
			if self.stopped:
#				print timestamp
				self.cap.release()
#				cv2.destroyWindow(self.windowName())
				return
		# Actually, the lines below are not necessary in a while True loop
#		self.stop()
#		self.finished.emit()

	def windowName(self):
		return 'CamID:{} (Click to print coordinate)'.format(self.id)
