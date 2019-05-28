import cv2, sys, re, time
import imgfilter
from PyQt5.QtCore import pyqtSignal, QMutexLocker, QMutex, QThread, Qt, QObject
from PyQt5.QtGui import QImage



import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Webcam(QObject):

	changePixmap = pyqtSignal(QImage)
	
	def __init__(self, index, parent=None,):
		super(Webcam, self).__init__(parent)
		self.id = index
		self.isEnabled = False
		self.bridge = CvBridge()
		topicName = "usb_cam{}/image_raw".format(self.id)
		rospy.Subscriber(topicName, Image, self.callback)

	def callback(self,data):
		if self.isEnabled:
			try:
				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			except CvBridgeError as e:
				print(e)
			rgbImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
			h, w, ch = rgbImage.shape
			bytesPerLine = ch * w
			convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
			p = convertToQtFormat.scaled(640, 360, Qt.KeepAspectRatio)
			self.changePixmap.emit(p)

	def start(self):
		self.isEnabled = True

	def stop(self):
		self.isEnabled = False

