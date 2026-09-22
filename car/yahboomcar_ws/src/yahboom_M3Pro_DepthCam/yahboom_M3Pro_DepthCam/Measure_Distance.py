from cv_bridge import CvBridge
import cv2
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
import numpy as np
encoding = ['16UC1', '32FC1']
class ImagetNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.sub_depth = self.create_subscription(Image,"/camera/depth/image_raw",self.get_DepthImgCallBack,100)		
		self.depth_bridge = CvBridge()	
		self.window_name = "result_image"
		self.y = 240
		self.x = 320


	def click_callback(self, event, x, y, flags, params):
		if event == 1:
			self.x = x
			self.y = y

	
	def get_DepthImgCallBack(self,depth_frame):
		#depth_image
		depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
		depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.45), cv2.COLORMAP_JET)
		depth_image_info = depth_image.astype(np.float32)
		dist = depth_image_info[self.y,self.x]
		dist = round(dist,3)
		dist = 'dist: ' + str(dist) + 'mm'
		cv2.putText(depth_to_color_image, dist,  (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
		cv2.setMouseCallback(self.window_name, self.click_callback)
		cv2.circle(depth_to_color_image,(self.x,self.y),1,(0,0,0),10)
		cv2.imshow("result_image", depth_to_color_image)
		key = cv2.waitKey(1)
		
def main():
	print('----------------------')
	rclpy.init()
	depth_color_image = ImagetNode('Depth_Color_Image_node')
	rclpy.spin(depth_color_image)	
