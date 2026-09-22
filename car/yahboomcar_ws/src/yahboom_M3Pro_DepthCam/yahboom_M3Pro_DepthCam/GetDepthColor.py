from cv_bridge import CvBridge
import cv2
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
encoding = ['16UC1', '32FC1']
class ImagetNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.sub_depth = self.create_subscription(Image,"/camera/depth/image_raw",self.get_DepthImgCallBack,100)		
		self.depth_bridge = CvBridge()	
		
	def get_DepthImgCallBack(self,depth_frame):
		#depth_image
		depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
		depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.35), cv2.COLORMAP_JET)
		key = cv2.waitKey(1)
		cv2.imshow("result_image", depth_to_color_image)
def main():
	print('----------------------')
	rclpy.init()
	depth_color_image = ImagetNode('Depth_Color_Image_node')
	rclpy.spin(depth_color_image)	
