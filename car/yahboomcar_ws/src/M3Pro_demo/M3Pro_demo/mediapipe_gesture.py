import cv2
import os
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge
import cv2 as cv
from arm_msgs.msg import ArmJoints
import time
from M3Pro_demo.media_library import *
from rclpy.node import Node
import rclpy
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
import threading

print('init done')
class MediapipeDetectNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.init_joints = [90, 150, 12, 20, 90, 0]
		self.rgb_bridge = CvBridge()
		self.depth_bridge = CvBridge()
		self.hand_detector = HandDetector()
		self.pr_time = time.time()
		self.pTime = time.time()
		self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 10)
		self.rgb_image_sub = Subscriber(self, Image, '/camera/color/image_raw')
		while not self.TargetAngle_pub.get_subscription_count():
			self.pubSix_Arm(self.init_joints)
			time.sleep(0.1)
		self.pubSix_Arm(self.init_joints)
		self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub], 1, 0.5)
		self.ts.registerCallback(self.callback)
		time.sleep(2)
		self.start_time = 0.0
		



	def pubSix_Arm(self, joints, id=6, angle=180.0, runtime=2000):
		arm_joint =ArmJoints()
		arm_joint.joint1 = joints[0]
		arm_joint.joint2 = joints[1]
		arm_joint.joint3 = joints[2]
		arm_joint.joint4 = joints[3]
		arm_joint.joint5 = joints[4]
		arm_joint.joint6 = joints[5]
		arm_joint.time = runtime
		self.TargetAngle_pub.publish(arm_joint)

	def callback(self,color_msg):
		rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_msg, "bgr8")
		self.process(rgb_image)


	def process(self, frame):
        #frame = cv.flip(frame, 1)
		frame, lmList, bbox = self.hand_detector.findHands(frame)
		if len(lmList) != 0:
			gesture = threading.Thread(target=self.Gesture_Detect_threading, args=(lmList,bbox))
			gesture.start()
			gesture.join()
		cTime = time.time()
		fps = 1 / (cTime - self.pTime)
		self.pTime = cTime
		text = "FPS : " + str(int(fps))
		cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        #self.media_ros.pub_imgMsg(frame)
		if cv.waitKey(1) & 0xFF == ord('q'):
			cv.destroyAllWindows()
		cv.imshow('frame', frame)

	def Gesture_Detect_threading(self, lmList,bbox):
		gesture = self.hand_detector.get_gesture(lmList)
		print("gesture: ",gesture)
			

def main():
	print('----------------------')
	rclpy.init()
	mediapipe_detect = MediapipeDetectNode('MediapipeDetect_node')
	rclpy.spin(mediapipe_detect)
