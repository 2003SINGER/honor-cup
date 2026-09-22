import cv2
import os
import numpy as npX5Plus
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge
import cv2 as cv
from arm_msgs.msg import ArmJoints
from std_msgs.msg import Bool,Int16,UInt16
from geometry_msgs.msg import Twist
import time
from M3Pro_demo.media_library import *
from rclpy.node import Node
import rclpy
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
import threading
import collections

encoding = ['16UC1', '32FC1']
print('init done')
class MediapipeDetectNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.init_joints = [90, 150, 12, 20, 90, 0]
		self.rgb_bridge = CvBridge()
		self.hand_detector = HandDetector()
		self.pub_gesture = True
		self.cnt = 0
		self.last_sum = 0
		self.pTime = self.cTime = 0
		self.pub_GesturetId = self.create_publisher(Int16,"GesturetId",1)
		self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 10)
		while not self.TargetAngle_pub.get_subscription_count():
			self.pubSix_Arm(self.init_joints)
			time.sleep(0.1)	
		self.pubSix_Arm(self.init_joints)
		self.rgb_image_sub = Subscriber(self, Image, '/camera/color/image_raw')
		self.depth_image_sub = Subscriber(self, Image, '/camera/depth/image_raw')
		self.sub_reset_gesture = self.create_subscription(Bool,"reset_gesture",self.get_resetCallBack,100)
		self.pub_beep = self.create_publisher(UInt16, "beep", 10)
		self.pubSix_Arm(self.init_joints)
		self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub], 1, 0.5)
		self.ts.registerCallback(self.callback)
		time.sleep(2)

		self.frame_times = collections.deque(maxlen=30)

	def Beep_Loop(self):
		beep = UInt16()
		beep.data = 1
		self.pub_beep.publish(beep)
		time.sleep(1.0)
		beep.data = 0
		self.pub_beep.publish(beep)

	
	def get_resetCallBack(self,msg):
		if msg.data == True:
			self.pub_gesture = True
			self.last_sum = 0
			self.cnt = 0
        

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
		if len(lmList) != 0 and self.pub_gesture == True:
			gesture = threading.Thread(target=self.Gesture_Detect_threading, args=(lmList,bbox))
			gesture.start()
			gesture.join()
		self.cTime = time.time()
		self.frame_times.append(self.cTime)
		if len(self.frame_times) >= 2:
			fps = 1 / (self.frame_times[-1] - self.frame_times[0])
		else:
			fps = 0
		self.pTime = self.cTime
		text = "FPS : " + str(int(fps))
		cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        #self.media_ros.pub_imgMsg(frame)
		if cv.waitKey(1) & 0xFF == ord('q'):
			cv.destroyAllWindows()
		cv.imshow('frame', frame)

	def Gesture_Detect_threading(self, lmList,bbox):
		fingers = self.hand_detector.fingersUp(lmList)
		print("sum of fingers: ",sum(fingers))
		
		print(self.pub_gesture)
		if sum(fingers) == self.last_sum:
			print("---------------------------")
			self.cnt = self.cnt + 1
			print("cnt: ",self.cnt)
			if self.cnt==30 and self.pub_gesture == True:
				self.Beep_Loop()
				print("sum of fingers: ",self.last_sum)
				self.pub_gesture = False
				sum_gesture = Int16()
				sum_gesture.data = self.last_sum   
				self.pub_GesturetId.publish(sum_gesture)
				
		else:
			self.cnt = 0
		self.last_sum = sum(fingers)
			

def main():
	print('----------------------')
	rclpy.init()
	mediapipe_detect = MediapipeDetectNode('MediapipeDetect_node')
	rclpy.spin(mediapipe_detect)

