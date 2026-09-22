import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from arm_msgs.msg import ArmJoints
from arm_msgs.msg import ArmJoint
import time
from M3Pro_demo.media_library import *
from rclpy.node import Node
import rclpy
import threading

print('init done')
class MediapipeDetectNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.init_joints = [90, 150, 12, 20, 90, 0]
		self.rgb_bridge = CvBridge()
		self.hand_detector = HandDetector()
		self.move_flag = True
		self.pr_time = time.time()
		self.pTime = self.cTime = 0
		self.event = threading.Event()
		self.event.set()
		self.arm_status = False
		self.move = False
		self.pub_SingleTargetAngle = self.create_publisher(ArmJoint, "arm_joint", 10)
		self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 10)
		self.sub_rgb = self.create_subscription(Image,"/camera/color/image_raw",self.get_RGBImageCallBack,100)
		time.sleep(2)
		self.pubSix_Arm(self.init_joints)
        

	def pubSix_Arm(self, joints, id=6, angle=180.0, runtime=1500):
		arm_joint =ArmJoints()
		arm_joint.joint1 = joints[0]
		arm_joint.joint2 = joints[1]
		arm_joint.joint3 = joints[2]
		arm_joint.joint4 = joints[3]
		arm_joint.joint5 = joints[4]
		arm_joint.joint6 = joints[5]
		arm_joint.time = runtime
		self.TargetAngle_pub.publish(arm_joint)

	def pubSingleArm(self, joint_id,joint_angle,run_time):
		arm_joint =ArmJoint()
		arm_joint.joint = joint_angle
		arm_joint.id = joint_id
		arm_joint.time = run_time
		self.pub_SingleTargetAngle.publish(arm_joint)

	def get_RGBImageCallBack(self,color_msg):
		rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_msg, "bgr8")
		self.process(rgb_image)

	def process(self, frame):
        #frame = cv.flip(frame, 1)
		frame, lmList, bbox = self.hand_detector.findHands(frame)
		key = cv2.waitKey(10)
		if key==32:
			self.move = True
		if len(lmList) != 0 and self.move == True:
			gesture = threading.Thread(target=self.Arm_Moving_threading, args=(lmList,bbox))
			gesture.start()
			#gesture.join()
		self.cTime = time.time()
		fps = 1 / (self.cTime - self.pTime)
		self.pTime = self.cTime
		text = "FPS : " + str(int(fps))
		cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        #self.media_ros.pub_imgMsg(frame)
		if cv.waitKey(1) & 0xFF == ord('q'):
			cv.destroyAllWindows()
		cv.imshow('frame', frame)

	def Arm_Moving_threading(self, lmList,bbox):
		if self.event.is_set():
			self.event.clear()
			fingers = self.hand_detector.fingersUp(lmList)
			self.hand_detector.draw = False
			gesture = self.hand_detector.get_gesture(lmList)
			if gesture == "Yes":	
				self.arm_status = False
				self.dance()
				time.sleep(0.5)
				self.init_pose()
				time.sleep(1.0)
				self.arm_status = True
			
			elif gesture == "OK":
				self.arm_status = False
				for i in range(3):
					time.sleep(0.1)
					self.pubSix_Arm([80, 135, 0, 15, 90, 180])
					time.sleep(0.5)
					self.pubSix_Arm([110, 135, 0, 15, 90, 90])
					time.sleep(0.5)
				self.init_pose()
				sleep(1.0)
				self.arm_status = True
			elif gesture == "Thumb_down":
				self.arm_status = False
				self.pubSix_Arm([90, 0, 180, 0, 90, 180])
				time.sleep(0.5)
				self.pubSix_Arm([90, 0, 180, 0, 90, 90])
				time.sleep(1.5)
				self.init_pose()
				time.sleep(1.0)
				self.arm_status = True
			elif sum(fingers) == 1: 
				self.arm_status = False
				self.arm_nod()
				self.init_pose()
				time.sleep(1.0)
				self.arm_status = True
			elif fingers[1] == fingers[4] == 1 and sum(fingers) == 2: 
				self.arm_status = False
				self.shake()
				self.init_pose()
				time.sleep(1.0)
				self.arm_status = True
			elif sum(fingers) == 5: 
				self.arm_status = False
				self.arm_applaud()
				self.init_pose()
				time.sleep(1.0)
				self.arm_status = True
			self.move  = False
			self.event.set()

	def dance(self):
		time_sleep = 0.5
		#time.sleep(time_sleep)
		self.pubSix_Arm([90, 90, 90, 90, 90, 90])
		time.sleep(time_sleep)
		self.pubSix_Arm([90, 60, 120, 60, 90, 90])
		time.sleep(time_sleep)
		self.pubSix_Arm([90, 45, 135, 45, 90, 90])
		time.sleep(time_sleep)
		self.pubSix_Arm([90, 60, 120, 60, 90, 90])
		time.sleep(time_sleep)
		self.pubSix_Arm([90, 90, 90, 90, 90, 90])
		time.sleep(time_sleep)
		self.pubSix_Arm([90, 100, 80, 80, 90, 90])
		time.sleep(time_sleep)
		self.pubSix_Arm([90, 120, 60, 60, 90, 90])
		time.sleep(time_sleep)
		self.pubSix_Arm([90, 135, 45, 45, 90, 90])
		time.sleep(time_sleep)
		self.pubSix_Arm([90, 90, 90, 90, 90, 90])
		time.sleep(time_sleep)
		self.pubSix_Arm([90, 90, 90, 20, 90, 150])
		time.sleep(time_sleep)
		self.pubSix_Arm([90, 90, 90, 90, 90, 90])
		time.sleep(time_sleep)
		self.pubSix_Arm([90, 90, 90, 20, 90, 150])
		time.sleep(time_sleep)
		self.pubSix_Arm([0, 90, 90, 90, 0, 90])
		time.sleep(time_sleep)
		self.pubSix_Arm([0, 90, 180, 0, 0, 90])
		time.sleep(time_sleep)
		self.pubSingleArm(6, 180, 1500)
		time.sleep(time_sleep)
		self.pubSingleArm(6, 30, 1500)
		time.sleep(time_sleep)
		self.pubSix_Arm([90, 90, 90, 90, 90, 90])
		time.sleep(time_sleep)
		self.pubSix_Arm([90, 135, 0, 45, 90, 90])
		time.sleep(time_sleep)
    
	def init_pose(self):
		self.pubSix_Arm([90, 150, 12, 20, 90, 0])
		time.sleep(0.5)
        
	def arm_applaud(self):
		for i in range(3):
			self.pubSix_Arm([90, 145, 0, 71, 90, 31])    
			time.sleep(0.5)
			self.pubSix_Arm([91, 144, 0, 71, 90, 168])    
			time.sleep(0.5)
		self.pubSix_Arm([90, 145, 0, 0, 90, 31])
    
	def shake(self):
		for i in range(3):
			self.pubSix_Arm([138, 94, 92, 88, 92, 172])    
			time.sleep(0.5)
			self.pubSix_Arm([48, 94, 92, 87, 92, 172])    
			time.sleep(0.5)
		self.pubSix_Arm([90, 145, 0, 0, 90, 31])
    
	def arm_nod(self):
		for i in range(3):
			self.pubSix_Arm([82, 89, 93, 93, 89, 32])    
			time.sleep(0.5)
			self.pubSix_Arm([82, 89, 93, 33, 89, 32])    
			time.sleep(0.5)
		self.pubSix_Arm([90, 145, 0, 0, 90, 31])

		   
def main():
	print('----------------------')
	rclpy.init()
	mediapipe_detect = MediapipeDetectNode('MediapipeDetect_node')
	rclpy.spin(mediapipe_detect)

