#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
import message_filters
from X5Plus_demo.vutils import draw_tags
from dt_apriltags import Detector
from cv_bridge import CvBridge
import cv2 as cv
from dofbot_pro_interface.srv import DofbotProKinemarics
from dofbot_pro_interface.msg import AprilTagInfo,ArmJoint
from arm_msgs.msg import ArmJoints
from std_msgs.msg import Float32,Bool
encoding = ['16UC1', '32FC1']
import time

from rclpy.node import Node
import rclpy
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
print('init done')
class AprilTagDetectNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.init_joints = [90, 120, 0, 0, 90, 90]
		self.rgb_bridge = CvBridge()
		self.depth_bridge = CvBridge()
		self.pubPos_flag = False
		self.pr_time = time.time()
		self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

		self.pos_info_pub = self.create_publisher(AprilTagInfo,"PosInfo",1)
		self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 10)
		self.rgb_image_sub = Subscriber(self, Image, '/camera/color/image_raw')
		self.depth_image_sub = Subscriber(self, Image, '/camera/depth/image_raw')
		self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 1, 0.5)
		self.ts.registerCallback(self.callback)
		time.sleep(2)
		self.pubSix_Arm(self.init_joints)


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

	def callback(self,color_msg,depth_msg):
		rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_msg, "bgr8")
		depth_image = self.depth_bridge.imgmsg_to_cv2(depth_msg, "16UC1")
		depth_to_color_image = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=1.0), cv.COLORMAP_JET)
		frame = cv.resize(depth_image, (640, 480))
		depth_image_info = frame.astype(np.float32)
		tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
		draw_tags(rgb_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
		key = cv2.waitKey(10)
		if key == 32:
			self.pubPos_flag = True
		if len(tags) > 0 :
			for i in range(len(tags)):
				center_x, center_y = tags[i].center
				#cv.circle(depth_to_color_image,(int(center_x),int(center_y)),1,(255,255,255),10)
				#print("center_x, center_y: ",center_x, center_y)
				#print("depth: ",depth_image_info[int(center_y),int(center_x)]/1000)
				if self.pubPos_flag == True:
					center_x, center_y = tags[i].center
					cv.circle(rgb_image, (int(center_x),int(center_y)), 10, (0,210,255), thickness=-1)
					pos = AprilTagInfo()
					pos.id = tags[i].tag_id
					pos.x = center_x
					pos.y = center_y
					pos.z = depth_image_info[int(center_y),int(center_x)]/1000
					print("tag_id: ",tags[i].tag_id)
					print("center_x, center_y: ",center_x, center_y)
					print("depth: ",depth_image_info[int(center_y),int(center_x)]/1000)
					if pos.z>0:
						self.pos_info_pub.publish(pos)
						self.pubPos_flag = False
					else:
						print("Invalid distance.")
		
		cv2.imshow("result_image", rgb_image)
		cv2.imshow("depth_image", depth_to_color_image)
		key = cv2.waitKey(1)
		   
def main():
	print('----------------------')
	rclpy.init()
	apriltag_detect = AprilTagDetectNode('ApriltagDetect_node')
	rclpy.spin(apriltag_detect)

