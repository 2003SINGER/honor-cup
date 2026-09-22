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
from X5Plus_demo.Dofbot_Track import *
from X5Plus_demo.Robot_Move import *
from dofbot_pro_interface.srv import DofbotProKinemarics
from dofbot_pro_interface.msg import AprilTagInfo,Position,CurJoints
from arm_msgs.msg import ArmJoints
from std_msgs.msg import Float32,Bool
encoding = ['16UC1', '32FC1']
import time

from rclpy.node import Node
import rclpy
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
print('init done')
class KCFTrackNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.init_joints = [90, 150, 12, 20, 90, 30]
		self.dofbot_tracker = DofbotTrack('Tracker')
		self.rgb_bridge = CvBridge()
		self.depth_bridge = CvBridge()
		self.pubPos_flag = False
		self.pr_time = time.time()
		self.cnt = 0
		self.cur_distance = 0.0
		self.track_flag = True
		self.pub_pos_flag = True
		self.prev_dist = 0
		self.prev_angular = 0
		self.minDist = 200
		self.linear_PID = (0.5, 0.0, 0.2)
		self.angular_PID = (0.5, 0.0, 0.2)
		self.pub_SixTargetAngle = self.create_publisher(ArmJoints, "arm6_joints", 10)
		self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

		self.pos_sub = Subscriber(self, Position, '/pos_xyz')
		self.sub_grasp_status = self.create_subscription(Bool,"grasp_done",self.get_graspStatusCallBack,100)
		self.pub_CurJoints = self.create_publisher(CurJoints,"Curjoints",1)
		self.pos_info_pub = self.create_publisher(AprilTagInfo,"PosInfo",1)
		self.move_statu_pub = self.create_publisher(Bool,"Move",1)
		self.CmdVel_pub = self.create_publisher(Twist,"cmd_vel",1)
		self.ts = ApproximateTimeSynchronizer([self.pos_sub], 1, 0.5,allow_headerless=True)
		self.ts.registerCallback(self.callback)
		self.PID_init()
		self.pubSixArm(self.init_joints)

	def get_graspStatusCallBack(self,msg):
		if msg.data == True:
			self.track_flag = True
			self.pub_pos_flag = True
			self.dofbot_tracker.pubSixArm(self.init_joints)

	def robot_move(self,point_x,dist):
		if abs(self.prev_dist - dist) > 30:
			self.prev_dist = dist
			return
		if abs(self.prev_angular - point_x) > 300:
			self.prev_angular = point_x
			return
		linear_x = self.linear_pid.compute(dist, self.minDist)
		angular_z = self.angular_pid.compute(320, point_x)
		if abs(dist - self.minDist) < 20: linear_x = 0.0
		if abs(point_x - 320.0) < 10: angular_z = 0.0
		self.pubVel(linear_x,angular_z)    

	def pubVel(self,vx,vz):
		vel = Twist()
		vel.linear.x = vx
		vel.angular.z = vz
		self.CmdVel_pub.publish(vel)

	def pubSixArm(self, joints, id=6, angle=180.0, runtime=2000):
		arm_joints =ArmJoints()
		arm_joints.joint1 = 180 - joints[0]
		arm_joints.joint2 = joints[1]
		arm_joints.joint3 = joints[2]
		arm_joints.joint4 = joints[3]
		arm_joints.joint5 = joints[4]
		arm_joints.joint6 = joints[5]
		arm_joints.time = runtime
		self.pub_SixTargetAngle.publish(arm_joints)

	def PID_init(self):
		self.linear_pid = simplePID(self.linear_PID[0] / 1000.0, self.linear_PID[1] / 1000.0, self.linear_PID[2] / 1000.0)
		self.angular_pid = simplePID(self.angular_PID[0] / 100.0, self.angular_PID[1] / 100.0, self.angular_PID[2] / 100.0)
            
	def callback(self,msg):
		center_x, center_y = msg.x,msg.y
		self.cur_distance = msg.z*1000
		if (abs(center_x-320) >10 or abs(self.cur_distance - self.minDist) > 20) and self.track_flag == True:
			self.robot_move(center_x,self.cur_distance)  
            
		if abs(center_x-320) <10 and abs(self.cur_distance - self.minDist)<20:
			self.pubVel(0.0,0.0)
			if self.cur_distance!=0 and self.pub_pos_flag == True:
				print("center_x: ",center_x)
				print("center_y: ",center_y)
				print("take it now!")
				self.track_flag = False
				self.pub_pos_flag = False
				move_flag = Bool()
				move_flag.data = False
				self.move_statu_pub.publish(move_flag)
				arm_joint = CurJoints()
				arm_joint.joints = self.init_joints
				self.pub_CurJoints.publish(arm_joint)
				time.sleep(2.0)
				tag = AprilTagInfo()
				tag.x = center_x
				tag.y = center_y
				tag.z = float(self.cur_distance/1000.0)
				self.pos_info_pub.publish(tag)

		   
def main():
	print('----------------------')
	rclpy.init()
	kcf_track = KCFTrackNode('KCFTrack_node')
	rclpy.spin(kcf_track)

