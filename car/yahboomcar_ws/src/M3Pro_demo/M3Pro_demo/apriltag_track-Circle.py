#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from X5Plus_demo.vutils import draw_tags
from dt_apriltags import Detector
from cv_bridge import CvBridge
import cv2 as cv
from X5Plus_demo.Dofbot_Track import *
from X5Plus_demo.Robot_Move import *
from X5Plus_demo.compute_pose import *
from dofbot_pro_interface.srv import DofbotProKinemarics
from dofbot_pro_interface.msg import AprilTagInfo,CurJoints
from arm_msgs.msg import ArmJoints
from std_msgs.msg import Float32,Bool
encoding = ['16UC1', '32FC1']
import time
import transforms3d.euler as t3d_euler
import math

from rclpy.node import Node
import rclpy
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

print('init done')
class AprilTagTrackNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.init_joints = [90, 150, 12, 20, 90, 0]
		self.init_joints_float = [90.0, 150.0, 12.0, 20.0, 90.0, 30.0]
		self.dofbot_tracker = DofbotTrack('Tracker')
		self.rgb_bridge = CvBridge()
		self.depth_bridge = CvBridge()
		self.pub_pos_falg = True
		self.pr_time = time.time()
		self.cnt = 0
		self.cur_distance = 0.0
		self.track_flag = True
		self.prev_dist = 0
		self.prev_angular = 0
		self.prev_roll = 0
		self.minDist = 280
		self.xy_track_flag = True
		self.linearx_PID = (0.5, 0.0, 0.2)
		self.lineary_PID = (0.1, 0.0, 0.05)
		self.angz_PID = (30.0, 0.0, 10.0)
		self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

		self.rgb_image_sub = Subscriber(self, Image, '/camera/color/image_raw')
		self.sub_grasp_status = self.create_subscription(Bool,"grasp_done",self.get_graspStatusCallBack,100)
		self.depth_image_sub = Subscriber(self, Image, '/camera/depth/image_raw')
		self.CmdVel_pub = self.create_publisher(Twist,"cmd_vel",1)
		self.pub_CurJoints = self.create_publisher(CurJoints,"Curjoints",1)
		self.pos_info_pub = self.create_publisher(AprilTagInfo,"PosInfo",1)
		self.pub_SixTargetAngle = self.create_publisher(ArmJoints, "arm6_joints", 10)
		self.pub_beep = self.create_publisher(Bool, "beep", 10)
		self.PID_init()
		self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
		self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 1, 0.5)
		self.ts.registerCallback(self.callback)

		self.pubSixArm(self.init_joints)
		self.tag_size = 0.05
		self.camera_matrix = None
		self.dist_coeffs = None
		self.rotation_direction = 1
		self.roll_track_flag = True
		self.start_grasp = False
		self.move_y = True
		self.compute_dist = ComputePose("compute_dist")

	def Beep_Loop(self):
		beep = Bool()
		beep.data = True
		self.pub_beep.publish(beep)
		time.sleep(1.0)
		beep.data = False
		self.pub_beep.publish(beep)

	def PID_init(self):
		self.linearx_pid = simplePID(self.linearx_PID[0] / 1000.0, self.linearx_PID[1] / 1000.0, self.linearx_PID[2] / 1000.0)
		self.lineary_pid = simplePID(self.lineary_PID[0] / 100.0, self.lineary_PID[1] / 100.0, self.lineary_PID[2] / 100.0)
		self.angz_pid = simplePID(self.angz_PID[0] / 100.0, self.angz_PID[1] / 100.0, self.angz_PID[2] / 100.0)

	def camera_info_callback(self, msg):
        # 获取相机内参矩阵和畸变系数
		self.camera_matrix = np.array(msg.k).reshape(3, 3)
		self.dist_coeffs = np.array(msg.d)

	def get_graspStatusCallBack(self,msg):
		if msg.data == True:
			self.pubSixArm(self.init_joints)
			self.track_flag = True
			self.pub_pos_falg = True
            
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

	def robot_move(self,point_x,dist):
		if abs(self.prev_dist - dist) > 30:
			self.prev_dist = dist
			return
		if abs(self.prev_angular - point_x) > 300:
			self.prev_angular = point_x
			return
		linear_x = self.linearx_pid.compute(dist, self.minDist)*0.6
		if abs(dist - self.minDist) < 10: 
			self.start_grasp = True
			self.pubVel(0,0,0)
		self.pubVel(linear_x,0,0)

	def robot_rotation(self,roll):
		roll_degree = abs(math.degrees(roll))
		ang_z = -self.rotation_direction * 0.4
		lin_y = self.rotation_direction * 0.3
		if abs(roll_degree - 175) < 3: 
			self.pubVel(0,0,0)
			self.roll_track_flag = False
			print("**********************")
			print("Rotation done.")
		#print("angular_z: ",angular_z)
		else:
			self.pubVel(0.0,lin_y,ang_z)


	def pubVel(self,vx,vy,vz):
		vel = Twist()
		vel.linear.x = float(vx)
		vel.linear.y = float(vy)
		vel.angular.z = float(vz)
		self.CmdVel_pub.publish(vel)
            
	def callback(self,color_frame,depth_frame):
        # 将画面转为 opencv 格式
		rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
		result_image = np.copy(rgb_image)
		#depth_image
		depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
		#depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1.0), cv2.COLORMAP_JET)
		frame = cv2.resize(depth_image, (640, 480))
		depth_image_info = frame.astype(np.float32)

		tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
		tags = sorted(tags, key=lambda tag: tag.tag_id) 
		draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
		show_frame = threading.Thread(target=self.img_out, args=(result_image,))
		show_frame.start()
		show_frame.join()
		if len(tags) > 0 :
            #print("tag: ",tags)
			center_x, center_y = tags[0].center
			corners = tags[0].corners
			cur_depth = depth_image_info[int(center_y),int(center_x)]
			dist = round(self.cur_distance,3)
			get_dist = self.compute_dist.compute(center_x,center_y,cur_depth/1000.0)
			if get_dist!=[]:
				print("get_dist: ",math.sqrt(get_dist[1] ** 2 + get_dist[0]** 2) )
				self.cur_distance = math.sqrt(get_dist[1] ** 2 + get_dist[0]** 2)*1000

			tag_size = 0.05  # 标记的尺寸（单位：米）
			object_points = np.array([
                [-tag_size / 2, -tag_size / 2, 0],
                [tag_size / 2, -tag_size / 2, 0],
                [tag_size / 2, tag_size / 2, 0],
                [-tag_size / 2, tag_size / 2, 0]
            ], dtype=np.float32)

			image_points = np.array(corners, dtype=np.float32)

            # 使用 solvePnP 计算相机到 AprilTag 的位姿
			retval, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)
			if retval:
				rotation_matrix, _ = cv2.Rodrigues(rvec)
				euler_angles = t3d_euler.mat2euler(rotation_matrix)
				print("euler_angles: ",euler_angles[0])
                
			if euler_angles[0]<0:
				self.rotation_direction = -1
                
			if self.xy_track_flag==True and self.roll_track_flag==True: 
				print("Rotation.")
				self.robot_rotation(euler_angles[0])  
                
			if self.roll_track_flag == False :
				self.robot_move(320,self.cur_distance)

			if self.start_grasp == True:
				self.Beep_Loop()
				time.sleep(1.0)
				cx, cy = tags[0].center
				dist = depth_image_info[int(center_y),int(center_x)]/1000
				if dist!=0:
					self.start_grasp = False
					pos = AprilTagInfo()
					pos.x = cx
					pos.y = cy
					pos.z = dist
					print("pos_info: ",pos)
					self.pos_info_pub.publish(pos)
				else:
					print("Invalid Distance.")
                
		else:
			self.pubVel(0.0,0.0,0.0)  


	def img_out(self,frame):
		frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
		cv2.imshow("result_image", frame)
		key = cv2.waitKey(1)


		   
def main():
	print('----------------------')
	rclpy.init()
	apriltag_track = AprilTagTrackNode('ApriltagTrack_node')
	rclpy.spin(apriltag_track)

           