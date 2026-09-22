#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from std_msgs.msg import Bool
import time
import math
from arm_msgs.msg import ArmJoints
from arm_msgs.msg import ArmJoint
from arm_interface.srv import *
import transforms3d as tfs
import tf_transformations as tf
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist


class M3ProMoveNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.init_joints = [90, 150, 12, 20, 90, 0]
		self.CurEndPos =[0.11960304060136814, -0.003155561702528526, 0.3441364762143022, -3.151713610590853e-06, -0.03490854071745721, -0.028421869689521588]
		self.pub_SixTargetAngle = self.create_publisher(ArmJoints, "arm6_joints", 10)
		self.sub_start = self.create_subscription(Bool,"/start_dancing",self.startFlagCallBack,100)
		self.CmdVel_pub = self.create_publisher(Twist,"cmd_vel",1)
		self.client = self.create_client(ArmKinemarics, 'get_kinemarics')
		#self.sub_target_pos = self.create_subscription(Odometry,"/odom",self.get_odom_info,100)
		while not self.client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available, waiting again...')			 
		self.cur_joints = self.init_joints
		self.direction = 1
		self.get_current_end_pos()
		time.sleep(5)
		self.done = True
		self.runtime = 10
		self.vx = 0.1
		self.pubSixArm(self.init_joints)
		print('init done')
        

	def pubVel(self,vx,vy,vz):
		vel = Twist()
		vel.linear.x = float(vx)
		vel.linear.y = float(vy)
		vel.angular.z = float(vz)
		self.CmdVel_pub.publish(vel)

	def base_move(self,base_dir):
		self.vx = self.direction * 0.1
		self.pubVel(self.vx,0.0,0.0)
		time.sleep(self.runtime)
		self.pubVel(0.0,0.0,0.0)
		self.direction = -self.direction
        
	def arm_move(self,arm_dir):
		print("------------------------------------------------")
		#print("pose_T: ",pose_T)
		request = ArmKinemarics.Request()
		request.tar_x = self.CurEndPos[0] + arm_dir*(abs(self.vx)*self.runtime)/10
		request.tar_y = self.CurEndPos[1] 
		request.tar_z = math.tan(-0.03490672894389764) * (request.tar_x - self.CurEndPos[0]) + self.CurEndPos[2]
		print("request.tar_x: ",request.tar_x)
		print("request.tar_z: ",request.tar_z)
		request.kin_name = "ik"
		request.roll = self.CurEndPos[3]
		request.pitch  = self.CurEndPos[4]
		request.yaw = self.CurEndPos[5]
		print("calcutelate_request: ",request)
		future = self.client.call_async(request)
		future.add_done_callback(self.get_ik_respone_callback)
    
	def get_ik_respone_callback(self, future):
		try:
			response = future.result()
			joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
			joints[0] = int(response.joint1) #response.joint1
			joints[1] = int(response.joint2)
			joints[2] = int(response.joint3)
			joints[3] = int(response.joint4)
			joints[4] = 90
			joints[5] = 30
			print("compute_joints: ",joints)
			self.cur_joints = joints
			self.pubSixArm(joints)
			time.sleep(1.5)
			self.done = True
		except Exception as e:
			self.get_logger().error(f'Service call failed: {e}')
  
	def pubSixArm(self, joints, id=6, angle=180.0, runtime=4500):
		arm_joints =ArmJoints()
		arm_joints.joint1 = 180 - joints[0]
		arm_joints.joint2 = joints[1]
		arm_joints.joint3 = joints[2]
		arm_joints.joint4 = joints[3]
		arm_joints.joint5 = joints[4]
		arm_joints.joint6 = joints[5]
		arm_joints.time = self.runtime*900
		self.pub_SixTargetAngle.publish(arm_joints)

		
	def get_current_end_pos(self):
		request = ArmKinemarics.Request()
		request.cur_joint1 = float(self.cur_joints[0])
		request.cur_joint2 = float(self.cur_joints[1])
		request.cur_joint3 = float(self.cur_joints[2])
		request.cur_joint4 = float(self.cur_joints[3])
		request.cur_joint5 = float(self.cur_joints[4])
		request.kin_name = "fk"
		future = self.client.call_async(request)
		future.add_done_callback(self.get_fk_respone_callback)

	def get_fk_respone_callback(self, future):
		try:
			response = future.result()
			#self.get_logger().info(f'Response received: {response.x}')
			self.CurEndPos[0] = response.x
			self.CurEndPos[1] = response.y
			self.CurEndPos[2] = response.z
			self.CurEndPos[3] = response.roll
			self.CurEndPos[4] = response.pitch
			self.CurEndPos[5] = response.yaw
			#self.get_logger().info(f'Response received: {self.CurEndPos}')
			print("self.CurEndPose: ",self.CurEndPos)
			self.done = True
		except Exception as e:
			self.get_logger().error(f'Service call failed: {e}')
            
	def startFlagCallBack(self,msg):
		if msg.data == True:
			'''if self.done == True:
				self.done = False'''
				#self.get_current_end_pos()
			base = threading.Thread(target=self.base_move, args=(self.direction,))
			arm = threading.Thread(target=self.arm_move, args=(-self.direction,))
			
			arm.start()
			base.start()
				
def main():
	print('----------------------')
	rclpy.init()
	M3Pro_Dancing = M3ProMoveNode('M3ProMove_Node')
	rclpy.spin(M3Pro_Dancing)
