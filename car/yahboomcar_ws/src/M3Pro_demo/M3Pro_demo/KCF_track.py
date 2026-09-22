import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
import message_filters
from M3Pro_demo.vutils import draw_tags
from dt_apriltags import Detector
from cv_bridge import CvBridge
import cv2 as cv
from M3Pro_demo.Robot_Move import *
from M3Pro_demo.compute_pose import *
from arm_interface.srv import ArmKinemarics
from arm_interface.msg import AprilTagInfo,Position,CurJoints
from arm_msgs.msg import ArmJoints
from std_msgs.msg import Float32,Bool,UInt16
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
		self.minDist = 300
		self.grasp_Dist = 220
		self.linearx_PID = (0.5, 0.0, 0.2)
		self.lineary_PID = (0.2, 0.0, 0.1)
		self.angz_PID = (0.5, 0.0, 0.5)
		self.rotation_direction = 1
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
		self.reset_pub = self.create_publisher(Bool,"reset_flag",1)
		self.CmdVel_pub = self.create_publisher(Twist,"cmd_vel",1)
		self.pub_beep = self.create_publisher(UInt16, "beep", 10)
		self.pubSixArm(self.init_joints)
		self.ts = ApproximateTimeSynchronizer([self.pos_sub], 1, 0.5,allow_headerless=True)
		self.ts.registerCallback(self.callback)
		self.PID_init()
		
		self.compute_dist = ComputePose("compute_dist")
		self.x_roll_track_flag = True
		self.y_roll_track_flag = False
		self.x_track_done = False
		self.roll_track_done = False
		self.y_track_done = False
		self.adjust_dist = False
		self.start_grasp = False

	def get_graspStatusCallBack(self,msg):
		if msg.data == True:
			self.pubSixArm(self.init_joints)
			self.track_flag = True
			self.x_track_done = False
			self.roll_track_done = False
			self.y_track_done = False

	def move_dist(self,dist):
		if abs(self.prev_dist - dist) > 30:
			self.prev_dist = dist
			return
		linear_x = self.linearx_pid.compute(dist, self.grasp_Dist)
		if abs(dist - self.grasp_Dist) < 30: 
			linear_x = 0
			self.adjust_dist = False
			self.start_grasp = True
		self.pubVel(linear_x,0,0)

	def robot_move(self,point_x,dist,roll):
		#roll_degree = abs(math.degrees(roll))
		print("point_x: ",point_x)
		print("dist: ",dist)
		#print("roll_degree: ",roll_degree)
		if abs(self.prev_dist - dist) > 30:
			self.prev_dist = dist
			return
		if abs(self.prev_angular - point_x) > 300:
			self.prev_angular = point_x
			return
		linear_x = self.linearx_pid.compute(dist, self.minDist)
		angular_z = self.angz_pid.compute(320, point_x)
		if abs(dist - self.minDist) < 30: 
			linear_x = 0
			self.x_track_done = True
		else:
			self.x_track_done = False
		if abs(point_x - 320) < 10:
			angular_z = 0.0
			self.roll_track_done = True
		else:
			self.roll_track_done = False
		print("linear_x: ",linear_x)
		print("angular_z: ",angular_z)
		#print("angular_z: ",angular_z)
		self.pubVel(linear_x,0,angular_z)
		if self.x_track_done == True and self.roll_track_done == True:
			print("adjust dist to grasp it.")
			self.track_flag = False  
			self.adjust_dist = True
		else:
			self.track_flag = True
			self.adjust_dist = False

	def pubVel(self,vx,vy,vz):
		vel = Twist()
		vel.linear.x = float(vx)
		vel.linear.y = float(vy)
		vel.angular.z = float(vz)
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
		self.linearx_pid = simplePID(self.linearx_PID[0] / 1000.0, self.linearx_PID[1] / 1000.0, self.linearx_PID[2] / 1000.0)
		self.lineary_pid = simplePID(self.lineary_PID[0] / 100.0, self.lineary_PID[1] / 100.0, self.lineary_PID[2] / 100.0)
		self.angz_pid = simplePID(self.angz_PID[0] / 100.0, self.angz_PID[1] / 100.0, self.angz_PID[2] / 100.0)
            
	def callback(self,msg):
		center_x, center_y = msg.x,msg.y
		cur_depth = msg.z
		if cur_depth>0.0:  
			get_dist = self.compute_dist.compute(center_x,center_y,cur_depth)
			self.cur_distance = math.sqrt(get_dist[1] ** 2 + get_dist[0]** 2)*1000
				
			if  self.track_flag == True:
				print("Tracking.")
				self.robot_move(center_x,self.cur_distance,3.05)
        
			if self.adjust_dist == True:
				print("adjust_dist.")
				self.move_dist(self.cur_distance)
        
			if self.start_grasp == True:
				
				time.sleep(1.0)
				cx, cy = msg.x,msg.y
				dist = cur_depth
				if dist!=999:
					self.start_grasp = False
					pos = AprilTagInfo()
					pos.x = cx
					pos.y = cy
					pos.z = dist
					print("pos_info: ",pos)
					self.pos_info_pub.publish(pos)
					self.Beep_Loop()
					reset = Bool()
					reset.data  = True
					self.reset_pub.publish(reset)
				else:
					print("Invalid Distance.")
		else:
			print("Too far.")
			self.pubVel(0,0,0)
        
	def Beep_Loop(self):
		beep =UInt16()
		beep.data = 1
		self.pub_beep.publish(beep)
		time.sleep(1.0)
		beep.data = 0
		self.pub_beep.publish(beep)   

		   
def main():
	print('----------------------')
	rclpy.init()
	kcf_track = KCFTrackNode('KCFTrack_node')
	rclpy.spin(kcf_track)

