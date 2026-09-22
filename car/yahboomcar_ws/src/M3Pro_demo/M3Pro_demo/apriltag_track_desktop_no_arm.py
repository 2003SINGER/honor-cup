#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
import message_filters
from M3Pro_demo.vutils import draw_tags
from M3Pro_demo.compute_joint5 import *
from dt_apriltags import Detector
from cv_bridge import CvBridge
import cv2 as cv
from arm_interface.srv import ArmKinemarics
from arm_interface.msg import AprilTagInfo,CurJoints
from arm_msgs.msg import ArmJoints
from std_msgs.msg import Float32,Bool,Int16,UInt16
encoding = ['16UC1', '32FC1']
import time
import transforms3d as tfs
import tf_transformations as tf
import yaml
import math
from M3Pro_demo.follow_common import *
from rclpy.node import Node
import rclpy
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


offset_file = "/home/jetson/yahboomcar_ws/src/arm_kin/param/offset_value.yaml"
with open(offset_file, 'r') as file:
    offset_config = yaml.safe_load(file)
print(offset_config)
print("----------------------------")
print("x_offset: ",offset_config.get('x_offset'))
print("y_offset: ",offset_config.get('y_offset'))
print("z_offset: ",offset_config.get('z_offset'))


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
		self.Center_x_list = []
		self.Center_y_list = []
		self.CurEndPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
		self.EndToCamMat = np.array([[ 0 ,0 ,1 ,-1.00e-01],
									 [-1  ,0 ,0  ,0],
									 [0  ,-1  ,0 ,4.82000000e-02],
									 [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])
        
		self.pos_info_pub = self.create_publisher(AprilTagInfo,"PosInfo",1)
		self.CmdVel_pub = self.create_publisher(Twist,"cmd_vel",1)
		self.sub_grasp_status = self.create_subscription(Bool,"grasp_done",self.get_graspStatusCallBack,100)
		self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 10)
		self.TargetJoint5_pub = self.create_publisher(Int16, "set_joint5", 10)
		self.rgb_image_sub = Subscriber(self, Image, '/camera/color/image_raw')
		self.depth_image_sub = Subscriber(self, Image, '/camera/depth/image_raw')
		self.client = self.create_client(ArmKinemarics, 'get_kinemarics')
		self.pub_cur_joints = self.create_publisher(CurJoints,"Curjoints",1)
		self.pub_beep = self.create_publisher(UInt16, "beep", 10)
		self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 1, 0.5)
		self.ts.registerCallback(self.callback)
		self.get_current_end_pos()
		self.pubSix_Arm(self.init_joints)
		self.pubCurrentJoints()
		self.x_offset = offset_config.get('x_offset')
		self.y_offset = offset_config.get('y_offset')
		self.z_offset = offset_config.get('z_offset')
		self.adjust_dist = True
		self.done_flag = True
		self.prev_dist = 0
		self.grasp_Dist = 220.0
		self.start = 0.0
		self.scale = 1000
		self.joint5 = Int16()
		self.move_flag = True 
		self.RemovePID = (60, 0, 20.0)
		self.PID_init()
        
	def Beep_Loop(self):
		beep =UInt16()
		beep.data = 1
		self.pub_beep.publish(beep)
		time.sleep(1.0)
		beep.data = 0
		self.pub_beep.publish(beep)  

	def pubCurrentJoints(self):
		cur_joints = CurJoints()
		cur_joints.joints = self.init_joints
		self.pub_cur_joints.publish(cur_joints) 
    
	def get_graspStatusCallBack(self,msg):
		if msg.data == True:
			self.pubPos_flag = False
			self.done_flag = True
			self.move_flag = True
			time.sleep(1.0)

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

	def pubVel(self,vx,vy,vz):
		vel = Twist()
		vel.linear.x = float(vx)
		vel.linear.y = float(vy)
		vel.angular.z = float(vz)
		self.CmdVel_pub.publish(vel)
        
 
	def get_current_end_pos(self):
		request = ArmKinemarics.Request()
		request.cur_joint1 = float(self.init_joints[0])
		request.cur_joint2 = float(self.init_joints[1])
		request.cur_joint3 = float(self.init_joints[2])
		request.cur_joint4 = float(self.init_joints[3])
		request.cur_joint5 = float(self.init_joints[4])
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
			print("self.CurEndPose: ",self.CurEndPos)
		except Exception as e:
			self.get_logger().error(f'Service call failed: {e}')

	def callback(self,color_frame,depth_frame):
        
        #rgb_image
		rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
		result_image = np.copy(rgb_image)
		result_image = cv.resize(result_image, (640, 480))
		#depth_image
		depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
		depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1.0), cv2.COLORMAP_JET)
		frame = cv.resize(depth_image, (640, 480))
		depth_image_info = frame.astype(np.float32)
		tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
		#tags = sorted(tags, key=lambda tag: tag.tag_id) # 貌似出来就是升序排列的不需要手动进行排列
		draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))

		key = cv2.waitKey(1)

            
		if len(tags) > 0 :
			center_x, center_y = tags[0].center
			if (abs(center_x-320) >10 or abs(center_y-300)>10) and self.move_flag == True: 
				print("adjusting.")
				self.remove_obstacle(center_x, center_y)
				self.start = time.time()

            
			elif abs(center_x-320) <10 and abs(center_y-300)<10 and self.pubPos_flag == False and self.done_flag==True:
				self.pubVel(0,0,0) 
				if time.time() - self.start >3:
					self.Beep_Loop()
					print("---------------")
					self.pubPos_flag = True
					self.move_flag = False
        
			if self.pubPos_flag == True:
				tag = AprilTagInfo()
				tag.id = tags[0].tag_id
				tag.x = center_x
				tag.y = center_y
				tag.z = depth_image_info[int(tag.y),int(tag.x)]/1000
				vx = int(tags[0].corners[0][0]) - int(tags[0].corners[1][0])
				vy = int(tags[0].corners[0][1]) - int(tags[0].corners[1][1])
				target_joint5 = compute_joint5(vx,vy)
				print("target_joint5: ",target_joint5)
				self.joint5.data = int(target_joint5)                        
				if tag.z!=0:
					self.pubPos_flag = False
					self.done_flag = False
					self.TargetJoint5_pub.publish(self.joint5)
					time.sleep(1.0)
					self.pos_info_pub.publish(tag)                    
				else:
					print("Invalid distance.")        
				     
		else:
			self.pubVel(0,0,0)
		result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
		cur_time = time.time()
		fps = str(int(1/(cur_time - self.pr_time)))
		self.pr_time = cur_time
		cv2.putText(result_image, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
		cv2.imshow("result_image", result_image)
		#cv2.imshow("depth_image", depth_to_color_image)


	def compute_heigh(self,x,y,z):
		camera_location = self.pixel_to_camera_depth((x,y),z)
        #print("camera_location: ",camera_location)
		PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
        #PoseEndMat = np.matmul(self.xyz_euler_to_mat(camera_location, (0, 0, 0)),self.EndToCamMat)
		EndPointMat = self.get_end_point_mat()
		WorldPose = np.matmul(EndPointMat, PoseEndMat) 
        #WorldPose = np.matmul(PoseEndMat,EndPointMat)
		pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)
		pose_T[0] = pose_T[0] + self.x_offset
		pose_T[1] = pose_T[1] + self.y_offset
		pose_T[2] = pose_T[2] + self.z_offset
		#print("pose_T: ",pose_T)
		#print("pose_R: ",pose_R)
		return pose_T

	def get_end_point_mat(self):
        #print("Get the current pose is ",self.CurEndPos)
		end_w,end_x,end_y,end_z = self.euler_to_quaternion(self.CurEndPos[3],self.CurEndPos[4],self.CurEndPos[5])
		endpoint_mat = self.xyz_quat_to_mat([self.CurEndPos[0],self.CurEndPos[1],self.CurEndPos[2]],[end_w,end_x,end_y,end_z])
        #print("endpoint_mat: ",endpoint_mat)
		return endpoint_mat
        
	def pixel_to_camera_depth(self,pixel_coords, depth):
		fx, fy, cx, cy = self.camera_info_K[0],self.camera_info_K[4],self.camera_info_K[2],self.camera_info_K[5]
		px, py = pixel_coords
		x = (px - cx) * depth / fx
		y = (py - cy) * depth / fy
		z = depth
		return np.array([x, y, z])
        
	def xyz_euler_to_mat(self,xyz, euler, degrees=False):
		if degrees:
			mat = tfs.euler.euler2mat(math.radians(euler[0]), math.radians(euler[1]), math.radians(euler[2]))
		else:
			mat = tfs.euler.euler2mat(euler[0], euler[1], euler[2])
		mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
		return mat 
        
	def euler_to_quaternion(self,roll,pitch, yaw):
		quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
		qw = quaternion[3]
		qx = quaternion[0]
		qy = quaternion[1]
		qz = quaternion[2]
        #print("quaternion: ",quaternion )
		return np.array([qw, qx, qy, qz])
        
	def xyz_quat_to_mat(self,xyz, quat):
		mat = tfs.quaternions.quat2mat(np.asarray(quat))
		mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
		return mat
        
	def mat_to_xyz_euler(self,mat, degrees=False):
		t, r, _, _ = tfs.affines.decompose(mat)
		if degrees:
			euler = np.degrees(tfs.euler.mat2euler(r))
		else:
			euler = tfs.euler.mat2euler(r)
		return t, euler

	def remove_obstacle(self,point_x, point_y):
		[y, x] = self.Remove_PID_controller.update([(point_x - 320) / 10.0, (point_y - 300) / 10.0])
		if x >= 0.10: x = 0.10 #0.10
		elif x <= -0.10: x = -0.10
		if y >= 0.10: y = 0.10
		elif y <= -0.10: y = -0.10
		self.pubVel(x, y,0) 

	def PID_init(self): 
		self.Remove_PID_controller = simplePID(
            [0, 0],
            [self.RemovePID[0] / 1.0 / (self.scale), self.RemovePID[0] / 1.0 / (self.scale)],
            [self.RemovePID[1] / 1.0 / (self.scale), self.RemovePID[1] / 1.0 / (self.scale)],
            [self.RemovePID[2] / 1.0 / (self.scale), self.RemovePID[2] / 1.0 / (self.scale)]) 
		   
def main():
	print('----------------------')
	rclpy.init()
	apriltag_detect = AprilTagDetectNode('ApriltagDetect_node')
	rclpy.spin(apriltag_detect)

