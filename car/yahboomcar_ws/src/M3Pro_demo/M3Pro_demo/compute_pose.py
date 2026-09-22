#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
import message_filters
from M3Pro_demo.vutils import draw_tags
from dt_apriltags import Detector
from cv_bridge import CvBridge
import cv2 as cv
from arm_interface.srv import ArmKinemarics
from arm_interface.msg import AprilTagInfo
from arm_msgs.msg import ArmJoints
from std_msgs.msg import Float32,Bool
encoding = ['16UC1', '32FC1']
import time
import transforms3d as tfs
import tf_transformations as tf
import yaml
import math

from rclpy.node import Node
import rclpy
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image


offset_file = "/home/jetson/yahboomcar_ws/src/arm_kin/param/offset_value.yaml"
with open(offset_file, 'r') as file:
    offset_config = yaml.safe_load(file)
print(offset_config)
print("----------------------------")
print("x_offset: ",offset_config.get('x_offset'))
print("y_offset: ",offset_config.get('y_offset'))
print("z_offset: ",offset_config.get('z_offset'))


print('init done')
class ComputePose(Node):
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
		#self.CurEndPos_joint2_100 =[0.13468214978467385, 0.00012488085077297901, 0.1605914780815732, 0.000360345551029897, 1.3962632352230782, 0.00033093215718788563]
		self.CurEndPos_joint2_100 =[0.0942525134759935, -0.0024349099059140476, 0.14848996182768281, -4.340805769083934e-06, 1.3962614348313127, -0.02842314305412809]
		self.CurEndPos_joint2_150 =[0.11960304060136814, -0.003155561702528526, 0.3441364762143022, -3.151713610590853e-06, -0.03490854071745721, -0.028421869689521588]
		self.CurEndPos =[0.15775829711660408, 0.0001113964545386204, 0.3442932716404664, 6.048784709527745e-05, -0.03490672894389764, -2.9475235577497835e-05]
		self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
		self.EndToCamMat = np.array([[ 0 ,0 ,1 ,-1.000e-01],
									 [-1  ,0 ,0  ,0],
									 [0  ,-1  ,0 ,4.82000000e-02],
									 [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])
		self.x_offset = offset_config.get('x_offset')
		self.y_offset = offset_config.get('y_offset')
		self.z_offset = offset_config.get('z_offset')
		self.Invalid_dist = []

	def compute(self,x,y,z):
		if z!=0:

			camera_location = self.pixel_to_camera_depth((x,y),z)
        #print("camera_location: ",camera_location)
			PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
        #PoseEndMat = np.matmul(self.xyz_euler_to_mat(camera_location, (0, 0, 0)),self.EndToCamMat)
			EndPointMat = self.get_end_point_mat(self.CurEndPos_joint2_150)
			WorldPose = np.matmul(EndPointMat, PoseEndMat) 
        #WorldPose = np.matmul(PoseEndMat,EndPointMat)
			pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)
			pose_T[0] = pose_T[0] + self.x_offset*0
			pose_T[1] = pose_T[1] + self.y_offset*0
			pose_T[2] = pose_T[2] + self.z_offset
			#print("pose_T: ",pose_T)
			return pose_T
		else:
			return self.Invalid_dist

	def get_dist(self,x,y,z):
		if z!=0:
			print("get z: ",z)
			camera_location = self.pixel_to_camera_depth((x,y),z)
        #print("camera_location: ",camera_location)
			PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
        #PoseEndMat = np.matmul(self.xyz_euler_to_mat(camera_location, (0, 0, 0)),self.EndToCamMat)
			EndPointMat = self.get_end_point_mat(self.CurEndPos_joint2_100)
			WorldPose = np.matmul(EndPointMat, PoseEndMat) 
        #WorldPose = np.matmul(PoseEndMat,EndPointMat)
			pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)
			pose_T[0] = pose_T[0] + self.x_offset
			pose_T[1] = pose_T[1] + self.y_offset
			pose_T[2] = pose_T[2] + self.z_offset
			#print("pose_T: ",pose_T)
			return pose_T
		else:
			return self.Invalid_dist
    

	def get_end_point_mat(self,Arm_Cur_Pose):
        #print("Get the current pose is ",self.CurEndPos)
		end_w,end_x,end_y,end_z = self.euler_to_quaternion(Arm_Cur_Pose[3],Arm_Cur_Pose[4],Arm_Cur_Pose[5])
		endpoint_mat = self.xyz_quat_to_mat([Arm_Cur_Pose[0],Arm_Cur_Pose[1],Arm_Cur_Pose[2]],[end_w,end_x,end_y,end_z])
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
