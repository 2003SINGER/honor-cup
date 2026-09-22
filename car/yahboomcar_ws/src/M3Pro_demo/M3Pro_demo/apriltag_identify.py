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
from dofbot_pro_interface.srv import DofbotProKinemarics
from dofbot_pro_interface.msg import AprilTagInfo,TargetXYRoll
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
import transforms3d.euler as t3d_euler

offset_file = "/home/jetson/yahboomcar_ws/src/dofbot_pro_info/param/offset_value.yaml"
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
		self.init_joints = [90, 150, 12, 20, 90, 30]
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
		self.EndToCamMat = np.array([[ 0 ,0 ,1 ,-9.90000000e-02],
									 [-1  ,0 ,0  ,0],
									 [0  ,-1  ,0 ,4.90000000e-02],
									 [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])
        
		self.pos_info_pub = self.create_publisher(AprilTagInfo,"PosInfo",1)
		self.Target_pos_pub = self.create_publisher(TargetXYRoll,"xy_roll",1)
		self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 10)
		self.rgb_image_sub = Subscriber(self, Image, '/camera/color/image_raw')
		self.depth_image_sub = Subscriber(self, Image, '/camera/depth/image_raw')
		self.client = self.create_client(DofbotProKinemarics, 'get_kinemarics')
		self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
		self.move_done_sub = self.create_subscription(Bool, '/move_done', self.getMoveDonecallback, 10)
		self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 1, 0.5)
		self.ts.registerCallback(self.callback)
		self.get_current_end_pos()
		self.pubSix_Arm(self.init_joints)
		self.x_offset = -0.0016
		self.y_offset = 0.0016
		self.z_offset = -0.027
		self.target_pos_x = 0.256
		self.target_pos_y = 0.000
		self.target_roll = 3.14
		self.target_position = TargetXYRoll()
		self.camera_matrix = None
		self.dist_coeffs = None
		self.pub_tag = False

	def getMoveDonecallback(self,msg):
		if msg.data == True:
			self.pub_tag = True

	def camera_info_callback(self, msg):
        # 获取相机内参矩阵和畸变系数
		self.camera_matrix = np.array(msg.k).reshape(3, 3)
		self.dist_coeffs = np.array(msg.d)


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

	def get_current_end_pos(self):
		request = DofbotProKinemarics.Request()
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
			#print("self.CurEndPose: ",self.CurEndPos)
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
		tags = sorted(tags, key=lambda tag: tag.tag_id) # 貌似出来就是升序排列的不需要手动进行排列
		draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))

		key = cv2.waitKey(10)
		self.Center_x_list = list(range(len(tags)))
		self.Center_y_list = list(range(len(tags)))
		if key == 32:
			self.pubPos_flag = True
		if len(tags) > 0 :
			for i in range(len(tags)):
				center_x, center_y = tags[i].center
				corners = tags[i].corners
				#cv2.circle(depth_to_color_image,(int(center_x),int(center_y)),1,(255,255,255),10)
				self.Center_x_list[i] = center_x
				self.Center_y_list[i] = center_y
				cx = center_x
				cy = center_y
				cz = depth_image_info[int(cy),int(cx)]/1000
				print("cz: ",cz)
				print("cx: ",cx)
				print("cy: ",cy)
				cv2.circle(result_image,(int(cx),int(cy)),1,(255,255,255),10)
				pose = self.compute_heigh(cx,cy,cz)
				self.target_position.x = pose[0] - self.target_pos_x 
				self.target_position.y = pose[1]
				compute_heigh = round(pose[2],4)*1000 
				heigh = 'heigh: ' + str(compute_heigh) + 'mm'
				dist_detect = math.sqrt(pose[1] ** 2 + pose[0]** 2)
				dist_detect = round(dist_detect,3)*1000
				dist = 'dist: ' + str(dist_detect) + 'mm'
				cv.putText(result_image, heigh, (int(cx)+5, int(cy)-15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
				#cv.putText(result_image, dist, (int(cx)+5, int(cy)+15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
				#print("Pose: ",pose)


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
					print("euler_angles: ",euler_angles)
					self.target_position.roll = self.target_roll - abs(euler_angles[0])
                

				if self.pubPos_flag == True:
					self.pubPos_flag = False
					self.Target_pos_pub.publish(self.target_position)
				if self.pub_tag == True:
					self.pub_tag = False
					center_x, center_y = tags[i].center
					cx = center_x
					cy = center_y
					cz = depth_image_info[int(cy),int(cx)]/1000
					if cz!=0:
						pos = AprilTagInfo()
						pos.id = tags[i].tag_id
						pos.x = center_x
						pos.y = center_y
						pos.z = cz
						self.pos_info_pub.publish(pos)                    
                    
                    
                  
		result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
		cur_time = time.time()
		fps = str(int(1/(cur_time - self.pr_time)))
		self.pr_time = cur_time
		cv2.putText(result_image, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
		cv2.imshow("result_image", result_image)
		#cv2.imshow("depth_image", depth_to_color_image)
		key = cv2.waitKey(1)

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
		print("pose_T: ",pose_T)
		print("pose_R: ",pose_R)
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
		   
def main():
	print('----------------------')
	rclpy.init()
	apriltag_detect = AprilTagDetectNode('ApriltagDetect_node')
	rclpy.spin(apriltag_detect)

