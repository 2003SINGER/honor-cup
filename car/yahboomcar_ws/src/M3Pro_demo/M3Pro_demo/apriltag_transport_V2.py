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
from arm_msgs.msg import ArmJoint
from std_msgs.msg import Float32,Bool,Int16,UInt16
encoding = ['16UC1', '32FC1']
import time
import transforms3d as tfs
import tf_transformations as tf
import yaml
import math
from M3Pro_demo.Robot_Move import *
from rclpy.node import Node
import rclpy
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped,PoseStamped
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped,Twist
from nav2_msgs.action import NavigateThroughPoses
from visualization_msgs.msg import MarkerArray

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
		self.CurEndPos = [0.1458589529828534, 0.00022969568906952754, 0.18566515428310748, 0.00012389155580734876, 1.0471973953319513, 8.297829493472317e-05]
		self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
		self.EndToCamMat = np.array([[ 0 ,0 ,1 ,-1.00e-01],
									 [-1  ,0 ,0  ,0],
									 [0  ,-1  ,0 ,4.82000000e-02],
									 [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])
        
		self.pos_info_pub = self.create_publisher(AprilTagInfo,"PosInfo",1)
		self.CmdVel_pub = self.create_publisher(Twist,"cmd_vel",1)
		self.sub_grasp_status = self.create_subscription(Bool,"grasp_done",self.get_graspStatusCallBack,100)
		self.sub_init_pose = self.create_subscription(PoseWithCovarianceStamped,"/initialpose",self.get_InitPoseCallBack,1)
		self.sub_start_detect = self.create_subscription(Bool,"/start_detect",self.get_StartDetectCallBack,100)
		self.sub_start_transport = self.create_subscription(Bool,"/start_transport",self.get_StartTransportCallBack,100)
		self.sub_start_back2orin = self.create_subscription(Bool,"/transport_done",self.get_TransbotStatusCallBack,100)
		self.sub_rotation_done= self.create_subscription(Bool,"/rotation_done",self.get_RotationStatusCallBack,100)
		self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 10)
		self.pub_SingleTargetAngle = self.create_publisher(ArmJoint, "arm_joint", 10)
		self.pub_orin_pose = self.create_publisher(PoseStamped, "/goal_pose", 10)
		self.TargetJoint5_pub = self.create_publisher(Int16, "set_joint5", 10)
		self.pub_status = self.create_publisher(Int16, "/next_status", 10)
		self.rgb_image_sub = Subscriber(self, Image, '/camera/color/image_raw')
		self.depth_image_sub = Subscriber(self, Image, '/camera/depth/image_raw')
		self.client = self.create_client(ArmKinemarics, 'get_kinemarics')
		self.pub_cur_joints = self.create_publisher(CurJoints,"Curjoints",1)
		self.pub_beep = self.create_publisher(UInt16, "beep", 10)
		self.pub_no_found = self.create_publisher(Bool, "/No_found", 10)
		self.pub_unload_done = self.create_publisher(Bool, "/unload_done", 10)
		self.pub_detect_res = self.create_publisher(Bool, "/detect_result", 10)
		self.pub_start_rotate = self.create_publisher(Bool, "/start_rotate", 10)
		self.send_back2orin = self.create_publisher(Bool, "/back_orin", 10)
		self.get_current_end_pos()
		self.pubSix_Arm(self.init_joints)
		self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 1, 0.5)
		self.ts.registerCallback(self.callback)
        

		self.pubCurrentJoints()
		self.x_offset = offset_config.get('x_offset')
		self.y_offset = offset_config.get('y_offset')
		self.z_offset = offset_config.get('z_offset')
		self.adjust_dist = True
		self.prev_dist = 0
		self.linearx_PID = (0.3, 0.0, 0.1)
		self.lineary_PID = (0.2, 0.0, 0.2)
		self.linearx_pid = simplePID(self.linearx_PID[0] / 1000.0, self.linearx_PID[1] / 1000.0, self.linearx_PID[2] / 1000.0)
		self.lineary_pid = simplePID(self.lineary_PID[0] / 100.0, self.lineary_PID[1] / 100.0, self.lineary_PID[2] / 100.0)
		self.grasp_Dist = 220.0
		self.joint5 = Int16()
		self.start_transport = False
		self.rotate = False
		self.y_track_done = False
		self.y_track = False
		self.grasp_done = False
		self.orinal_pose = PoseStamped()
		self.orinal_pose.header.frame_id = "map"
		self.start_detect = False
		self.stop_rotate = False
		self.come_back = False
		self.next_navigation = False

	def get_RotationStatusCallBack(self,msg):
		if msg.data == True:
			self.stop_rotate = True
			print("Click on the image screen and press the following buttons to select the next step: ")
			print("N to navigate to the next point")
			print("B to come back to the origin")             
            

	def get_StartDetectCallBack(self,msg):
		if msg.data == True:
			self.start_detect = True
			print("Detect the tag.")
			print("Click on the image screen and press the following buttons to select the next step: ")
			print("n to navigate to the next point")
			print("b to come back to the origin") 
			print("r to start rotating to find the machine block.")

	def get_StartTransportCallBack(self,msg):
		if msg.data == True:
			print("Transbot the tag.")
			if self.grasp_done == True:
				self.unload()
				print("Click on the image screen and press the following buttons to select the next step: ")
				print("n to navigate to the next point")
				print("b to come back to the origin")
				#print("Back to the orin.")
				#self.pub_orin_pose.publish(self.orinal_pose)
                               
	def get_TransbotStatusCallBack(self,msg):
		if msg.data == True:
			#self.rotate  =  True
			self.y_track_done = False
			self.adjust_dist = True
			self.stop_rotate = False
			if self.grasp_done == True:
				self.unload()
			print("Back to the orin,change the rotate status.")
			print("Click on the image screen and press the following buttons to select the next step: ")
			print("n to navigate to the next point")            
			
	def get_InitPoseCallBack(self,msg):
		#print("get the init_pose: ",msg.pose)
		self.orinal_pose.pose.position.x = msg.pose.pose.position.x
		self.orinal_pose.pose.position.y = msg.pose.pose.position.y
		self.orinal_pose.pose.orientation.w = 1.0
        

	def robot_Y_move(self,point_x):
		linera_y = self.lineary_pid.compute(320,point_x)*0.1
		if abs(point_x - 320) < 15:
			linera_y = 0.0
			self.y_track_done = True
		self.pubVel(0.0,linera_y,0.0)
		if self.y_track_done == True:
			self.y_track = False
			self.pubVel(0.0,0.0,0.0)
			print("adjust dist to grasp it.")
			self.pubPos_flag = True
			self.adjust_dist = True

	def unload(self):
		if self.cur_tagId == 1:
			self.down_joint = [155, 35, 70, 5, 60,120]
		elif self.cur_tagId == 2:
			self.down_joint = [180, 35, 60, 0, 90,120]
		elif self.cur_tagId == 3:
			self.down_joint = [28, 30, 70, 2, 90,120]
		elif self.cur_tagId == 4:
			self.down_joint = [0, 43, 48, 6, 90,120]
		self.pubSix_Arm(self.down_joint)
		time.sleep(2.5)
		self.pubSingleArm(6, 30, 2000)
		time.sleep(2.5)
		self.pubSix_Arm(self.init_joints)
		time.sleep(2.5)
		self.Beep_Loop()
		self.grasp_done = False
		unload_status = Bool()
		unload_status.data = True
		self.pub_unload_done.publish(unload_status)
		self.y_track_done = False


	def Beep_Loop(self):
		beep = UInt16()
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
			#self.pubPos_flag = True
			#self.adjust_dist = True
			self.grasp_done = True
			print("Click the rviz window with the mouse to give the next target point, or press the b key on the keyboard to return to the origin")

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

	def pubSingleArm(self, joint_id,joint_angle,run_time):
		arm_joint =ArmJoint()
		arm_joint.joint = joint_angle
		arm_joint.id = joint_id
		arm_joint.time = run_time
		self.pub_SingleTargetAngle.publish(arm_joint)

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
		tags = sorted(tags, key=lambda tag: tag.tag_id) # 貌似出来就是升序排列的不需要手动进行排列
		draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))

		key = cv2.waitKey(10)
		self.Center_x_list = list(range(len(tags)))
		self.Center_y_list = list(range(len(tags)))
		if key == ord('r'):
			self.rotate = True
			self.stop_rotate = False
			start_rotate = Bool()
			start_rotate.data = True
			self.pub_start_rotate.publish(start_rotate)
			print("start to rotate.")
            
		elif key == ord('n'):
			self.next_navigation = True
			self.rotate = False
			self.stop_rotate = False
			#self.start_detect = False
			next_status = Int16()
			next_status.data = 1
			self.pub_status.publish(next_status)
			print("Go to the next point.")
		elif key == ord('b'):
			print("Come back to the orin.")
			self.rotate = False
			self.start_detect = False
			next_status = Int16()
			next_status.data = 0
			self.pub_status.publish(next_status)
			self.come_back = True
			back_orin = Bool()
			back_orin.data = True
			self.send_back2orin.publish(back_orin)
		if self.start_detect == True:   
			if self.rotate == True and len(tags)==0 and self.stop_rotate==False:
				self.pubVel(0,0,0.2)
				print("Rotating to detect the apriltag.")
			elif len(tags)>0 and self.rotate == True :
				detect = Bool()
				detect.data = True
				self.pub_detect_res.publish(detect)
				self.rotate = False
				self.pubVel(0,0,0)  
				self.Beep_Loop()
				print("Found the apriltag.")
				self.y_track = True


			elif self.stop_rotate==True and len(tags)==0:
				#print("Do not found any apriltag.")
				self.pubVel(0,0,0) 
				#self.start_detect = False
				no_found = Bool()
				no_found.data = True
				self.pub_no_found.publish(no_found)
				#self.stop_rotate = False
				'''if self.come_back==True:
					print("Come back to the orin.")
				elif self.next_navigation==True and self.come_back==False:
					self.rotate = False
					self.start_detect = True
					self.stop_rotate = False'''
					#next_status = Int16()
					#next_status.data = 1
					#self.pub_status.publish(next_status)
				
                
		if len(tags) > 0 and self.rotate==False:
			for i in range(len(tags)):
				center_x, center_y = tags[i].center
				#cv2.circle(depth_to_color_image,(int(center_x),int(center_y)),1,(255,255,255),10)
				self.Center_x_list[i] = center_x
				self.Center_y_list[i] = center_y
				cx = center_x
				cy = center_y
				cz = depth_image_info[int(cy),int(cx)]/1000
				'''cv2.circle(result_image,(int(cx),int(cy)),1,(255,255,255),10)
				cv2.circle(result_image,(int(tags[i].corners[0][0]),int(tags[i].corners[0][1])),1,(255,255,0),10)
				cv2.circle(result_image,(int(tags[i].corners[1][0]),int(tags[i].corners[1][1])),1,(255,0,0),10)'''
				pose = self.compute_heigh(cx,cy,cz)
				degree = pose[1]/pose[0]
				#print("degree: ",degree)
				dist_detect = math.sqrt(pose[1] ** 2 + pose[0]** 2)
				dist_detect = dist_detect*1000
				dist = 'dist: ' + str(dist_detect) + 'mm'
				cv.putText(result_image, dist, (int(cx)+5, int(cy)+15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
				vx = int(tags[i].corners[0][0]) - int(tags[i].corners[1][0])
				vy = int(tags[i].corners[0][1]) - int(tags[i].corners[1][1])
				target_joint5 = compute_joint5(vx,vy)
				#print("target_joint5: ",target_joint5)
				if  self.y_track==True:
					self.robot_Y_move(center_x)
				if self.pubPos_flag == True : 
					
					if abs(dist_detect - 200.0)>10 and self.adjust_dist==True:
						self.move_dist(dist_detect)
					else:
						self.adjust_dist = False
						self.pubVel(0,0,0)
						tag = AprilTagInfo()
						tag.id = tags[i].tag_id
						self.cur_tagId = tag.id
						tag.x = self.Center_x_list[i]
						tag.y = self.Center_y_list[i]
						tag.z = depth_image_info[int(tag.y),int(tag.x)]/1000
						#vx = int(tags[i].corners[0][0]) - int(tag.x)
						#vy = int(tags[i].corners[0][1]) - int(tag.y)
						vx = int(tags[i].corners[0][0]) - int(tags[i].corners[1][0])
						vy = int(tags[i].corners[0][1]) - int(tags[i].corners[1][1])
						target_joint5 = compute_joint5(vx,vy)
						print("target_joint5: ",target_joint5)
						self.joint5.data = int(target_joint5)                        
						if tag.z!=0:
							self.TargetJoint5_pub.publish(self.joint5)
							self.pos_info_pub.publish(tag)
							self.pubPos_flag = False
							self.start_detect = False
							next_status = Int16()
							next_status.data = 2
							self.pub_status.publish(next_status)
						else:
							print("Invalid distance.")              
		result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
		cur_time = time.time()
		fps = str(int(1/(cur_time - self.pr_time)))
		self.pr_time = cur_time
		cv2.putText(result_image, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
		cv2.imshow("result_image", result_image)
		#cv2.imshow("depth_image", depth_to_color_image)
		key = cv2.waitKey(1)

	def move_dist(self,dist):
		linear_x = self.linearx_pid.compute(dist, 200)
		#print("lin.x: ",linear_x)
		self.pubVel(linear_x,0,0)

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
		   
def main():
	print('----------------------')
	
	rclpy.init()
	apriltag_detect = AprilTagDetectNode('ApriltagDetect_node')
	rclpy.spin(apriltag_detect)

