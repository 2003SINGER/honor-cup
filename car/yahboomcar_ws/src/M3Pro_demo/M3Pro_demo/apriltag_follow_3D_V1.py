import cv2
import os
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from M3Pro_demo.vutils import draw_tags
from dt_apriltags import Detector
from cv_bridge import CvBridge
import cv2 as cv
from M3Pro_demo.Robot_Move import *
from M3Pro_demo.compute_pose import *
from arm_interface.srv import ArmKinemarics
from arm_interface.msg import AprilTagInfo,CurJoints
from arm_msgs.msg import ArmJoints
from std_msgs.msg import Float32,Bool,Int16,UInt16
encoding = ['16UC1', '32FC1']
import time
import transforms3d.euler as t3d_euler
import math

from rclpy.node import Node
import rclpy
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import threading

from M3Pro_demo.PID import *
from M3Pro_demo.compute_joint5 import *
offset_file = "/root/yahboomcar_ws/src/arm_kin/param/offset_value.yaml"
with open(offset_file, 'r') as file:
    offset_config = yaml.safe_load(file)
print(offset_config)
print("----------------------------")
print("x_offset: ",offset_config.get('x_offset'))
print("y_offset: ",offset_config.get('y_offset'))
print("z_offset: ",offset_config.get('z_offset'))
print('init done')
class AprilTagTrackNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.init_joints = [90, 150, 12, 20, 90, 0]
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
		self.minDist = 200
		self.grasp_Dist = 260
		self.linearx_PID = (0.5, 0.0, 0.2)
		self.lineary_PID = (0.2, 0.0, 0.1)
		self.angz_PID = (0.5, 0.0, 0.5)
		self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
		self.EndToCamMat = np.array([[ 0 ,0 ,1 ,-1.00e-01],
									 [-1  ,0 ,0  ,0],
									 [0  ,-1  ,0 ,4.82000000e-02],
									 [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])
		self.CurEndPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.x_offset = offset_config.get('x_offset')
		self.y_offset = offset_config.get('y_offset')
		self.z_offset = offset_config.get('z_offset')
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
		self.TargetJoint5_pub = self.create_publisher(Int16, "set_joint5", 10)
		self.pub_beep = self.create_publisher(UInt16, "beep", 10)
		self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
		self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 100, 0.1)
		self.ts.registerCallback(self.callback)
		self.pub_cur_joints = self.create_publisher(CurJoints,"Curjoints",1)
		self.client = self.create_client(ArmKinemarics, 'get_kinemarics')
		self.pubSixArm(self.init_joints)

		self.tag_size = 0.05
		self.camera_matrix = None
		self.dist_coeffs = None
		self.rotation_direction = 1
		self.roll_track_flag = True
		self.start_grasp = False
		self.move_y = True
		self.compute_dist = ComputePose("compute_dist")
		self.x_direction = 1
		self.x_roll_track_flag = True
		self.y_roll_track_flag = False
		self.x_track_done = False
		self.roll_track_done = False
		self.y_track_done = False
		self.adjust_dist = False

		self.px = 0.0
		self.py = 0.0

		self.target_servox=90
		self.target_servoy=180

		self.xservo_pid = PositionalPID(0.25, 0.1, 0.05)
		self.yservo_pid = PositionalPID(0.25, 0.1, 0.05)

		self.y_out_range = False
		self.x_out_range = False
        
		self.cur_joints = self.init_joints
		self.get_current_end_pos()
		self.a = 0
		self.b = 0

		self.pubPos_flag = False
		self.XY_Track_flag =False
        
		self.joint5 = Int16()
		self.Grasp_flag = False

		self.Done_flag = True

		self.PID_init()

		self.last_distance = 119.0

		self.adjust_flag = True

		self.compute_flag = True

		self.offset_adjust = 0.0

		self.target_dist = 200.0

		self.last_dist = 0.0

		self.start_time = 0.0

		self.Depth_Track_done = True

		self.XY_Track_done = False

		self.Depth_Track_flag = True

		self.last_cx = 0
        
		self.last_cy = 0

		self.offset_pos = 0.0
        	

	def Beep_Loop(self):
		beep =UInt16()
		beep.data = 1
		self.pub_beep.publish(beep)
		time.sleep(1.0)
		beep.data = 0
		self.pub_beep.publish(beep)

	def PID_init(self):
		self.linearx_pid = simplePID(self.linearx_PID[0] / 1000.0, self.linearx_PID[1] / 1000.0, self.linearx_PID[2] / 1000.0)


	def camera_info_callback(self, msg):
        # 获取相机内参矩阵和畸变系数
		self.camera_matrix = np.array(msg.k).reshape(3, 3)
		self.dist_coeffs = np.array(msg.d)

	def get_graspStatusCallBack(self,msg):
		if msg.data == True:
			self.pubSixArm(self.init_joints)
			self.track_flag = True
			self.x_track_done = False
			self.roll_track_done = False
			self.y_track_done = False
			self.XY_Track_flag = True
			self.pubPos_flag = True
			self.Done_flag = True
            
            
            
	def pubSixArm(self, joints, id=6, angle=180.0, runtime=1000):
		arm_joints =ArmJoints()
		arm_joints.joint1 = 180 - int(joints[0])
		arm_joints.joint2 = joints[1]
		arm_joints.joint3 = joints[2]
		arm_joints.joint4 = joints[3]
		arm_joints.joint5 = joints[4]
		arm_joints.joint6 = joints[5]
		arm_joints.time = runtime
		self.pub_SixTargetAngle.publish(arm_joints)

            
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
		#tags = sorted(tags, key=lambda tag: tag.tag_id) 
		draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
		show_frame = threading.Thread(target=self.img_out, args=(result_image,))
		show_frame.start()
		show_frame.join()
		key = cv2.waitKey(1)
		if key == 32:
			self.compute_flag = True
		if key == 97:
			self.adjust_flag = False
		if len(tags) > 0 :
            #print("tag: ",tags)
			center_x, center_y = tags[0].center
			corners = tags[0].corners
			#print("corners: ",corners)
			cur_depth = depth_image_info[int(center_y),int(center_x)]
			print("cur_depth: ",cur_depth)
			
			if (abs(center_x-320) >20 or abs(center_y-240)>20) and (abs(center_x - self.last_cx)>20 or abs(center_y - self.last_cy)>20):
				self.XY_Track_flag = True
				self.XY_Track_done = False

			else:
				self.XY_Track_flag = False
				self.XY_Track_done = True
				get_end = threading.Thread(target=self.get_current_end_pos, args=())
				get_end.start()
				get_end.join()
				if cur_depth>0 and self.compute_flag==True: 
					res_pos = self.compute_heigh(int(center_x),int(center_y),cur_depth/1000)
					print(self.CurEndPos)
					print("//////////////////////")
					end_pos = math.sqrt(self.CurEndPos[1] ** 2 + self.CurEndPos[0]** 2)
					print("end_pos: ",end_pos)
					print("target_pos: ",res_pos)
					self.offset_pos = res_pos - end_pos
					print("offset_pos: ",self.offset_pos)
    

			if abs(self.offset_pos) > 0.2 and cur_depth>0:
				if time.time() - self.start_time >1: 
					self.Depth_Track_flag = True
					self.Depth_Track_done = False
					self.last_cx = center_x
					self.last_cy = center_y

			elif abs(self.last_dist - cur_depth) > 10 and cur_depth>0:
				self.Depth_Track_flag = False
				#self.Depth_Track_done = True
				self.last_dist = cur_depth
				self.start_time = time.time()
                
			if self.XY_Track_flag==True and self.Depth_Track_flag == False and self.Depth_Track_done==True:
				print("XY_Tracking")
				print("-------------------------------------")
				self.XY_track(center_x,center_y)
				#self.target_dist = cur_depth
				

			elif self.XY_Track_flag==False and self.Depth_Track_flag == True and self.XY_Track_done == True:
				print("Depth_Tracking")
				print("++++++++++++++++++++++++++++++++++++")
				#res = cur_depth - self.target_dist
				res = cur_depth - self.target_dist
				print("res: ",res)
				if cur_depth>0 and abs(self.offset_pos)>0.2:
					self.compute_flag = False
					self.Depth_Track_flag = False
					offset_adjust = self.offset_pos*0.5
					adjust_move = threading.Thread(target=self.adjust, args=(offset_adjust,))
					adjust_move.start()
					adjust_move.join()
					self.offset_pos = 0.0


	def adjust(self,offset):
		print("offset: ",offset)
		request = ArmKinemarics.Request()
		request.tar_x = self.CurEndPos[0]   + offset
		request.tar_y = self.CurEndPos[1] 
		request.tar_z =  self.CurEndPos[2] + offset*0.2
		request.kin_name = "ik"
		request.roll = -3.151713610590853e-06 
		request.pitch  =  0.03490854071745721 
		request.yaw = -0.028421869689521588
		print("request",request)
		future = self.client.call_async(request)
		future.add_done_callback(self.get_ik_respone_callback)
           
	def get_ik_respone_callback(self, future):
		try:
			response = future.result()
			#print("response: ",response)
			joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
			joints[0] = int(response.joint1) 
			joints[1] = int(response.joint2)
			joints[2] = int(response.joint3)
			joints[3] = int(response.joint4)
			joints[4] = 90
			joints[5] = 0
			print("compute_joints: ",joints)
			self.cur_joints = joints
			get_end = threading.Thread(target=self.get_current_end_pos, args=())
			get_end.start()
			get_end.join()
			self.pubSixArm(joints)			
			waitting_ = threading.Thread(target=self.timer_second, args=())
			waitting_.start()
			waitting_.join()

		except Exception as e:
			self.get_logger().error(f'Service call failed: {e}')
                        
	def XY_track(self,center_x,center_y):
        #self.pub_arm(self.init_joint)
		self.px = center_x
		self.py = center_y

		if not (self.target_servox>=180 and center_x<=320 and self.a == 1 or self.target_servox<=0 and center_x>=320 and self.a == 1):
			if(self.a == 0):
				self.xservo_pid.SystemOutput = center_x
				if self.x_out_range == True:
					if self.target_servox<0:
						self.target_servox = 0
						self.xservo_pid.SetStepSignal(630)
					if self.target_servox>0:
						self.target_servox = 180
						self.xservo_pid.SetStepSignal(10)
					self.x_out_range = False
				else:
					self.xservo_pid.SetStepSignal(320)
					self.x_out_range = False
               
				self.xservo_pid.SetInertiaTime(0.01, 0.1)
                
				target_valuex = int(1500 + self.xservo_pid.SystemOutput)
                
				self.target_servox= int((target_valuex - 500) / 10) -10
                #print("self.target_servox:",self.target_servox)
                
				if self.target_servox > 180:
					self.x_out_range = True
                    
				if self.target_servox < 0:
					self.x_out_range = True

		if not (self.target_servoy>=180 and center_y<=240 and self.b == 1 or self.target_servoy<=0 and center_y>=240 and self.b == 1):
			if(self.b == 0):
				self.yservo_pid.SystemOutput = center_y
				if self.y_out_range == True:
					self.yservo_pid.SetStepSignal(450)
					self.y_out_range = False
				else:
					self.yservo_pid.SetStepSignal(240)

				self.yservo_pid.SetInertiaTime(0.01, 0.1)
               
				target_valuey = int(1500 + self.yservo_pid.SystemOutput)
				if target_valuey<=1000:
					target_valuey = 1000
					self.y_out_range = True
				self.target_servoy = int((target_valuey - 500) / 10) - 55#int((target_valuey - 500) / 10) - 55
				if self.target_servoy > 180: self.target_servoy = 180 #if self.target_servoy > 390: self.target_servoy = 390
				if self.target_servoy < 0: self.target_servoy = 0 
                #print("self.target_servoy = ",self.target_servoy)
				joint2 = 120 + self.target_servoy
				joint3 =  self.target_servoy / 4.5
				joint4 =  self.target_servoy / 3
                

        
		joints_0 = [int(self.target_servox), int(joint2), int(joint3), int(joint4), 90, 30]
		print(joints_0)
		self.cur_joints = joints_0
		self.pubSixArm(joints_0)
		

	def pubCurrentJoints(self):
		cur_joints = CurJoints()
		cur_joints.joints = self.cur_joints
		self.pub_cur_joints.publish(cur_joints) 
    

	def img_out(self,frame):
		frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
		cv2.imshow("result_image", frame)
		#key = cv2.waitKey(1)


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
		print("target_dist: ",math.sqrt(pose_T[1] ** 2 + pose_T[0]** 2))
		res_dist = math.sqrt(pose_T[1] ** 2 + pose_T[0]** 2)
		print("pose_R: ",pose_R)
		print("pose_T: ",pose_T)
		return res_dist

	def get_end_point_mat(self):
        #print("Get the current pose is ",self.CurEndPos)
		self.get_current_end_pos()
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
			
			print("self.CurEndPose: ",self.CurEndPos)
		except Exception as e:
			self.get_logger().error(f'Service call failed: {e}')

	def timer_second(self):
		time.sleep(1)
		self.adjust_flag = False
		self.compute_flag = True
		self.Depth_Track_done = True

def main():
	print('----------------------')
	rclpy.init()
	apriltag_track = AprilTagTrackNode('ApriltagTrack_node')
	rclpy.spin(apriltag_track)

           