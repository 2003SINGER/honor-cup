import cv2
import os
import numpy as np
import message_filters
from M3Pro_demo.vutils import draw_tags
from dt_apriltags import Detector
from cv_bridge import CvBridge
import cv2 as cv
from arm_interface.srv import ArmKinemarics
from arm_msgs.msg import ArmJoints
import time
import transforms3d.euler as t3d_euler
import math
from rclpy.node import Node
import rclpy
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
import threading
import yaml
import transforms3d as tfs
import tf_transformations as tf

encoding = ['16UC1', '32FC1']
offset_file = "/home/jetson/yahboomcar_ws/src/arm_kin/param/offset_value.yaml"
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
		self.cur_joints = self.init_joints
		self.rgb_bridge = CvBridge()
		self.depth_bridge = CvBridge()
		self.cur_distance = 0.0
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
		self.depth_image_sub = Subscriber(self, Image, '/camera/depth/image_raw')
		self.pub_SixTargetAngle = self.create_publisher(ArmJoints, "arm6_joints", 10)
		self.client = self.create_client(ArmKinemarics, 'get_kinemarics')
		while not self.pub_SixTargetAngle.get_subscription_count():
			self.pubSixArm(self.init_joints)
			time.sleep(0.1)	
		self.pubSixArm(self.init_joints)
		self.get_current_end_pos()
		self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 100, 0.1)
		self.ts.registerCallback(self.callback)
		self.adjust_flag = False
		self.compute_flag = False
		self.start_time = 0.0
        #存储机械臂末端夹爪的位置xyz
		self.last_z = 0.34
		self.last_y = 0.0
		self.last_x = 0.12
        #z轴（高度）变化的方向1为往上，-1为往下
		self.dir_z = 1
        #xyz三个方向上移动的时间
		self.start_time_z = time.time()
		self.start_time_y = time.time()
		self.start_time_x = time.time()
		self.first_track = True #第一次追踪的标志位,用于给self.pitch进行赋值
    
        
            
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
        #按下空格键开始追踪程序
		if key == 32:
			self.compute_flag = True
		if len(tags) > 0 :
			center_x, center_y = tags[0].center
			corners = tags[0].corners
			cur_depth = depth_image_info[int(center_y),int(center_x)]
			if cur_depth>0:
				res_pos = self.compute_heigh(int(center_x),int(center_y),cur_depth/1000)
				print("heigh: ",res_pos[2])
				if self.compute_flag==True:
                    #判断初次追踪时机器码的高度是高于/低于当前机械臂夹爪末端的高度(0.34)并且根据判断结果赋值self.pitch的初值
					if res_pos[2] - 0.34 > 0.03 and self.first_track == True:
						self.pitch = 0.25
						self.first_track = False
    					
					elif res_pos[2] - 0.34 < 0.03 and self.first_track == True:
						self.pitch = -0.25
						self.first_track = False

                        
					elif res_pos[2] - 0.34 == 0.03 and self.first_track == True:
						self.pitch = 0.0 
						self.first_track = False
				
        
				#print("self.pitch: ",self.pitch)
                #判断机器码的高度是否有改变0.03表示上/下移动了3cm
				if abs(res_pos[2] - self.last_z)>0.03  and self.compute_flag==True:
					#等待时间1秒以确保数据稳定
					if  (time.time() - self.start_time_z)>1:

                        #通过比对上一次的高度来判断机器码的高度是低了还是高了,也就是判断是往上移动还是往下移动,根据判断结果,修改self,dir_z的值
						if (res_pos[2] - self.last_z)<0:
							self.dir_z = -1
						else:
							self.dir_z = 1
                            #满足在1秒内判断移动距离大于3cm,则修改self.adjust_flag的值为True说明机械臂需要调节姿态
						self.adjust_flag = True
                                            
				else:
                    #移动距离大于3cm但是时间不足1秒,则修改起始为当前时间,方便下次时间计算
					self.start_time_z = time.time()

				if abs(res_pos[1] - self.last_y)>0.03 and self.compute_flag == True:
					if  (time.time() - self.start_time_y)>1:
						self.adjust_flag = True

				else:
					self.start_time_y = time.time()
                    
				if abs(res_pos[0] - self.last_x)>0.03 and self.compute_flag == True:
					if  (time.time() - self.start_time_x)>1:
						self.adjust_flag = True
				else:
					self.start_time_x = time.time()

                #如果self.adjust_flag和self.compute_flag的值都为真,表示机械臂需要进行调节姿态进行追踪
				if self.adjust_flag == True and self.compute_flag == True:
                    #改变两个值,防止多次进入该判断
					self.adjust_flag = False
					self.compute_flag = False
                    #计算末端夹爪移动的偏移量
					of_x = res_pos[0]
					of_y = res_pos[1] * 0.3 #为了保证相机能够识别到物体，使得识别的物体在画面中间
					of_z = res_pos[2] 
                    #开启线程执行线程传入的参数为偏移量
					adjust_move = threading.Thread(target=self.adjust, args=(of_x,of_y,of_z,))
					adjust_move.start()
					adjust_move.join()  
                    #更新最后一次的机器码的xyz的值以便于下次移动距离的判断
					self.last_z = res_pos[2]
					self.last_y = res_pos[1]
					self.last_x = res_pos[0]

                    

	def adjust(self,offset_x,offset_y,offset_z):
		request = ArmKinemarics.Request()
        #这里我们机械臂末端夹爪的目标的x是与机器码的x与0.17的差,也就是机械臂夹爪末端的x与机器码要保持17cm的距离,以便能够获取到相机的深度数据
		request.tar_x = offset_x - 0.17
        #判断机械臂末端的x值是否小于0.10如果是则赋值为0.10
		if request.tar_x<0.10:
			request.tar_x = 0.10
        #机械臂末端的y值是取传入的offset_y，也就是机器码木块的y值得0.3倍，这里取0.3倍是为了保证机器码在画面中间  
		request.tar_y = offset_y
        #机械臂末端的z值是取传入的offset_z，也就是机器码木块的z值，表示机器码木块此时的高度                 
		request.tar_z = offset_z 
        #判断机械臂末端的z值是否小于0.25也就是25cm,如果是，则赋值为0.25
		if request.tar_z<0.25:
			request.tar_z = 0.25
		request.kin_name = "ik"
		request.roll = 0.0 
        #根据self.dir_z的值判断机器码的高度改变是往上还是往下俯仰角则需要相对于的进行改变,这里的0.2是比例系数,根据实际情况进行调试
		if self.dir_z<0:
			self.pitch = self.pitch - offset_z*0.2*self.dir_z    
		else:
			self.pitch = self.pitch + offset_z*0.2*self.dir_z   
        #判断计算出来的self.pitch的值是否大于0.174弧度如果是则赋值为0.174,这里是为了防止俯仰角过低
		if self.pitch>0.174:
			self.pitch = 0.174
		request.pitch  =  self.pitch
		print("self.pitch: ",self.pitch)
        #通过offset_y左右偏移量计算yaw的值
		request.yaw = offset_y/0.5
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
            #开启一个线程执行获取目标姿态的末端信息,用于后边计算机器码木块的位置信息
			get_end = threading.Thread(target=self.get_current_end_pos, args=())
			get_end.start()
			get_end.join()
			self.pubSixArm(joints)		
            #开启线程计算1秒等待机械臂移动完成
			waitting_ = threading.Thread(target=self.timer_second, args=())
			waitting_.start()
			waitting_.join()

		except Exception as e:
			self.get_logger().error(f'Service call failed: {e}')
                        
	def pubCurrentJoints(self):
		cur_joints = CurJoints()
		cur_joints.joints = self.cur_joints
		self.pub_cur_joints.publish(cur_joints) 
    

	def img_out(self,frame):
		frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
		cv2.imshow("result_image", frame)

	def compute_heigh(self,x,y,z):
		camera_location = self.pixel_to_camera_depth((x,y),z)
		PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
		EndPointMat = self.get_end_point_mat()
		WorldPose = np.matmul(EndPointMat, PoseEndMat) 
		pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)
		pose_T[0] = pose_T[0] + self.x_offset
		pose_T[1] = pose_T[1] - self.y_offset
		pose_T[2] = pose_T[2] + self.z_offset
		return pose_T

	def get_end_point_mat(self):
		self.get_current_end_pos()
		end_w,end_x,end_y,end_z = self.euler_to_quaternion(self.CurEndPos[3],self.CurEndPos[4],self.CurEndPos[5])
		endpoint_mat = self.xyz_quat_to_mat([self.CurEndPos[0],self.CurEndPos[1],self.CurEndPos[2]],[end_w,end_x,end_y,end_z])
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
			self.CurEndPos[0] = response.x 
			self.CurEndPos[1] = response.y
			self.CurEndPos[2] = response.z 
			self.CurEndPos[3] = response.roll
			self.CurEndPos[4] = response.pitch
			self.CurEndPos[5] = response.yaw
			
		except Exception as e:
			self.get_logger().error(f'Service call failed: {e}')

	def timer_second(self):
		time.sleep(1)
        #改变self.compute_flag,开启下一次的位置计算以及执行调整函数的条件
		self.compute_flag = True
        #改变计算移动等待时间的值为当前的时间戳，以便于下一次位置时的计算
		self.start_time_z = time.time()
		self.start_time_y = time.time()
		self.start_time_x = time.time()

def main():
	print('----------------------')
	rclpy.init()
	apriltag_track = AprilTagTrackNode('ApriltagTrack_node')
	rclpy.spin(apriltag_track)

           
