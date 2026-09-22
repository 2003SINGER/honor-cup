import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from M3Pro_demo.Robot_Move import *
from M3Pro_demo.color_common import *
from arm_interface.srv import ArmKinemarics
from arm_interface.msg import AprilTagInfo,CurJoints
from arm_msgs.msg import ArmJoints
from std_msgs.msg import Bool,Int16
import time
import transforms3d as tfs
import tf_transformations as tf
import yaml
import math
from rclpy.node import Node
import rclpy
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
import threading
from M3Pro_demo.compute_joint5 import *

encoding = ['16UC1', '32FC1']
package_pwd =  "/home/jetson/yahboomcar_ws/src/M3Pro_demo/M3Pro_demo/"

offset_file = "/home/jetson/yahboomcar_ws/src/arm_kin/param/offset_value.yaml"
with open(offset_file, 'r') as file:
    offset_config = yaml.safe_load(file)
print(offset_config)
print("----------------------------")
print("x_offset: ",offset_config.get('x_offset'))
print("y_offset: ",offset_config.get('y_offset'))
print("z_offset: ",offset_config.get('z_offset'))

print('init done')
class ColorRecognizeNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.init_joints = [90, 110, 0, 0, 90, 0]
		self.rgb_bridge = CvBridge()
		self.depth_bridge = CvBridge()
		self.pub_pos_flag = False
		self.linearx_PID = (0.5, 0.0, 0.2)
		self.CurEndPos = [0.13850614623645266, 0.000231128779644737, 0.16629191805489946, 0.00018255962454541956, 1.221730317129278, 0.00014750319860679475]
		self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
		self.EndToCamMat = np.array([[ 0 ,0 ,1 ,-1.00e-01],
									 [-1  ,0 ,0  ,0],
									 [0  ,-1  ,0 ,4.82000000e-02],
									 [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])        
		self.rgb_image_sub = Subscriber(self, Image, '/camera/color/image_raw')
		self.sub_grasp_status = self.create_subscription(Bool,"grasp_done",self.get_graspStatusCallBack,100)
		self.depth_image_sub = Subscriber(self, Image, '/camera/depth/image_raw')
		self.CmdVel_pub = self.create_publisher(Twist,"cmd_vel",1)
		self.pub_cur_joints = self.create_publisher(CurJoints,"Curjoints",1)
		self.pos_info_pub = self.create_publisher(AprilTagInfo,"PosInfo",1)
		self.pub_SixTargetAngle = self.create_publisher(ArmJoints, "arm6_joints", 10)
		self.client = self.create_client(ArmKinemarics, 'get_kinemarics')
		while not self.client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available, waiting again...')		
		self.get_current_end_pos()
		
		while not self.pub_SixTargetAngle.get_subscription_count():
			self.pubSixArm(self.init_joints)
			time.sleep(0.1)	
		self.pubSixArm(self.init_joints)

		while not self.pub_cur_joints.get_subscription_count():
			self.pubCurrentJoints()
			time.sleep(0.1)	
		self.pub_beep = self.create_publisher(Bool, "beep", 10)
		self.TargetJoint5_pub = self.create_publisher(Int16, "set_joint5", 10)
		self.pubCurrentJoints()
		self.get_current_end_pos()
		self.pubSixArm(self.init_joints)
		self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 1, 0.5)
		self.ts.registerCallback(self.callback)
		self.start_grasp = False
		self.x_offset = offset_config.get('x_offset')
		self.y_offset = offset_config.get('y_offset')
		self.z_offset = offset_config.get('z_offset')
		self.adjust_dist = False
		self.linearx_pid = simplePID(self.linearx_PID[0] / 1000.0, self.linearx_PID[1] / 1000.0, self.linearx_PID[2] / 1000.0)
		self.done_flag = True
		self.target_color = 0
		self.red_hsv_text = os.path.join(package_pwd, 'red_colorHSV.text')
		self.green_hsv_text = os.path.join(package_pwd, 'green_colorHSV.text')
		self.blue_hsv_text = os.path.join(package_pwd, 'blue_colorHSV.text')
		self.yellow_hsv_text = os.path.join(package_pwd, 'yellow_colorHSV.text')
		self.hsv_range = ()
		self.select_flags = False
		self.windows_name = 'frame'
		self.Track_state = 'init'
		self.Mouse_XY = (0, 0)
		self.cols, self.rows = 0, 0
		self.Roi_init = ()
		self.color = color_detect()
		self.cur_color = None
		self.text_color = (0,0,0)
		self.cx = 0
		self.cy = 0
		self.c_x = 0
		self.c_y = 0
		self.c_r = 0
		self.circle_r = 0
		self.valid_dist = True
		self.CX_list = []
		self.CY_list = []
		self.R_list = []
		self.detect_flag = False
		self.compute_height = True
		self.joint5 = Int16()
		self.corners = np.empty((4, 2), dtype=np.int32)
		self.cur_target_color = 0
		self.updata_flag = False
		print("Init done.")
              
	def Beep_Loop(self):
		beep = Bool()
		beep.data = True
		self.pub_beep.publish(beep)
		time.sleep(1.0)
		beep.data = False
		self.pub_beep.publish(beep)

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


	def camera_info_callback(self, msg):
        # 获取相机内参矩阵和畸变系数
		self.camera_matrix = np.array(msg.k).reshape(3, 3)
		self.dist_coeffs = np.array(msg.d)

	def get_graspStatusCallBack(self,msg):
		if msg.data == True:
			
			self.done_flag = True
			self.adjust_dist = True
			self.detect_flag = False
			self.compute_height = True
			self.valid_dist = True
			time.sleep(2.0)
			self.pub_pos_flag = True
			
            
			
	def pubCurrentJoints(self):
		cur_joints = CurJoints()
		cur_joints.joints = self.init_joints
		self.pub_cur_joints.publish(cur_joints)            

	def move_dist(self,dist):
		linear_x = self.linearx_pid.compute(dist, 220)
		#print("lin.x: ",linear_x)
		self.pubVel(linear_x,0,0)
    
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
    

	def robot_move(self,point_x,dist,roll):
		roll_degree = abs(math.degrees(roll))
		print("point_x: ",point_x)
		print("dist: ",dist)
		print("roll_degree: ",roll_degree)
		if abs(self.prev_dist - dist) > 30:
			self.prev_dist = dist
			return
		linear_x = self.linearx_pid.compute(dist, self.minDist)
		angular_z = -self.rotation_direction * 0.25
		linera_y = self.rotation_direction * 0.1
		if abs(dist - self.minDist) < 30: 
			linear_x = 0
			self.x_track_done = True
		if abs(roll_degree - 175) < 5: 
			angular_z = 0
			#linera_y = 0
			self.roll_track_done = True
			linera_y = self.lineary_pid.compute(320,point_x)*0.3
			if abs(point_x - 320) < 5:
				linera_y = 0.0
				self.y_track_done = True
		print("linear_x: ",linear_x)
		print("linear_y: ",linera_y)
		print("angular_z: ",angular_z)
		self.pubVel(linear_x,linera_y,angular_z)
		if self.x_track_done == True and self.roll_track_done == True and self.y_track_done == True:
			print("adjust dist to grasp it.")
			self.track_flag = False
			self.adjust_dist = True

            
	def pubVel(self,vx,vy,vz):
		vel = Twist()
		vel.linear.x = float(vx)
		vel.linear.y = float(vy)
		vel.angular.z = float(vz)
		self.CmdVel_pub.publish(vel)

	def Reset(self):
		self.hsv_range = ()
		self.circle = (0, 0, 0)
		self.Mouse_XY = (0, 0)
		self.Track_state = 'init'
		print("Change the state.")
		self.cx = 0
		self.cy = 0
		self.pubPos_flag = False
		#self.target_color = 0
		self.pubVel(0,0,0)

	def img_out(self,result_frame,binary):
		if len(binary) != 0: cv.imshow(self.windows_name, ManyImgs(0.8, ([result_frame, binary])))
		else:
			cv.imshow(self.windows_name, result_frame)

	def onMouse(self, event, x, y, flags, param):
		if event == 1:
			self.Track_state = 'init'
			self.select_flags = True
			self.Mouse_XY = (x, y)
		if event == 4:
			self.select_flags = False
			self.Track_state = 'mouse'
		if self.select_flags == True:
			self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
			self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
			self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])
            
	def callback(self,color_frame,depth_frame):
        # 将画面转为 opencv 格式
		rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
		rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
		result_image = np.copy(rgb_image)
		#depth_image
		depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
		frame = cv.resize(depth_image, (640, 480))
		depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1.0), cv2.COLORMAP_JET)
		depth_image_info = frame.astype(np.float32)  
		key = cv2.waitKey(10)& 0xFF
		result_frame, binary = self.process(rgb_image,key)
		show_frame = threading.Thread(target=self.img_out, args=(result_frame,binary,))
		show_frame.start()
		show_frame.join()
		if key == 32:
			self.adjust_dist = True
			self.pub_pos_flag = True
		if len(self.CX_list)>0 and self.done_flag==True:
			for i in range(len(self.CX_list)):
				if self.R_list_[i]>30:
					cx = int(self.CX_list[i])
					cy = int(self.CY_list[i])
					dist = depth_image_info[int(cy),int(cx)]/1000
					pose = self.compute_heigh(cx,cy,dist)
					compute_heigh = round(pose[2],2)*100
					heigh = 'heigh: ' + str(compute_heigh) + ' cm'
					cv.putText(result_image, heigh, (int(cx)+10, int(cy)-25), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
					if  compute_heigh > 4.0 and self.pub_pos_flag == True  and self.compute_height == True: 
						self.c_x = int(self.CX_list[i])
						self.c_y = int(self.CY_list[i])
						self.c_r = self.R_list_[i]
						self.detect_flag = True
						
					if self.detect_flag == True :
						cz = depth_image_info[int(self.c_y),int(self.c_x)]/1000
						pose = self.compute_heigh(self.c_x,self.c_y,cz)
						dist_detect = math.sqrt(pose[1] ** 2 + pose[0]** 2)
						dist_detect = dist_detect*1000
						dist = 'dist: ' + str(dist_detect) + ' mm'
						cv.putText(result_image, dist, (int(cx)+5, int(cy)+15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
						if abs(dist_detect - 220.0)>5 :
							if self.adjust_dist==True:
								self.move_dist(dist_detect)
						else:
							self.compute_height = False
							self.pubVel(0,0,0)
							self.adjust_dist = False
							cx = self.c_x
							cy = self.c_y
							dist = depth_image_info[int(cy),int(cx)]/1000 
							if dist!=0 and self.pub_pos_flag == True:
								vx = self.corners[0][0][0] - self.corners[1][0][0]
								vy = self.corners[0][0][1] - self.corners[1][0][1]
								target_joint5 = compute_joint5(vx,vy)
								self.joint5.data = int(target_joint5) 
								pos = AprilTagInfo()
								pos.id = self.target_color
								pos.x = float(cx)
								pos.y = float(cy)
								pos.z = float(dist)
								if self.pub_pos_flag == True:
									self.pub_pos_flag = False
									self.done_flag = False
									self.pos_info_pub.publish(pos)  
									self.TargetJoint5_pub.publish(self.joint5)
									print("Publish the position.")
			if self.detect_flag == False:
				self.pubVel(0,0,0)
				print("Do not find target hight.")
		cv2.imshow("result_image", result_image)

	def process(self,rgb_img,key):
		rgb_img = cv.resize(rgb_img, (640, 480))
		binary = []
		'''if key!=255:
			print("key: ",key)'''
		#print("self.target_color: ",self.target_color)
		#if action == 32: self.pubPos_flag = True
		
		if key == ord('c') or key == ord('C'):
			self.target_color = 0
			self.Reset()  
			self.updata_flag = True
		elif key == ord('r') or key == ord('R'): 
			self.target_color = 1
			self.cur_target_color = self.target_color
		elif key == ord('g') or key == ord('G'): 
			self.target_color = 2
			self.cur_target_color = self.target_color
		elif key == ord('b') or key == ord('B'): 
			self.target_color = 3
			self.cur_target_color = self.target_color
		elif key == ord('y') or key == ord('Y'): 
			self.target_color = 4
			self.cur_target_color = self.target_color
		elif key == ord('i') or key == ord('I') or self.target_color!=0: self.Track_state = "identify"
		#print("self.Track_state: ",self.Track_state)
		if self.Track_state == 'init':
			cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
			cv.setMouseCallback(self.windows_name, self.onMouse, 0)
			if self.select_flags == True:
				cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
				cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
				if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
					rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
					self.dyn_update = True
				else: self.Track_state = 'init'
                    
		elif self.Track_state == "identify":
			if self.target_color == 1:
				self.hsv_range = read_HSV(self.red_hsv_text)
				self.cur_color = "red"
				self.text_color = (0, 0, 255)

                
			elif self.target_color == 2:
				self.hsv_range = read_HSV(self.green_hsv_text)
				self.cur_color = "green"
				self.text_color = (0, 255, 0)

                
			elif self.target_color == 3:
				self.hsv_range = read_HSV(self.blue_hsv_text)
				self.cur_color = "blue"
				self.text_color = (255, 0, 0)
                
			elif self.target_color == 4:
				self.hsv_range = read_HSV(self.yellow_hsv_text)
				self.cur_color = "yellow"
				self.text_color = (255, 255, 0)
                
			else: 
				self.Track_state = 'init'

		if self.Track_state != 'init':
			if len(self.hsv_range) != 0:
				rgb_img, binary, self.CX_list,self.CY_list,self.R_list_,self.corners= self.color.object_follow_list(rgb_img, self.hsv_range)
				'''self.cx = self.circle[0]
				self.cy = self.circle[1]
				self.circle_r = self.circle[2]'''
				if self.cur_target_color == 1 and self.updata_flag == True:
					write_HSV(self.red_hsv_text, self.hsv_range)
				elif self.cur_target_color  == 2 and self.updata_flag == True:
					write_HSV(self.green_hsv_text, self.hsv_range)
				elif self.cur_target_color == 3 and self.updata_flag == True:
					write_HSV(self.blue_hsv_text, self.hsv_range)
				elif self.cur_target_color == 4 and self.updata_flag == True:
					write_HSV(self.yellow_hsv_text, self.hsv_range)
				self.updata_flag = False
                    
		rgb_img = cv2.putText(rgb_img, self.cur_color, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, self.text_color, 2)             
		return rgb_img, binary



		   
def main():
	print('----------------------')
	try:
		rclpy.init()
		color_recognize = ColorRecognizeNode('ColorRecognize_node')
		rclpy.spin(color_recognize)
	except Exception as e:  # 捕获所有其他异常（谨慎使用）
	    print(f"Unexpected error: {e}")
	    color_recognize.pubVel(0,0,0)

           