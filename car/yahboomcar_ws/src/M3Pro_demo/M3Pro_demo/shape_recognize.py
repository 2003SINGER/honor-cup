import cv2
import os
import numpy as np
from cv_bridge import CvBridge
import cv2 as cv
from M3Pro_demo.Robot_Move import *
from arm_interface.srv import ArmKinemarics
from arm_interface.msg import AprilTagInfo,CurJoints
from arm_msgs.msg import ArmJoints
from std_msgs.msg import Float32,Bool,Int16
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
from M3Pro_demo.compute_joint5 import *
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
class ShapeRecognizeNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.init_joints = [90, 100, 0, 0, 90, 0]
		self.rgb_bridge = CvBridge()
		self.depth_bridge = CvBridge()
		self.pub_pos_flag = False
		self.CurEndPos = [0.1279009179959246, 0.00023254956548456117, 0.1484898062979958, 0.00036263794618046863, 1.3962632350758744, 0.0003332603981328959]
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
		self.TargetJoint5_pub = self.create_publisher(Int16, "set_joint5", 10)
		self.pubSixArm(self.init_joints)
		self.get_current_end_pos()
		self.pubCurrentJoints()
		self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 1, 0.5)
		self.ts.registerCallback(self.callback)
		self.Target_Shape = "Cylinder" #Rectangle  Square  Cylinder
		self.x_offset = offset_config.get('x_offset')
		self.y_offset = offset_config.get('y_offset')
		self.z_offset = offset_config.get('z_offset')
		self.adjust_dist = True
		self.linearx_PID = (0.5, 0.0, 0.2)
		self.linearx_pid = simplePID(self.linearx_PID[0] / 1000.0, self.linearx_PID[1] / 1000.0, self.linearx_PID[2] / 1000.0)
		self.done_flag = True
		self.joint5 = Int16()
		self.corners = np.empty((4, 2), dtype=np.int32)
		self.valid_dist = True
		print("Init done.")
       

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
			#self.pub_pos_flag = True
			self.done_flag = True
			self.pub_pos_flag = True
			self.Target_Shape = input("Please enter a target shape, which can be selected from Rectangle  Square  Cylinder：" )
			
	def pubCurrentJoints(self):
		cur_joints = CurJoints()
		cur_joints.joints = self.init_joints
		self.pub_cur_joints.publish(cur_joints)            

	def move_dist(self,dist):
		linear_x = self.linearx_pid.compute(dist, 200)
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
    

            
	def pubVel(self,vx,vy,vz):
		vel = Twist()
		vel.linear.x = float(vx)
		vel.linear.y = float(vy)
		vel.angular.z = float(vz)
		self.CmdVel_pub.publish(vel)
            
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
		gray_image = cv2.cvtColor(depth_to_color_image, cv2.COLOR_BGR2GRAY)
		black_image = np.zeros_like(gray_image)
		black_image[0:400, 0:640] = gray_image[0:400, 0:640]
		black_image[black_image < 60] = 0
		cv2.circle(black_image, (320,240), 1, (255,255,255), 1)
		gauss_image = cv2.GaussianBlur(black_image, (3, 3), 1)
		_,threshold_img = cv2.threshold(gauss_image, 0, 255, cv2.THRESH_BINARY)
		erode_img = cv2.erode(threshold_img, np.ones((5, 5), np.uint8))
		dilate_img = cv2.dilate(erode_img, np.ones((5, 5), np.uint8))
		contours, hierarchy = cv2.findContours(dilate_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		for obj in contours:
			area = cv2.contourArea(obj)
			if area < 2000 :
				continue
			cv2.drawContours(depth_to_color_image, obj, -1, (255, 255, 0), 4) 
			perimeter = cv2.arcLength(obj, True)
			approx = cv2.approxPolyDP(obj, 0.035 * perimeter, True)
			self.corners = approx
			cv2.drawContours(depth_to_color_image, approx, -1, (255, 0, 0), 4) 
			CornerNum = len(approx)
			#print(CornerNum)
			x, y, w, h = cv2.boundingRect(approx)
			if CornerNum == 3: objType = "triangle"
			elif CornerNum == 4:
				side_lengths = []
				for i in range(4):
					p1 = approx[i]
					p2 = approx[(i + 1) % 4]
					side_lengths.append(np.linalg.norm(p1 - p2))  # 计算两点间的距离
            
				side_lengths = np.array(side_lengths)
				#print("side_lengths[1]: ",side_lengths[1])
				#print("side_lengths[0]: ",side_lengths[0])
				if np.allclose(side_lengths[1], side_lengths[0], atol=50):  # 允许一些小的误差
					objType = "Square"
				else:
					objType = "Rectangle"
			elif CornerNum > 5:
				objType = "Cylinder"
			else:
				objType = "None"
			rect = cv2.minAreaRect(obj)
			center = rect[0]
			key = cv2.waitKey(1)
			if key == 32:
				self.pub_pos_flag = True
				self.adjust_dist = True
			if self.pub_pos_flag == True:
				if objType == self.Target_Shape and self.done_flag == True:
					cx = int(center[0])
					cy = int(center[1])
					dist = depth_image_info[int(cy),int(cx)]/1000
					pose = self.compute_heigh(cx,cy,dist)
					dist_detect = math.sqrt(pose[1] ** 2 + pose[0]** 2)
					dist_detect = dist_detect*1000
					if dist_detect<130:
						self.valid_dist = False
						print("Invalid dist.Plese restart the program.")
					dist = 'dist: ' + str(dist_detect) + 'mm'
					cv.putText(rgb_image, dist, (int(cx)+5, int(cy)+15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
					if abs(dist_detect - 200.0)>10 and self.adjust_dist==True and self.valid_dist == True:
						self.move_dist(dist_detect)
					elif abs(dist_detect - 200.0)<10 and self.valid_dist == True:
						self.pubVel(0,0,0)
						self.adjust_dist = False
						cx = int(center[0])
						cy = int(center[1])
						dist = depth_image_info[int(cy),int(cx)]/1000 
						print("dist: ",dist)
						if dist!=0:
							vx = self.corners[0][0][0] - self.corners[1][0][0]
							vy = self.corners[0][0][1] - self.corners[1][0][1]
							target_joint5 = compute_joint5(vx,vy)
							self.joint5.data = int(target_joint5)
							pos = AprilTagInfo()
							pos.x = float(cx)
							pos.y = float(cy)
							pos.z = float(dist)
							if self.pub_pos_flag == True:
								self.pub_pos_flag = False
								self.done_flag = False
								self.pos_info_pub.publish(pos)
								self.TargetJoint5_pub.publish(self.joint5)
				cv2.circle(rgb_image, (int(center[0]),int(center[1])), 5, (0,255,255), 5)
			cv2.putText(rgb_image, objType, (x + w // 2, y + (h //2)), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 0), 1) 

		cv2.imshow("depth_to_color_image", depth_to_color_image)
		cv2.imshow("rgb_image", rgb_image)
		key = cv2.waitKey(10)



		   
def main():
	print('----------------------')
	rclpy.init()
	shape_recognize = ShapeRecognizeNode('ShapeRecognize_node')
	shape_recognize.Target_Shape = input("Please enter a target shape, which can be selected from Rectangle  Square  Cylinder：" )
	rclpy.spin(shape_recognize)

           
