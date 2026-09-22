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
from std_msgs.msg import Float32,Bool
encoding = ['16UC1', '32FC1']
import time
import transforms3d.euler as t3d_euler
import math

from rclpy.node import Node
import rclpy
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

print('init done')
class ShapeRecognizeNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.init_joints = [90, 100, 0, 0, 90, 0]
		self.init_joints_float = [90.0, 150.0, 12.0, 20.0, 90.0, 30.0]
		self.rgb_bridge = CvBridge()
		self.depth_bridge = CvBridge()
		self.pub_pos_flag = True
		self.pr_time = time.time()
		self.cnt = 0
		self.cur_distance = 0.0
		self.track_flag = True
		self.prev_dist = 0
		self.prev_angular = 0
		self.prev_roll = 0
		self.minDist = 400
		self.grasp_Dist = 250
		self.xy_track_flag = True
		self.linearx_PID = (0.5, 0.0, 0.2)
		self.lineary_PID = (0.2, 0.0, 0.1)
		self.angz_PID = (0.5, 0.0, 0.5)
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
		self.pub_beep = self.create_publisher(Bool, "beep", 10)
		self.PID_init()
		self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
		self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 1, 0.5)
		self.ts.registerCallback(self.callback)

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
		self.Target_Shape = "Square"
       
	def Beep_Loop(self):
		beep = Bool()
		beep.data = True
		self.pub_beep.publish(beep)
		time.sleep(1.0)
		beep.data = False
		self.pub_beep.publish(beep)

	def PID_init(self):
		self.linearx_pid = simplePID(self.linearx_PID[0] / 1000.0, self.linearx_PID[1] / 1000.0, self.linearx_PID[2] / 1000.0)
		self.lineary_pid = simplePID(self.lineary_PID[0] / 100.0, self.lineary_PID[1] / 100.0, self.lineary_PID[2] / 100.0)
		self.angz_pid = simplePID(self.angz_PID[0] / 100.0, self.angz_PID[1] / 100.0, self.angz_PID[2] / 100.0)

	def camera_info_callback(self, msg):
        # 获取相机内参矩阵和畸变系数
		self.camera_matrix = np.array(msg.k).reshape(3, 3)
		self.dist_coeffs = np.array(msg.d)

	def get_graspStatusCallBack(self,msg):
		if msg.data == True:
			self.pub_pos_flag = True
			
            
            
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
		black_image[black_image < 90] = 0
		cv2.circle(black_image, (320,240), 1, (255,255,255), 1)
		gauss_image = cv2.GaussianBlur(black_image, (3, 3), 1)
		_,threshold_img = cv2.threshold(gauss_image, 0, 255, cv2.THRESH_BINARY)
		erode_img = cv2.erode(threshold_img, np.ones((5, 5), np.uint8))
		dilate_img = cv2.dilate(erode_img, np.ones((5, 5), np.uint8))
		contours, hierarchy = cv2.findContours(dilate_img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
		for obj in contours:
			area = cv2.contourArea(obj)
			if area < 2000 :
				continue
			cv2.drawContours(depth_to_color_image, obj, -1, (255, 255, 0), 4) 
			perimeter = cv2.arcLength(obj, True)
			approx = cv2.approxPolyDP(obj, 0.035 * perimeter, True)
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
					#print("p1[0]: ",p1[0])
					#print("p1[0][0]: ",p1[0][0])
					#print("p1[0][1]: ",p1[0][1])
					#cv2.circle(rgb_image, (approx[0][0][0],approx[0][0][1]), 5, (0,0,255), 5)
					#cv2.circle(rgb_image, (approx[1][0][0],approx[1][0][1]), 5, (0,255,0), 5)
					#cv2.circle(rgb_image, (approx[2][0][0],approx[2][0][1]), 5, (255,255,0), 5)
					#cv2.circle(rgb_image, (approx[3][0][0],approx[3][0][1]), 5, (255,0,0), 5)
            
				side_lengths = np.array(side_lengths)
				#print("side_lengths[1]: ",side_lengths[1])
				#print("side_lengths[0]: ",side_lengths[0])
				if np.allclose(side_lengths[1], side_lengths[0], atol=50):  # 允许一些小的误差
					objType = "Square"
					rect = cv2.minAreaRect(obj)
					center = rect[0]
					x =  int(center[0])  
					y =  int(center[1])  
					depth = depth_image_info[y,x]/1000
					#print("depth: ",depth)
                    
					result  = self.compute_dist.get_dist(x,y,depth)
                    
					if result!=[]:
						result[2] = result[2] * 100
						Square_Volume = result[2] * result[2] *result[2]
						print("the volume of Square is ",Square_Volume) #单位是立方厘米
					#print("len: ",result[2])
				else:
					objType = "Rectangle"


					rect = cv2.minAreaRect(obj)
					center = rect[0]
					cx =  int(center[0])  
					cy =  int(center[1])  
					depth = depth_image_info[cy,cx]/1000
					center = self.compute_dist.get_dist(cx,cy,depth)
					print("center: ",center)
					print("-----------------------------")
					corner_x =  int(approx[0][0][0]) 
					corner_y =  int(approx[0][0][1]) 
					if corner_x>cx:
						corner_x = corner_x -1
					else:
						corner_x = corner_x + 1
                        
					if corner_y>cy:
						corner_y = corner_y -1
					else:
						corner_y = corner_y + 1
					corner_depth = depth_image_info[corner_y,corner_x]/1000
					cv2.circle(rgb_image, (corner_x,corner_y), 5, (0,125,255), 5)
					#print("corner_depth: ",corner_depth)
					corner =  self.compute_dist.get_dist(corner_x,corner_y,corner_depth)
					print("corner: ",corner)
					x1 = center[0] *100
					x2 = corner[0] *100
					y1 = center[1] *100
					y2 = corner[1] *100
					height = center[2] *100
					print("height: ",height)
					diagonal = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)*2
					print("diagonal: ",diagonal)
					length = math.sqrt(diagonal**2 - height**2)
					print("length: ",length)
					Rectangle_Volume = length * height * height 
					print("the volume of Rectangle is ",Rectangle_Volume)#单位是立方厘米
			elif CornerNum > 5:
				objType = "cylinder"
				cv2.circle(rgb_image, (approx[0][0][0],approx[0][0][1]), 5, (0,0,255), 5)
				rect = cv2.minAreaRect(obj)
				center = rect[0]
				cx =  int(center[0])
				cy =  int(center[1])
				depth_c = depth_image_info[cy,cx]/1000
				center_result  = self.compute_dist.get_dist(x,y,depth_c)
				height = center_result[2]*100
				print("height: ",height)

				px =  approx[0][0][0] + 1
				py =  approx[0][0][1] + 1
				depth_p = depth_image_info[py,px]/1000
				#print("depth_p: ",depth_p)
				point_result  = self.compute_dist.get_dist(px,py,depth_p)
                
				if center_result!=[] and point_result!=[]:
					print("center_result: ",center_result)
					print("point_result: ",point_result)
					x1 = center_result[0] *100
					x2 = point_result[0] *100
					y1 = center_result[1] *100
					y2 = point_result[1] *100
					radius = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
					print("radius: ",radius)
					Cylinder_Volume = height * radius * radius * math.pi
					print("the volume of Cylinder is ",Cylinder_Volume) #单位是立方厘米
					print("---------------------------------------------------")
			else:
				objType = "None"
			rect = cv2.minAreaRect(obj)
			center = rect[0]

			cv2.circle(rgb_image, (int(center[0]),int(center[1])), 5, (0,255,255), 5)
			cv2.putText(rgb_image, objType, (x + w // 2, y + (h //2)), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 0), 1)   

		cv2.imshow("depth_to_color_image", depth_to_color_image)
		#cv2.imshow("gray_image", gray_image)
		#cv2.imshow("gauss_image", gauss_image)
		#cv2.imshow("threshold_img", threshold_img)
		#cv2.imshow("black_image", black_image)
		#cv2.imshow("dilate_img", dilate_img)
		cv2.imshow("rgb_image", rgb_image)
		key = cv2.waitKey(1)

    


		   
def main():
	print('----------------------')
	rclpy.init()
	shape_recognize = ShapeRecognizeNode('ShapeRecognize_node')
	rclpy.spin(shape_recognize)

           