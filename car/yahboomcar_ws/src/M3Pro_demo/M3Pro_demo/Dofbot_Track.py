#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import numpy as np
import math
import transforms3d as tfs
import tf_transformations as tf
import yaml
from X5Plus_demo.PID import *
from dofbot_pro_interface.srv import *
from dofbot_pro_interface.msg import AprilTagInfo
from arm_msgs.msg import ArmJoints
from std_msgs.msg import Float32,Bool
from rclpy.node import Node
import rclpy
offset_file = "/home/jetson/yahboomcar_ws/src/dofbot_pro_info/param/offset_value.yaml"
with open(offset_file, 'r') as file:
    offset_config = yaml.safe_load(file)
print(offset_config)
print("----------------------------")
print("x_offset: ",offset_config.get('x_offset'))
print("y_offset: ",offset_config.get('y_offset'))
print("z_offset: ",offset_config.get('z_offset'))
class DofbotTrack(Node):
    def __init__(self, name):
        super().__init__(name)
        self.xservo_pid = PositionalPID(0.25, 0.1, 0.05)
        self.yservo_pid = PositionalPID(0.25, 0.1, 0.05)
        self.target_servox=90
        self.target_servoy=180
        self.a = 0
        self.b = 0
        self.c = 0
        self.CurEndPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.down_joint = [130.0, 55.0, 34.0, 16.0, 90.0,118.0]
        self.cx = 640.0
        self.cy = 480.0
        self.px = 0.0
        self.py = 0.0
        self.init_joints = [90.0, 150.0, 12.0, 20.0, 90.0, 30.0]
        self.y_out_range = False
        self.x_out_range = False
        self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[ 0 ,0 ,1 ,-9.90000000e-02],
									 [-1  ,0 ,0  ,0],
									 [0  ,-1  ,0 ,4.90000000e-02],
									 [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])
        self.cur_joints = self.init_joints
        self.move_flag = True
        self.last_y = 0.28
        self.move_done = True
        self.compute_x = 0.0
        self.compute_y = 0.0
        self.set_joint5 = 90.0

        #self.pub_TargetAngle = self.create_publisher(ArmJoint,"TargetAngle",1)
        self.pub_SixTargetAngle = self.create_publisher(ArmJoints, "arm6_joints", 10)
        self.pub_Buzzer = self.create_publisher(Bool,"Buzzer",1)
        self.sub_move = self.create_subscription(Bool,"move",self.moveCallBack,100)
        self.client = self.create_client(DofbotProKinemarics, 'get_kinemarics')
        self.get_current_end_pos(self.init_joints)
        self.x_offset = offset_config.get('x_offset')
        self.y_offset = offset_config.get('y_offset')
        self.z_offset = offset_config.get('z_offset')

    def moveCallBack(self,msg):
        if msg.data == True:
            self.move_flag = True

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
                
                self.target_servox = int((target_valuex - 500) / 10) -10
                #print("self.target_servox:",self.target_servox)
                
                if self.target_servox > 180:
                    self.x_out_range = True
                    
                if self.target_servox < 0:
                    self.x_out_range = True
                 
        #180 240 0 240            
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
                joint2 = 120 + int(self.target_servoy)
                joint3 =  int(self.target_servoy / 4.5)
                joint4 =  int(self.target_servoy / 3.0)
                

        
        joints_0 = [self.target_servox, joint2, joint3, joint4, 90, 30]
        print(joints_0)
        self.cur_joints = joints_0 
        self.pubSixArm(joints_0)
         
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


    def Clamping(self,cx,cy,cz):

        print("cx: ",cx)
        print("cy: ",cy)
        print("cz: ",cz)
        print("self.cur_joints: ",self.cur_joints)
        self.get_current_end_pos(self.cur_joints)
        camera_location = self.pixel_to_camera_depth((cx,cy),cz)
        PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
        EndPointMat = self.get_end_point_mat()
        WorldPose = np.matmul(EndPointMat, PoseEndMat) 
        pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)
        pose_T[0] = pose_T[0] + self.x_offset*0
        pose_T[1] = pose_T[1] + self.y_offset*0
        pose_T[2] = pose_T[2] + self.z_offset*0
        print("pose_T: ",pose_T)


        request = DofbotProKinemarics.Request()
        request.tar_x = pose_T[0]
        request.tar_y = pose_T[1]
        request.tar_z = pose_T[2]  + (math.sqrt(request.tar_y**2+request.tar_x**2)-0.181)*0.2*0
        request.kin_name = "ik"
        request.roll = self.CurEndPos[3] 
        request.pitch  = self.CurEndPos[4]
        request.yaw = self.CurEndPos[5] 
        future = self.client.call_async(request)
        future.add_done_callback(self.get_ik_respone_callback)
        #print("calcutelate_request: ",request)
        


    def get_ik_respone_callback(self, future):
        print("****************************")
        try:
            
            response = future.result()
            joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
            joints[0] = response.joint1 #response.joint1
            joints[1] = response.joint2
            joints[2] = response.joint3
            if response.joint4>90:
                joints[3] = 90.0
            else:
                joints[3] = response.joint4
            joints[4] = 90.0
            joints[5] = 30.0
            print("compute_joints: ",joints)
            self.pubArm(joints)
            self.move()
            
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            
    def pubArm(self, joints, id=6, angle=180.0, runtime=2000):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        if len(joints) != 0: arm_joint.joints = joints
        else: arm_joint.joints = []
        #self.pub_TargetAngle.publish(arm_joint)
			
            
    def move(self):
        print("set_joint5: ",self.set_joint5)
        time.sleep(2.5)
        self.pubArm([],5, self.set_joint5, 2000)
        time.sleep(2.5)
        self.pubArm([],6, 125.0, 2000)
        time.sleep(2.5)
        self.pubArm([],2, 120.0, 2000)
        time.sleep(2.5)
        self.pubArm(self.down_joint)
        time.sleep(2.5)
        self.pubArm([],6, 90.0, 2000)
        time.sleep(2.5)
        self.pubArm([],2, 90.0, 2000)
        time.sleep(2.5)
        self.pubArm(self.init_joints)
          
    def Buzzer(self):
        beep = Bool()
        beep.data = True
        self.pub_buzzer.publish(beep)
        time.sleep(1)
        beep.data = False
        self.pub_buzzer.publish(beep)
        time.sleep(1)

    def get_current_end_pos(self,input_joints):
        request = DofbotProKinemarics.Request()
        request.cur_joint1 = input_joints[0]
        request.cur_joint2 = input_joints[1]
        request.cur_joint3 = input_joints[2]
        request.cur_joint4 = input_joints[3]
        request.cur_joint5 = input_joints[4]
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
            #self.get_logger().info(f'Response received: {self.CurEndPos}')
            print("self.CurEndPose: ",self.CurEndPos)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


        
        
        

        
        
