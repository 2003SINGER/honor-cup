#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#commom lib
import math
import numpy as np
import time
from time import sleep
from yahboom_M3Pro_laser.common import *
print ("improt done")
RAD2DEG = 180 / math.pi

class laserTracker(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a sub
        self.sub_laser = self.create_subscription(LaserScan,"/scan1",self.registerScan,1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState', self.JoyStateCallback,1)
        #create a pub
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',1)
        
        
        
        
        #declareparam
        self.declare_parameter("linear",0.5)
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.declare_parameter("angular",1.0)
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.declare_parameter("LaserAngle",40.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("ResponseDist",0.35)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter("Switch",False)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.Joy_active = False
        self.ros_ctrl = SinglePID()
        self.priorityAngle = 30  # 40
        self.Moving = False
        self.lin_pid = SinglePID(1.0, 0.0, 1.0)
        self.ang_pid = SinglePID(2.0, 0.0, 2.0)
        
        self.timer = self.create_timer(0.01,self.on_timer)
        
    def on_timer(self):
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        ranges = np.array(scan_data.ranges)
        offset = 0.0
        frontDistList = []
        frontDistIDList = []
        minDistList = []
        minDistIDList = []

        
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if (0<abs(angle)<90 or 270<abs(angle)<360) and ranges[i] !=0.0:
                frontDistList.append(ranges[i])
                frontDistIDList.append(angle)
        
        if len(frontDistIDList) != 0:
        	minDist = min(frontDistList)
        	minDistID = frontDistIDList[frontDistList.index(minDist)]
        else:
        	#self.pub_vel.publish(Twist())
        	print("-----------------------")
        	return

        if self.Joy_active or self.Switch == True:
        	if self.Moving == True:
        		self.pub_vel.publish(Twist())
        		self.Moving = not self.Moving
        	return
        self.Moving = True
        velocity = Twist()
        print("minDist: ",minDist)
        print("minDistID: ",minDistID)
        if abs(minDist - self.ResponseDist) < 0.1: minDist = self.ResponseDist
        velocity.linear.x = -self.lin_pid.pid_compute(self.ResponseDist, minDist)
        if 0< abs(minDistID) <90:
        	ang_pid_compute = self.ang_pid.pid_compute(( 0 - abs(minDistID) ) / 96, 0)
        	if abs(ang_pid_compute) < 0.3:
        		velocity.angular.z = 0.0
        	else:
        		velocity.angular.z = -ang_pid_compute
        elif 270< abs(minDistID) < 360:
        	ang_pid_compute = self.ang_pid.pid_compute((abs(minDistID) - 360) / 96, 0)
        	if abs(ang_pid_compute) < (360 / abs(minDistID))*0.3: 
        		velocity.angular.z = 0.0
        	else:
        		velocity.angular.z = ang_pid_compute
        print("orin_angular.z: ",velocity.angular.z)
        print("ang_pid_compute: ",ang_pid_compute)
       
        velocity.angular.z = -velocity.angular.z *0.2
        print("linear.x: ",velocity.linear.x)
        print("angular.z: ",velocity.angular.z)
        self.pub_vel.publish(velocity)

def main():
    rclpy.init()
    laser_tracker = laserTracker("laser_Tracker_a1")
    print ("start it")
    try:
        rclpy.spin(laser_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        laser_tracker.pub_vel.publish(Twist())
        laser_tracker.destroy_node()
        rclpy.shutdown()
