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
import os
print ("improt done")
RAD2DEG = 180 / math.pi

class laserTracker(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a sub
        self.sub_laser = self.create_subscription(LaserScan,"/scan",self.registerScan,1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState', self.JoyStateCallback,1)
        #create a pub
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',1)
        
        #declareparam
        self.declare_parameter("linear",0.5)
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.declare_parameter("angular",1.0)
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.declare_parameter("LaserAngle",45.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("ResponseDist",0.55)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        
        self.Joy_active = False
        self.lin_pid = SinglePID(1.0, 0.0, 1.0)
        self.ang_pid = SinglePID(2.0, 0.0, 2.0)
        

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        ranges = np.array(scan_data.ranges)
        minDistList = []
        minDistIDList = []
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if abs(angle) < self.LaserAngle and ranges[i] !=0.0 : 
				
            	#print("angle: ",angle)
            	minDistList.append(ranges[i])
            	minDistIDList.append(angle)
        
        if len(minDistList) != 0:
        	minDist = min(minDistList)
        	minDistID = minDistIDList[minDistList.index(minDist)]
        else:
        	#self.pub_vel.publish(Twist())
        	print("-----------------------")
        	return

        if self.Joy_active :
        	self.pub_vel.publish(Twist())
        	return
        velocity = Twist()
        print("minDist: ",minDist)
        print("minDistID: ",minDistID)
        if abs(minDist - self.ResponseDist) < 0.1: minDist = self.ResponseDist
        velocity.linear.x = -self.lin_pid.pid_compute(self.ResponseDist, minDist)
        ang_pid_compute = self.ang_pid.pid_compute(abs(minDistID)  / 72, 0)
        if 0< minDistID : 
            velocity.angular.z = ang_pid_compute
        elif  minDistID <0:
            velocity.angular.z = -ang_pid_compute
        if abs(ang_pid_compute) < 0.5: velocity.angular.z = 0.0
        velocity.angular.z = velocity.angular.z *0.6
        print("angular.z: ",velocity.angular.z)
        self.pub_vel.publish(velocity)

    def exit_pro(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)

def main():
    rclpy.init()
    laser_tracker = laserTracker("laser_Tracker_a1")
    print ("start it")
    try:
        rclpy.spin(laser_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        laser_tracker.exit_pro()
        laser_tracker.destroy_node()
        rclpy.shutdown()
