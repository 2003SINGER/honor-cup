#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt16
#commom lib
import math
import numpy as np
import time
from time import sleep
from yahboom_M3Pro_laser.common import *
import os
print ("improt done")
RAD2DEG = 180 / math.pi

class laserWarning(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a sub
        self.sub_laser = self.create_subscription(LaserScan,"/scan",self.registerScan,1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState', self.JoyStateCallback,1)
        #create a pub
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',1)
        self.pub_Buzzer = self.create_publisher(UInt16,'/beep',1)

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
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)

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
            #if (abs(angle) < self.LaserAngle or abs(angle) > 360 - self.LaserAngle) and ranges[i]!=0 : 
            if abs(angle) < self.LaserAngle and ranges[i] !=0.0 : 
            	#print("angle: ",angle)
            	minDistList.append(ranges[i])
            	minDistIDList.append(angle)
        if len(minDistList) == 0: return
        minDist = min(minDistList)
        minDistID = minDistIDList[minDistList.index(minDist)]
        print("minDistID: ",minDistID)
        #print("minDist: ",minDist)
        if self.Joy_active :
        	self.pub_vel.publish(Twist())
        	return

        print("minDist: ",minDist)
        if minDist <= self.ResponseDist and minDist!=0.0:
        	b = UInt16()
        	b.data = 1
        	self.pub_Buzzer.publish(b)
        else:
        	self.pub_Buzzer.publish(UInt16())
        velocity = Twist()
        print("minDistID: ",minDistID)	
        ang_pid_compute = self.ang_pid.pid_compute(abs(minDistID)  / 72, 0)
        if 0<minDistID :
        	velocity.angular.z = ang_pid_compute
        elif minDistID <0:
        	velocity.angular.z = -ang_pid_compute
        print("orin_angular.z: ",velocity.angular.z)
        if abs(ang_pid_compute) < 0.5: velocity.angular.z = 0.0
        velocity.angular.z = velocity.angular.z *0.5
        print("angular.z: ",velocity.angular.z)
        self.pub_vel.publish(velocity)

    def exit_pro(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)

def main():
    rclpy.init()
    laser_warn = laserWarning("laser_Warnning_a1")
    print ("start it")
    try:
        rclpy.spin(laser_warn)
    except KeyboardInterrupt:
        pass
    finally:
        laser_warn.exit_pro()
        laser_warn.destroy_node()
        rclpy.shutdown()
