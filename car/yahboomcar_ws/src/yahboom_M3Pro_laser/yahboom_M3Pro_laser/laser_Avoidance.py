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

class laserAvoid(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a sub
        self.sub_laser = self.create_subscription(LaserScan,"/scan",self.registerScan,1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState', self.JoyStateCallback,1)
        #create a pub
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',1)
             
        #declareparam
        self.declare_parameter("linear",0.3)
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.declare_parameter("angular",1.0)
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.declare_parameter("LaserAngle",30.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("ResponseDist",0.55)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.Joy_active = False
        self.conut = 10

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        ranges = np.array(scan_data.ranges)
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if abs(angle) < self.LaserAngle and ranges[i] !=0.0:
                if ranges[i] <= self.ResponseDist*1.5:
                    self.front_warning += 1
            if angle < (90 - self.LaserAngle) < 90 and angle>0 and ranges[i] !=0.0:
                if ranges[i] <= self.ResponseDist*1.5:
                    self.Left_warning += 1
            if -90 < -(90 - self.LaserAngle) < angle and angle<0 and ranges[i] !=0.0:
                if ranges[i] <= self.ResponseDist*1.5:
                    self.Right_warning += 1
        if self.Joy_active:
            self.pub_vel.publish(Twist())
            return
        twist = Twist()
        if self.front_warning > self.conut and self.Left_warning > self.conut and self.Right_warning > self.conut:
            print ('1, there are obstacles in the left and right, turn right')
            twist.linear.x = self.linear
            twist.angular.z = -self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
        
        elif self.front_warning > self.conut and self.Left_warning <= self.conut and self.Right_warning > self.conut:
            print ('2, there is an obstacle in the middle right, turn left')
            twist.linear.x = 0.0
            twist.angular.z = self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
            if self.Left_warning > self.conut and self.Right_warning <= self.conut:
                twist.linear.x = 0.0
                twist.angular.z = -self.angular
                self.pub_vel.publish(twist)
                sleep(0.5)
        elif self.front_warning > self.conut and self.Left_warning > self.conut and self.Right_warning <= self.conut:
            print ('4. There is an obstacle in the middle left, turn right')
            twist.linear.x = 0.0
            twist.angular.z = -self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
            if self.Left_warning <= self.conut and self.Right_warning > self.conut:
                twist.linear.x = 0.0
                twist.angular.z = self.angular
                self.pub_vel.publish(twist)
                sleep(0.5)
        elif self.front_warning > self.conut and self.Left_warning < self.conut and self.Right_warning < self.conut:

            print ('6, there is an obstacle in the middle, turn left')
            twist.linear.x = 0.0
            twist.angular.z = self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
        elif self.front_warning < self.conut and self.Left_warning > self.conut and self.Right_warning > self.conut:
            print ('7. There are obstacles on the left and right, turn right')
            twist.linear.x = 0.0
            twist.angular.z = -self.angular
            self.pub_vel.publish(twist)
            sleep(0.4)
        elif self.front_warning < self.conut and self.Left_warning > self.conut and self.Right_warning <= self.conut:
            print ('8, there is an obstacle on the left, turn right')
            twist.linear.x = 0.0
            twist.angular.z = -self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
        elif self.front_warning < self.conut and self.Left_warning <= self.conut and self.Right_warning > self.conut:
            print ('9, there is an obstacle on the right, turn left')
            twist.linear.x = 0.0
            twist.angular.z = self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
        elif self.front_warning <= self.conut and self.Left_warning <= self.conut and self.Right_warning <= self.conut:
            print ('10, no obstacles, go forward')
            twist.linear.x = self.linear
            twist.angular.z = 0.0
            self.pub_vel.publish(twist)

    def exit_pro(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)


def main():
    rclpy.init()
    laser_avoid = laserAvoid("laser_Avoidance_a1")
    print ("start it")
    try:
        rclpy.spin(laser_avoid)
    except KeyboardInterrupt:
        pass
    finally:
        laser_avoid.exit_pro()
        laser_avoid.destroy_node()
        rclpy.shutdown()
