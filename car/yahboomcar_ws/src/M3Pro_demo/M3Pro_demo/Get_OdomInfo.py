import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import copysign, sqrt, pow
#import time
from rclpy.duration import Duration
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import PyKDL
from math import pi
import os
import sys
from std_msgs.msg import Float32,Bool
from yahboomcar_bringup.transform_utils import *
from dofbot_pro_interface.msg import TargetXYRoll

class GetOdom(Node):
    def __init__(self,name):
        super().__init__(name)
        self.sub_target_pos = self.create_subscription(Odometry,"/odom",self.get_odom_info,100)
        self.start_x = 0.0

    def get_odom_info(self,msg):
        print("position.x",msg.pose.pose.position.x)
        #print("offset in x",msg.pose.pose.position.x - self.start_x)
        #self.start_x = msg.pose.pose.position.x
        #print("position.x",msg.pose.pose.position.x)
        #print("position.x",msg.pose.pose.position.x)

def main():
    rclpy.init()
    odom_data = GetOdom("get_odom_node")
    rclpy.spin_once(odom_data)

    
