#!/usr/bin/env python3
# encoding: utf-8

from Rosmaster_Lib import Rosmaster
import numpy as np
from math import pi
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from dofbot_pro_interface.msg import *

class Arm_Ctrl(Node):
	def __init__(self, name):
		super().__init__(name)
		self.Arm = Rosmaster("/dev/ttyUSB0")
		self.joints = [90, 145, 0, 45, 90, 30]
		self.cur_joints = [90.0, 90.0, 90.0, 00.0, 90.0,30] 
		self.sub_TargetAngle =self.create_subscription(ArmJoint,"TargetAngle",self.Armcallback,1)
	def Armcallback(self,msg):
		if not isinstance(msg, ArmJoint): 
			print("----------")
			return
		arm_joint = ArmJoint()
		print("msg.joints: ",msg.joints)
		print("msg.joints: ",msg.run_time)
		if len(msg.joints) != 0:
			for i in range(2):
				print("--------------------------")
				msg.joints[0] = 180.0 - msg.joints[0]
				self.Arm.set_uart_servo_angle_array(msg.joints, msg.run_time)
		else:
			for i in range(2):
				print("msg.id: ",msg.id)
				self.Arm.set_uart_servo_angle(msg.id, msg.angle, msg.run_time)
def main():
	rclpy.init()
	arm_ctrl = Arm_Ctrl('arm_driver_node')
	rclpy.spin(arm_ctrl)
