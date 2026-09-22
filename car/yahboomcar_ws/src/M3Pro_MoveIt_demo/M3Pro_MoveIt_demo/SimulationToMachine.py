import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from math import pi
import numpy as np
from arm_msgs.msg import ArmJoints

class SimulateToMachine(Node):
	def __init__(self, name):
		super().__init__(name)
		self.sub_state =self.create_subscription(JointTrajectoryControllerState,"/arm_group_controller/state",self.get_ArmPosCallback,1)
		self.pub_SixTargetAngle = self.create_publisher(ArmJoints, "arm6_joints", 10)
		self.joints = [90.0, 90.0, 90.0, 90.0, 90.0, 30.0]

	def get_ArmPosCallback(self,msg):
		#print("Get the position of arm : ",msg.actual.positions)
		arm_rad = np.array(msg.actual.positions)
		DEG2RAD = np.array([180 / pi])
		arm_deg = np.dot(arm_rad.reshape(-1, 1), DEG2RAD)
		#print("arm_deg: ",arm_deg)
		if len(msg.actual.positions) == 5:
			mid = np.array([90, 90, 90, 90, 90])
            #mid = np.array([0, 0, 0, 0, 0])
			arm_array = np.array(np.array(arm_deg) + mid)
			for i in range(5): self.joints[i] = arm_array[i]
		elif len(msg.actual.positions) == 1:
            # arm_deg: -88~0 ;arm_array: 91~180
			arm_array = np.array(np.array(arm_deg) + np.array([180]))
			self.joints[5] = np.interp(arm_array, [90, 180], [30, 180])[0]
		print("self.joints: ",self.joints)
		self.pubSixArm(self.joints)

	def pubSixArm(self, joints, id=6, angle=180.0, runtime=500):
		arm_joints =ArmJoints()
		arm_joints.joint1 = 180 - int(joints[0])
		arm_joints.joint2 = int(joints[1])
		arm_joints.joint3 = int(joints[2])
		arm_joints.joint4 = int(joints[3])
		arm_joints.joint5 = int(joints[4])
		arm_joints.joint6 = int(joints[5])
		arm_joints.time = runtime
		self.pub_SixTargetAngle.publish(arm_joints)

def main():
	rclpy.init()
	simulate2machine = SimulateToMachine('simulate_to_machine_node')
	rclpy.spin(simulate2machine)
