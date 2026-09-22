#!/usr/bin/env python
# encoding: utf-8

# public lib
import os
import time
import getpass
import threading
from time import sleep

# ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Int32, Bool, UInt16, ColorRGBA

from arm_msgs.msg import ArmJoint
from arm_msgs.msg import ArmJoints


class JoyTeleop(Node):
    def __init__(self, name):
        super().__init__(name)
        self.Joy_active = False
        self.arm_activa= False
        self.Buzzer_active = 0
        self.RGBLight_index = 0
        self.cancel_time = time.time()
        self.user_name = getpass.getuser()
        self.linear_Gear = 1.0 / 3
        self.angular_Gear = 1.0 / 4

        self.loop_active = True
        self.gripper_active = True
        self.arm_joints = [90, 120, 10, 20, 90, 0]
        self.arm_joint = ArmJoint()

        # create pub
        self.pub_goal = self.create_publisher(GoalID, "move_base/cancel", 10)
        self.pub_cmdVel = self.create_publisher(Twist, "cmd_vel", 1)
        self.pub_Buzzer = self.create_publisher(UInt16, "/beep", 1)
        self.pub_JoyState = self.create_publisher(Bool, "JoyState", 10)
        self.pub_RGBLight = self.create_publisher(ColorRGBA, "rgb", 10)
        self.pub_SingleTargetAngle = self.create_publisher(ArmJoint, "arm_joint", 100)
        self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 100)

        # create sub
        self.sub_Joy = self.create_subscription(Joy, "joy", self.buttonCallback, 10)

        # declare parameter and get the value
        self.declare_parameter("xspeed_limit", 1.0)
        self.declare_parameter("yspeed_limit", 1.0)
        self.declare_parameter("angular_speed_limit", 5.0)
        self.xspeed_limit = (
            self.get_parameter("xspeed_limit").get_parameter_value().double_value
        )
        self.yspeed_limit = (
            self.get_parameter("yspeed_limit").get_parameter_value().double_value
        )
        self.angular_speed_limit = (
            self.get_parameter("angular_speed_limit").get_parameter_value().double_value
        )
      
        while not self.TargetAngle_pub.get_subscription_count():
            self.pubSix_Arm(self.arm_joints )
            time.sleep(1)	
        self.pubSix_Arm(self.arm_joints )

    def pubSix_Arm(self, joints, id=6, angle=180.0, runtime=2000):
        arm_joint =ArmJoints()
        arm_joint.joint1 = joints[0]
        arm_joint.joint2 = joints[1]
        arm_joint.joint3 = joints[2]
        arm_joint.joint4 = joints[3]
        arm_joint.joint5 = joints[4]
        arm_joint.joint6 = joints[5]
        arm_joint.time = runtime
        self.TargetAngle_pub.publish(arm_joint)

    def buttonCallback(self, joy_data):
        if not isinstance(joy_data, Joy):
            return
        if self.user_name == "jetson":
            self.user_jetson(joy_data)
        else:
            self.user_pc(joy_data)

    def pub_armjoint(self, id, direction):
        self.loop_active = True
        arm_thread = threading.Thread(target=self.arm_ctrl, args=(id, direction))
        arm_thread.setDaemon(True)
        arm_thread.start()

    def arm_ctrl(self, id, direction):
        while 1:
            if self.loop_active:
                self.arm_joints[id - 1] += direction
                if id == 5:
                    if self.arm_joints[id - 1] > 270:
                        self.arm_joints[id - 1] = 270
                    elif self.arm_joints[id - 1] < 0:
                        self.arm_joints[id - 1] = 0
                elif id == 6:
                    if self.arm_joints[id - 1] >= 180:
                        self.arm_joints[id - 1] = 180
                    elif self.arm_joints[id - 1] <= 30:
                        self.arm_joints[id - 1] = 30
                else:
                    if self.arm_joints[id - 1] > 180:
                        self.arm_joints[id - 1] = 180
                    elif self.arm_joints[id - 1] < 0:
                        self.arm_joints[id - 1] = 0
                self.arm_joint.id = id
                self.arm_joint.joint = int(self.arm_joints[id - 1])
                self.arm_joint.time = 1000
                if self.arm_activa:
                    print("arm control")
                    self.pub_SingleTargetAngle.publish(self.arm_joint)
                sleep(0.5)
            else:
                break
            

    def user_jetson(self, joy_data):
        # arm_ctrl_start
        if joy_data.buttons[10] == 1:
            self.gripper_active = not self.gripper_active
        if (
            joy_data.buttons[0]
            == joy_data.buttons[1]
            == joy_data.buttons[6]
            == joy_data.buttons[3]
            == joy_data.buttons[4]
            == 0
            and joy_data.axes[7] == joy_data.axes[6] == 0
            and joy_data.axes[5] != -1
        ):
            self.loop_active = False
        else:
            if joy_data.buttons[3] == 1:
                print("1,-")
                self.pub_armjoint(1, -1)  # X
            if joy_data.buttons[1] == 1:
                self.pub_armjoint(1, 1)  # B
                print("1,+")
            if joy_data.buttons[0] == 1:
                self.pub_armjoint(2, -1)  # A
                print("2,-")
            if joy_data.buttons[4] == 1:
                self.pub_armjoint(2, 1)  # Y
                print("2,+")
            if joy_data.axes[6] != 0:
                self.pub_armjoint(3, -joy_data.axes[6])  # 左按键左正右负
                print("3,-/+")
            if joy_data.axes[7] != 0:
                self.pub_armjoint(4, joy_data.axes[7])  # 左按键上正下负
                print("4,-/+")
            if self.gripper_active:
                if joy_data.axes[5] == -1:
                    self.pub_armjoint(6, -1)  # L2
                    print("6,-")
                if joy_data.buttons[6] == 1:
                    self.pub_armjoint(6, 1)  # L1
                    print("6,+")
            else:
                if joy_data.axes[5] == -1:
                    self.pub_armjoint(5, -1)  # L2
                    print("5,-")
                if joy_data.buttons[6] == 1:
                    self.pub_armjoint(5, 1)  # L1
                    print("5,+")
        # arm_ctrl_end

        # cancel nav
        if joy_data.buttons[9] == 1:
            # print("-------------------")
            self.cancel_nav()
        # RGBLight
        if joy_data.buttons[7] == 1:
            self.RGBLight_index = self.RGBLight_index + 1.0
            if self.RGBLight_index > 7:
                self.RGBLight_index = 0.0

            rgb__msg = ColorRGBA()
            rgb__msg.a = 100.0 + self.RGBLight_index
            self.pub_RGBLight.publish(rgb__msg)
        # Buzzer
        if joy_data.buttons[11] == 1:
            Buzzer_ctrl = UInt16()
            if self.Buzzer_active == 0:
                self.Buzzer_active = self.Buzzer_active + 1
            else:
                self.Buzzer_active = self.Buzzer_active - 1
            Buzzer_ctrl.data = self.Buzzer_active
            self.pub_Buzzer.publish(Buzzer_ctrl)
            # linear Gear control
        if joy_data.buttons[13] == 1:
            if self.linear_Gear == 1.0:
                self.linear_Gear = 1.0 / 3
            elif self.linear_Gear == 1.0 / 3:
                self.linear_Gear = 2.0 / 3
            elif self.linear_Gear == 2.0 / 3:
                self.linear_Gear = 1
            # angular Gear control
        if joy_data.buttons[14] == 1:
            if self.angular_Gear == 1.0:
                self.angular_Gear = 1.0 / 4
            elif self.angular_Gear == 1.0 / 4:
                self.angular_Gear = 1.0 / 2
            elif self.angular_Gear == 1.0 / 2:
                self.angular_Gear = 3.0 / 4
            elif self.angular_Gear == 3.0 / 4:
                self.angular_Gear = 1.0
        xlinear_speed = (
            self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
        )

        ylinear_speed = (
            self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
        )
        angular_speed = (
            self.filter_data(joy_data.axes[2])
            * self.angular_speed_limit
            * self.angular_Gear
        )
        if xlinear_speed > self.xspeed_limit:
            xlinear_speed = self.xspeed_limit
        elif xlinear_speed < -self.xspeed_limit:
            xlinear_speed = -self.xspeed_limit
        if ylinear_speed > self.yspeed_limit:
            ylinear_speed = self.yspeed_limit
        elif ylinear_speed < -self.yspeed_limit:
            ylinear_speed = -self.yspeed_limit
        if angular_speed > self.angular_speed_limit:
            angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit:
            angular_speed = -self.angular_speed_limit
        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.linear.y = ylinear_speed
        twist.angular.z = angular_speed
        if self.Joy_active == True:
            #print("joy control now")
            self.pub_cmdVel.publish(twist)

    def user_pc(self, joy_data):
        # arm_ctrl_start
        if joy_data.buttons[10] == 1:
            self.gripper_active = not self.gripper_active
        # if (
        #     joy_data.buttons[0]
        #     == joy_data.buttons[1]
        #     == joy_data.buttons[6]
        #     == joy_data.buttons[3]
        #     == joy_data.buttons[4]
        #     == 0
        #     and joy_data.axes[7] == joy_data.axes[6] == 0
        #     and joy_data.axes[5] != -1
        # ):
        #     self.loop_active = False
        else:
            if joy_data.buttons[3] == 1:
                self.pub_armjoint(1, -1)  # X
            if joy_data.buttons[1] == 1:
                self.pub_armjoint(1, 1)  # B
            if joy_data.buttons[0] == 1:
                self.pub_armjoint(2, -1)  # A
            if joy_data.buttons[4] == 1:
                self.pub_armjoint(2, 1)  # Y
            if joy_data.axes[6] != 0:
                self.pub_armjoint(3, -joy_data.axes[6])  # 左按键左正右负
            if joy_data.axes[7] != 0:
                self.pub_armjoint(4, joy_data.axes[7])  # 左按键上正下负
            if self.gripper_active:
                if joy_data.axes[5] == -1:
                    self.pub_armjoint(6, -1)  # L2
                if joy_data.buttons[6] == 1:
                    self.pub_armjoint(6, 1)  # L1
            else:
                if joy_data.axes[5] == -1:
                    self.pub_armjoint(5, -1)  # L2
                if joy_data.buttons[6] == 1:
                    self.pub_armjoint(5, 1)  # L1
        # arm_ctrl_end
        # 取消 Cancel
        if joy_data.axes[5] == -1:
            self.cancel_nav()
        if joy_data.buttons[5] == 1:
            if self.RGBLight_index < 6:
                self.pub_RGBLight.publish(self.RGBLight_index)
                # print ("pub RGBLight success")
            else:
                self.RGBLight_index = 0
            self.RGBLight_index += 1
        if joy_data.buttons[7] == 1:
            return
            # 档位控制 Gear control
        if joy_data.buttons[9] == 1:
            if self.linear_Gear == 1.0:
                self.linear_Gear = 1.0 / 3
            elif self.linear_Gear == 1.0 / 3:
                self.linear_Gear = 2.0 / 3
            elif self.linear_Gear == 2.0 / 3:
                self.linear_Gear = 1
        if joy_data.buttons[10] == 1:
            if self.angular_Gear == 1.0:
                self.angular_Gear = 1.0 / 4
            elif self.angular_Gear == 1.0 / 4:
                self.angular_Gear = 1.0 / 2
            elif self.angular_Gear == 1.0 / 2:
                self.angular_Gear = 3.0 / 4
            elif self.angular_Gear == 3.0 / 4:
                self.angular_Gear = 1.0
        xlinear_speed = (
            self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
        )
        ylinear_speed = (
            self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
        )
        angular_speed = (
            self.filter_data(joy_data.axes[2])
            * self.angular_speed_limit
            * self.angular_Gear
        )
        if xlinear_speed > self.xspeed_limit:
            xlinear_speed = self.xspeed_limit
        elif xlinear_speed < -self.xspeed_limit:
            xlinear_speed = -self.xspeed_limit
        if ylinear_speed > self.yspeed_limit:
            ylinear_speed = self.yspeed_limit
        elif ylinear_speed < -self.yspeed_limit:
            ylinear_speed = -self.yspeed_limit
        if angular_speed > self.angular_speed_limit:
            angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit:
            angular_speed = -self.angular_speed_limit
        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.linear.y = ylinear_speed
        twist.angular.z = angular_speed
        for i in range(3):
            self.pub_cmdVel.publish(twist)

    def filter_data(self, value):
        if abs(value) < 0.2:
            value = 0
        return value

    def cancel_nav(self):
        now_time = time.time()
        if now_time - self.cancel_time > 1:
            Joy_ctrl = Bool()
            self.Joy_active = not self.Joy_active
            self.arm_activa = not self.arm_activa
            Joy_ctrl.data = self.Joy_active
            for i in range(3):
                self.pub_JoyState.publish(Joy_ctrl)
                self.pub_cmdVel.publish(Twist())
            self.cancel_time = now_time


def main():
    rclpy.init()
    joy_ctrl = JoyTeleop("joy_ctrl")
    rclpy.spin(joy_ctrl)
