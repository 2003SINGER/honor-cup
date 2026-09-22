import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
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

class CalibrateLinear(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a spublisher
        self.cmd_vel = self.create_publisher(Twist,"cmd_vel",5)
        self.start_test = False
        self.declare_parameter('test_distance',1.0)
        self.test_distance_x = 0.0
        self.test_distance_y = 0.0 
        self.test_roll = 0.0 
        
        self.declare_parameter('speed',0.1)
        self.speed = self.get_parameter('speed').get_parameter_value().double_value

        
        self.declare_parameter('tolerance',0.008)
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        
        self.declare_parameter('odom_linear_scale_correction',2.5)
        self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').get_parameter_value().double_value

        
    
        self.declare_parameter('start_test',True)
        
        
        self.declare_parameter('direction',True)
        self.direction = self.get_parameter('direction').get_parameter_value().bool_value
        
        self.declare_parameter('base_frame','base_footprint')
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
        self.declare_parameter('odom_frame','odom')
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value


        self.sub_target_pos = self.create_subscription(TargetXYRoll,"xy_roll",self.get_target_posCallBack,100)
        self.pub_move_done = self.create_publisher(Bool,"move_done",5)
        
        #init the tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rclpy.time.Time(), timeout = Duration(seconds = 60.0))
        #time.sleep(2)
        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y
        self.odom_angle = 0.0
        self.first_angle = 0.0
        self.test_angle = 0.0
        self.roll_tolerance = 0.03
        self.turn_angle = 0.0
        self.move_xy = False
        self.rotate_roll = False
        self.error_ang = 0.0
        self.odom_angular_scale_correction = 2.5
        self.rotate_speed = 0.5
        
        
        
        print ("finish init work")
        now = rclpy.time.Time()
        #trans = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now,timeout=Duration(seconds = 10.0))               
        #create timer 
        self.timer = self.create_timer(0.05, self.on_timer)

    def get_target_posCallBack(self,msg):
        self.test_distance_x = msg.x
        self.test_distance_y = msg.y
        self.test_angle = abs(msg.roll)
        self.test_roll = msg.roll
        self.start_test = True
        print("self.test_roll: ",self.test_roll)

        
    def on_timer(self):
        move_cmd = Twist()
        if self.start_test:

            self.position.x = self.get_position().transform.translation.x
            self.position.y = self.get_position().transform.translation.y
            #roll
            self.error_ang = abs(self.test_angle) - self.turn_angle
            distance = sqrt(pow((self.position.x - self.x_start), 2) +
                        pow((self.position.y - self.y_start), 2))
            #print("distance: ",distance)
            #print("distance_y: ",distance_y)
            #print("self.test_distance_x: ",self.test_distance_x)
            
            self.test_distance = sqrt(pow((self.test_distance_x - 0), 2) +
                        pow((self.test_distance_y - 0), 2))
            #print("self.test_distance: ",self.test_distance)
            error_lin =  self.test_distance  - distance
            #error_y =  self.test_distance_y - distance_y 
            #print("error_lin: ",error_lin)
            #print("self.error_angn: ",self.error_ang)
            #print("error_y: ",error_y)
            #start = time()
            if not self.start_test:

                self.start_test  = False
                #all_new_parameters = [self.start_test]
                #self.set_parameters(all_new_parameters)
                self.cmd_vel.publish(Twist())    
                print("done")
            if abs(error_lin) < self.tolerance:
                move_cmd.linear.x = 0.0
                move_cmd.linear.y = 0.0
                self.move_xy = True
                print("translation done.")
                
            elif  abs(error_lin) > self.tolerance and self.move_xy == False:
                #if abs(error_x) > self.tolerance:
                #move_cmd.linear.x = copysign(self.speed,1 * self.test_distance_x) 
                move_cmd.linear.x = self.speed
                #if abs(error_y) > self.tolerance:
                move_cmd.linear.y = copysign(self.speed,1 * self.test_distance_y)
                #print("move_cmd.linear.x: ",move_cmd.linear.x)
                #print("move_cmd.linear.y: ",move_cmd.linear.y)
                    #print("y")
            '''if  abs(self.error_ang) < self.tolerance:
                move_cmd.angular.z = 0.0
                self.rotate_roll = True
                print("rotation done")
                
            elif abs(self.error_ang) > self.roll_tolerance and self.rotate_roll == False:
                move_cmd.angular.z = copysign(self.rotate_speed, self.test_angle)
                #move_cmd.angular.z = -self.rotate_speed
                print("move_cmd.angular.z: ",move_cmd.angular.z)
                self.odom_angle = self.get_odom_angle() 
                self.delta_angle = self.odom_angular_scale_correction * self.normalize_angle(self.odom_angle - self.first_angle)
                self.turn_angle += self.delta_angle
                self.error_ang = self.test_angle - self.turn_angle
                self.first_angle = self.odom_angle'''
            self.cmd_vel.publish(move_cmd)
            #end = time()
        else:
            #print("Please change the state!")
            self.cmd_vel.publish(Twist())
        '''if  self.rotate_roll == True and self.move_xy == True :
        
            self.start_test = False
            done_flag = Bool()
            done_flag.data = True
            self.pub_move_done.publish(done_flag)
            self.rotate_roll = False
            self.move_xy = False'''
                    
            
        #print("self.x_start: ",self.x_start)
        #self.cmd_vel.publish(Twist() )    
        
    def get_param(self):
        #self.start_test = self.get_parameter('start_test').get_parameter_value().bool_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.test_distance = self.get_parameter('test_distance').get_parameter_value().double_value
        self.direction = self.get_parameter('direction').get_parameter_value().bool_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
          
     
    def get_position(self):
         try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now)
            #print("trans: ",trans)   
            return trans       
         except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            raise
            return

    def get_odom_angle(self):
         try:
            now = rclpy.time.Time()
            rot = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now)   
            #print("oring_rot: ",rot.transform.rotation) 
            cacl_rot = PyKDL.Rotation.Quaternion(rot.transform.rotation.x, rot.transform.rotation.y, rot.transform.rotation.z, rot.transform.rotation.w)
            #print("cacl_rot: ",cacl_rot)
            angle_rot = cacl_rot.GetRPY()[2]
         except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            return
        
         return angle_rot
             
            
    def exit_pro(self):
        print("------------------------------------------------------")
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)

    def normalize_angle(self,angle):
        res = angle
        #print("res: ",res)
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res
         
def main():
    rclpy.init()
    class_calibratelinear = CalibrateLinear("calibrate_linear")
    try:
        rclpy.spin(class_calibratelinear)
    except KeyboardInterrupt:
        pass
    finally:
        class_calibratelinear.exit_pro()
        class_calibratelinear.destroy_node()
        rclpy.shutdown()
    
