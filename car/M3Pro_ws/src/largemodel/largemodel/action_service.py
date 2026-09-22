import cv2
import re
import rclpy
import subprocess
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import time
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int16MultiArray, Bool
from arm_msgs.msg import ArmJoints, ArmJoint
from interfaces.action import Rot
import math
import pygame
from arm_interface.msg import CurJoints
import yaml
from concurrent.futures import Future
import psutil
from ament_index_python.packages import get_package_share_directory
import os
from threading import Thread
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from utils import large_model_interface
import threading
from rclpy.executors import MultiThreadedExecutor
import functools


class CustomActionServer(Node):
    def __init__(self):
        super().__init__("action_service_ndoe")
        # 初始化参数配置 / Initialize parameter configuration
        self.init_param_config()
        # 初始化ROS通信 / Initialize ROS communication
        self.init_ros_comunication()
        # 加载地图映射文件 / Load map mapping file
        self.load_target_points()
        # 初始化机械臂抓取功能 / Initialize arm grasping function
        self.arm_grasp_init()
        # 初始化语音合成功能 / Initialize text-to-speech synthesis function
        self.system_sound_init()
        # 初始化语言设置/Initialize language settings
        self.init_language()
        self.get_logger().info("action service started...")

    def init_param_config(self):
        """
        初始化参数配置 / Initialize parameter configuration
        """
        # 设置夹取启动文件路径 / Set the path for the grasping startup file
        pkg_share = get_package_share_directory("largemodel")
        self.map_mapping_config = os.path.join(pkg_share, "config", "map_mapping.yaml")
        # 声明参数 / Declare parameters
        self.declare_parameter("Speed_topic", "/cmd_vel")
        self.declare_parameter("use_double_llm", False)
        self.declare_parameter("text_chat_mode", False)
        self.declare_parameter("useolinetts", False)
        self.declare_parameter("language", "zh")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("regional_setting", "China")
        # 获取参数值 / Get parameter values
        self.Speed_topic = (
            self.get_parameter("Speed_topic").get_parameter_value().string_value
        )
        self.use_double_llm = (
            self.get_parameter("use_double_llm").get_parameter_value().bool_value
        )
        self.text_chat_mode = (
            self.get_parameter("text_chat_mode").get_parameter_value().bool_value
        )
        self.useolinetts = (
            self.get_parameter("useolinetts").get_parameter_value().bool_value
        )
        self.language = (
            self.get_parameter("language").get_parameter_value().string_value
        )
        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.regional_setting = (
            self.get_parameter("regional_setting").get_parameter_value().string_value
        )
        self.pkg_path = get_package_share_directory("largemodel")
        self.image_save_path = os.path.join(
            self.pkg_path, "resources_file", "image.png"
        )
        self.current_pose = PoseWithCovarianceStamped()
        self.record_pose = PoseStamped()
        self.combination_mode = False  # 组合模式 / Combination mode
        self.interrupt_flag = False  # 打断标志 / Interrupt flag
        self.action_runing = False  # 动作执行状态 / Action execution status
        self.first_record = True  # 首次记录位置 / First record
        self.is_recording = False  # 录音状态 / Recording status
        self.IS_SAVING = False #是否正在保存图像
        self.joint6 = (
            140  # 默认机械臂六轴的初始角度 / Default angle of the six-axis arm
        )

        # 图像处理对象 / Image processing object
        self.image_msg = None
        self.bridge = CvBridge()
        # 创建模型接口客户端 / Create model interface client
        self.model_client = large_model_interface.model_interface()

    def init_ros_comunication(self):
        """
        初始化创建ros通信对象、函数 / Initialize creation of ROS communication objects and functions
        """
        # 创建速度话题发布者 / Create velocity topic publisher
        self.publisher = self.create_publisher(Twist, self.Speed_topic, 10)
        # 创建导航功能客户端，请求导航动作服务器 / Create navigation function client, request navigation action server
        self.navclient = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # 创建动作执行服务器，用于接受动作列表，并执行动作 / Create action execution server to accept action lists and execute actions
        self._action_server = ActionServer(
            self, Rot, "action_service", self.execute_callback
        )
        # 创建机械臂角度发布者，用于发布arm6_joints，控制机械臂 / Create arm angle publisher to publish arm6_joints and control the arm
        self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 100)
        # 创建关节角度发布者，用于发布arm_joint控制关节 / Create joint angle publisher to publish arm_joint and control joints
        self.SingleJoint_pub = self.create_publisher(ArmJoint, "arm_joint", 100)
        # 创建执行动作状态发布者 / Create action execution status publisher
        self.actionstatus_pub = self.create_publisher(String, "actionstatus", 3)
        # 创建发布者，发布 seewhat_handle 话题 / Create publisher to publish seewhat_handle topic
        self.seewhat_handle_pub = self.create_publisher(String, "seewhat_handle", 1)
        # 创建物体位置发布者，发布待夹取物体的坐标 / Create object position publisher to publish coordinates of objects to be grasped
        self.object_position_pub = self.create_publisher(
            Int16MultiArray, "corner_xy", 1
        )
        # 创建JoyCb话题发布者，启动KCF_Tracker_ALM节点测距的功能 / Create JoyCb topic publisher to enable distance measurement functionality of KCF_Tracker_ALM node
        self.joy_pub = self.create_publisher(Bool, "JoyState", 1)
        # 创建当前机械臂关节角发布者 / Create current arm joint angle publisher
        self.pub_cur_joints = self.create_publisher(CurJoints, "Curjoints", 1)
        # 创建KCF_Tracker_ALM重置发布者 / Create KCF_Tracker_ALM reset publisher
        self.reset_pub = self.create_publisher(Bool, "reset_flag", 1)
        # 创建机械臂抓取完成话题订阅者 / Create subscriber for arm grasping completion topic
        self.largemodel_arm_done_sub = self.create_subscription(
            String, "/largemodel_arm_done", self.largemodel_arm_done_callback, 1
        )
        # 创建发布者，发布 tts_topic 主题 / Create publisher to publish tts_topic topic
        self.TTS_publisher = self.create_publisher(String, "tts_topic", 5)
        # 创建tf监听者，监听坐标变换 / Create tf listener to monitor coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 创建打断状态发布者 / Create interrupt status publisher
        self.interrupt_flag_pub = self.create_publisher(Bool, "interrupt_flag", 1)
        # wakeup话题订阅者 / Subscribe to wakeup topic
        self.wakeup_sub = self.create_subscription(
            Bool, "wakeup", self.wakeup_callback, 5
        )
        # 图像话题订阅者 / Image topic subscriber
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 2
        )
        # 录音状态话题订阅者 / Record status topic subscriber
        self.record_status_sub = self.create_subscription(
            Bool, "record_status", self.record_status_callback, 5
        )

    def system_sound_init(
        self,
    ):  # 初始化系统声音相关的功能 / Initialize system sound-related functions
        pkg_path = get_package_share_directory("largemodel")

        if self.regional_setting == "China":  # 如果是中国地区 /if it is in China
            if self.useolinetts:
                model_type = "oline"
                self.tts_out_path = os.path.join(
                    pkg_path, "resources_file", "tts_output.mp3"
                )
            else:
                model_type = "local"
                self.tts_out_path = os.path.join(
                    pkg_path, "resources_file", "tts_output.wav"
                )

        elif (
            self.regional_setting == "international"
        ):  # 如果是国际地区 /if it is international

            model_type = "XUNFEI_FOR_INTERNATIONAL"
            self.tts_out_path = os.path.join(
                pkg_path, "resources_file", "XUNFEI_TTS.mp3"
            )
        else:
            while True:
                self.get_logger().info()(
                    'Please check the regional_setting parameter in yahboom.yaml file, it should be either "China" or "international".'
                )
                time.sleep(1)

        self.model_client.tts_model_init(
            model_type, self.language
        )  # 初始化语音合成模型 / Initialize TTS model
        # 初始化音频播放器 / Initialize audio player
        pygame.mixer.init()
        self.stop_event = threading.Event()

    def init_language(self):
        language_list = ["zh", "en"]
        if self.language not in language_list:
            while True:
                self.get_logger().info(
                    "The language setting is incorrect. Please check the action_service'' language setting in the yahboom.yaml file"
                )
                self.get_logger().info(self.language)
                time.sleep(1)

        self.feedback_largemoel_dict = {
            "zh": {  # 中文 / Chinese
                "navigation_1": "机器人反馈:导航目标{point_name}被拒绝",
                "navigation_2": "机器人反馈:执行navigation({point_name})完成",
                "navigation_3": "机器人反馈:执行navigation({point_name})失败，目标点不存在",
                "navigation_4": "机器人反馈:执行navigation({point_name})失败",
                "get_current_pose_success": "机器人反馈:get_current_pose()成功",
                "arm_up_done": "机器人反馈:执行arm_up()完成",
                "arm_down_done": "机器人反馈:执行arm_down()完成",
                "drift_done": "机器人反馈:执行drift()完成",
                "wait_done": "机器人反馈:执行wait({duration})完成",
                "arm_shake_done": "机器人反馈:执行arm_shake()完成",
                "arm_nod_done": "机器人反馈:执行arm_nod()完成",
                "arm_applaud_done": "机器人反馈:执行arm_applaud()完成",
                "grasp_obj_done": "机器人反馈:执行grasp_obj({x1},{y1},{x2},{y2})完成",
                "grasp_obj_failed": "机器人反馈:执行grasp_obj({x1},{y1},{x2},{y2})失败",
                "putdown_done": "机器人反馈:执行putdown()完成",
                "set_cmdvel_done": "机器人反馈:执行set_cmdvel({linear_x},{linear_y},{angular_z},{duration})完成",
                "move_left_done": "机器人反馈:执行move_left({angle},{angular_speed})完成",
                "move_right_done": "机器人反馈:执行move_right({angle},{angular_speed})完成",
                "turn_left_done": "机器人反馈:执行turn_left()完成",
                "turn_right_done": "机器人反馈:执行turn_right()完成",
                "dance_done": "机器人反馈:执行dance()完成",
                "apriltag_sort_done": "机器人反馈:执行apriltag_sort({target_id})完成",
                "apriltag_sort_failed": "机器人反馈:执行apriltag_sort({target_id})失败",
                "apriltag_follow_2D_done": "机器人反馈:执行apriltag_follow_2D({target_id})完成",
                "apriltag_follow_2D_failed": "机器人反馈:执行apriltag_follow_2D({target_id})失败",
                "apriltag_remove_higher_done": "机器人反馈:执行apriltag_remove_higher({target_high})完成",
                "apriltag_remove_higher_failed": "机器人反馈:执行apriltag_remove_higher({target_high})失败",
                "color_follow_2D_done": "机器人反馈:执行color_follow_2D({color})完成",
                "color_follow_2D_failed": "机器人反馈:执行color_follow_2D({color})失败",
                "color_remove_higher_done": "机器人反馈:执行color_remove_higher({color},{target_high})完成",
                "color_remove_higher_failed": "机器人反馈:执行color_remove_higher({color},{target_high})失败",
                "follw_line_clear_done": "机器人反馈:执行follw_line_clear()完成",
                "response_done": "机器人反馈：回复用户完成",
                "failure_execute_action_function_not_exists": "机器人反馈:动作函数不存在，无法执行",
                "finish": "finish",
                "multiple_done": "机器人反馈：执行{actions}完成",
                "putdown_failed": "机器人反馈:执行putdown()失败,输入参数错误",
            },
            "en": {  # 英文 / English
                "navigation_1": "Robot feedback: Navigation target {point_name} rejected",
                "navigation_2": "Robot feedback: Execute navigation({point_name}) completed",
                "navigation_3": "Robot feedback: Execute navigation({point_name}) failed, target does not exist",
                "navigation_4": "Robot feedback: Execute navigation({point_name}) failed",
                "get_current_pose_success": "Robot feedback: get_current_pose() succeeded",
                "arm_up_done": "Robot feedback: Execute arm_up() completed",
                "arm_down_done": "Robot feedback: Execute arm_down() completed",
                "drift_done": "Robot feedback: Execute drift() completed",
                "wait_done": "Robot feedback: Execute wait({duration}) completed",
                "arm_shake_done": "Robot feedback: Execute arm_shake() completed",
                "arm_nod_done": "Robot feedback: Execute arm_nod() completed",
                "arm_applaud_done": "Robot feedback: Execute arm_applaud() completed",
                "grasp_obj_done": "Robot feedback: Execute grasp_obj({x1},{y1},{x2},{y2}) completed",
                "grasp_obj_failed": "Robot feedback: Execute grasp_obj({x1},{y1},{x2},{y2}) failed",
                "putdown_done": "Robot feedback: Execute putdown() completed",
                "set_cmdvel_done": "Robot feedback: Execute set_cmdvel({linear_x},{linear_y},{angular_z},{duration}) completed",
                "move_left_done": "Robot feedback: Execute move_left({angle},{angular_speed}) completed",
                "move_right_done": "Robot feedback: Execute move_right({angle},{angular_speed}) completed",
                "turn_left_done": "Robot feedback: Execute turn_left() completed",
                "turn_right_done": "Robot feedback: Execute turn_right() completed",
                "dance_done": "Robot feedback: Execute dance() completed",
                "apriltag_sort_done": "Robot feedback: Execute apriltag_sort({target_id}) completed",
                "apriltag_sort_failed": "Robot feedback: Execute apriltag_sort({target_id}) failed",
                "apriltag_follow_2D_done": "Robot feedback: Execute apriltag_follow_2D({target_id}) completed",
                "apriltag_follow_2D_failed": "Robot feedback: Execute apriltag_follow_2D({target_id}) failed",
                "apriltag_remove_higher_done": "Robot feedback: Execute apriltag_remove_higher({target_high}) completed",
                "apriltag_remove_higher_failed": "Robot feedback: Execute apriltag_remove_higher({target_high}) failed",
                "color_follow_2D_done": "Robot feedback: Execute color_follow_2D({color}) completed",
                "color_follow_2D_failed": "Robot feedback: Execute color_follow_2D({color}) failed",
                "color_remove_higher_done": "Robot feedback: Execute color_remove_higher({color},{target_high}) completed",
                "color_remove_higher_failed": "Robot feedback: Execute color_remove_higher({color},{target_high}) failed",
                "follw_line_clear_done": "Robot feedback: Execute follw_line_clear() completed",
                "response_done": "Robot feedback: Reply to user completed",
                "failure_execute_action_function_not_exists": "Robot feedback: Execute action function not exists",
                "finish": "finish",
                "multiple_done": "Robot feedback: Execution {actions} completed",
            },
        }

    def load_target_points(self):
        """
        加载地图映射文件 /Load map mapping file
        """
        with open(self.map_mapping_config, "r") as file:
            target_points = yaml.safe_load(file)
        self.navpose_dict = {}
        for name, data in target_points.items():
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = data["position"]["x"]
            pose.pose.position.y = data["position"]["y"]
            pose.pose.position.z = data["position"]["z"]
            pose.pose.orientation.x = data["orientation"]["x"]
            pose.pose.orientation.y = data["orientation"]["y"]
            pose.pose.orientation.z = data["orientation"]["z"]
            pose.pose.orientation.w = data["orientation"]["w"]
            self.navpose_dict[name] = pose

    def arm_grasp_init(self):
        """
        初始化机械臂抓取功能 /initialize the grasping function of the robotic arm
        """
        # 机械臂状态变量/Robotic arm status variable
        self.up_joints = [90, 90, 90, 90, 90, 90]
        self.down_joints = [90, 0, 90, 90, 90, 90]
        self.detect_joints = [90, 120, 0, 0, 90, 90]
        self.init_joints = [
            90,
            130,
            0,
            5,
            90,
            0,
        ]
        # 机械臂初始姿态/robot arm initial pose
        self.putsown_joints = [
            90,
            10,
            50,
            50,
            90,
            135,
        ]  # 机械臂放下姿态/robot arm putdown pose
        while not self.TargetAngle_pub.get_subscription_count():
            self.pubSix_Arm(self.init_joints)
            time.sleep(0.1)
        self.pubSix_Arm(self.init_joints)
        self.apriltag_sort_future = Future()
        self.apriltag_follow_2D_future = Future()
        self.apriltag_remove_higher_future = Future()
        self.color_follow_2D_future = Future()
        self.color_sort_future = Future()
        self.color_remove_higher_future = Future()
        self.grasp_obj_future = Future()
        self.follw_line_clear_future = Future()

    def record_status_callback(self, msg):
        # self.get_logger().info(f"record_status_callback:{msg.data}")
        if msg.data:
            self.is_recording = True
        else:
            self.is_recording = False

    def largemodel_arm_done_callback(self, msg):
        """
        机械臂抓取完成话题回调函数/robot arm done callback function
        用于接受机械臂抓取完成话题，并设置Future对象完成 /used to receive the topic of the robotic arm grasping completion, and set the Future object to complete
        """
        if msg.data in ["apriltag_sort_done", "apriltag_sort_failed"]:
            if not self.apriltag_sort_future.done():
                self.apriltag_sort_future.set_result(msg)
        elif msg.data == "apriltag_follow_2D_done":
            if not self.apriltag_follow_2D_future.done():
                self.apriltag_follow_2D_future.set_result(msg)
        elif msg.data in [
            "apriltag_remove_higher_done",
            "apriltag_remove_higher_failed",
        ]:
            self.get_logger().info(f"msg.data:{msg.data}")
            if not self.apriltag_remove_higher_future.done():
                self.apriltag_remove_higher_future.set_result(msg)
        elif msg.data == "color_follow_2D_done":
            if not self.color_follow_2D_future.done():
                self.color_follow_2D_future.set_result(msg)
        elif msg.data == "color_sort_done":
            if not self.color_sort_future.done():
                self.color_sort_future.set_result(msg)
        elif msg.data == "grasp_obj_done":
            if not self.grasp_obj_future.done():
                self.grasp_obj_future.set_result(msg)
        elif msg.data == "color_remove_higher_done":
            if not self.color_remove_higher_future.done():
                self.color_remove_higher_future.set_result(msg)
        elif msg.data == "follw_line_clear_future_done":
            if not self.follw_line_clear_future.done():
                self.follw_line_clear_future.set_result(msg)

    def wakeup_callback(self, msg):
        """
        唤醒打断回调函数/Wake-up interrupt callback function
        用于接受唤醒信号，判断是否需要打断当前的动作、语音 /used to receive the wake-up signal, determine whether to interrupt the current action, voice
        """

        if msg.data:
            if (
                pygame.mixer.music.get_busy()  # 如果音乐正在播放/If the music is playing
            ):
                self.stop_event.set()  # 停止正在播放的音乐/Stop the music currently playing
            if (
                self.action_runing  # 如果当前有动作正在执行/If there is an action currently being
            ):
                self.interrupt_flag = True  # 置位中断标志位/Set the interruption flag
                self.stop()
        self.check_all_process()

    def get_current_pose(self):
        """
        获取当前在全局地图坐标系下的位置 /Get the current position in the global map coordinate system
        """
        # 获取当前目标点坐标
        transform = self.tf_buffer.lookup_transform(
            "map", "base_footprint", rclpy.time.Time()
        )
        # 提取位置和姿态
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = transform.transform.rotation
        self.navpose_dict["zero"] = pose
        # 打印记录的坐标
        position = pose.pose.position
        orientation = pose.pose.orientation
        self.get_logger().info(
            f"Recorded Pose - Position: x={position.x}, y={position.y},\
                                z={position.z},Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}"
        )
        if not self.interrupt_flag:
            self.action_status_pub("get_current_pose_success")

    def action_status_pub(self, key, **kwargs):
        """
        多语言版本的动作结果发布方法
        :param key: 文本标识
        :param**kwargs: 占位符参数
        """
        text_template = self.feedback_largemoel_dict[self.language].get(key)

        try:
            message = text_template.format(**kwargs)
        except KeyError as e:
            self.get_logger().error(f"Translation placeholder error: {e} (key: {key})")
            message = f"[Translation failed: {key}]"

        # 发布消息
        self.actionstatus_pub.publish(String(data=message))
        self.get_logger().info(f"Published message: {message}")

    def navigation(self, point_name):
        """
        从navpose_dict字典中获取目标点坐标.并导航到目标点
        """
        self.navigation_finish_flag = False
        self.goal_handle = None
        self.result = None
        point_name = point_name.strip("'\"")
        if point_name not in self.navpose_dict:
            self.get_logger().error(
                f"Target point '{point_name}' does not exist in the navigation dictionary."
            )
            self.action_status_pub(
                "navigation_3", point_name=point_name
            )  # 目标点地图映射中不存在
            return

        if self.first_record:
            # 出发前记录当前在全局地图中的坐标(只有在每个任务周期的第一次执行时才会记录)/ before starting a new task, record the current pose in the global map
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = transform.transform.rotation
            self.navpose_dict["zero"] = pose
            self.first_record = False

        # 获取目标点坐标 /get_target_pose
        target_pose = self.navpose_dict.get(point_name)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        send_goal_future = self.navclient.send_goal_async(goal_msg)

        def goal_response_callback(future):
            self.goal_handle = future.result()
            if not self.goal_handle or not self.goal_handle.accepted:
                self.get_logger().error("Goal was rejected!")
                self.action_status_pub("navigation_1", point_name=point_name)
                return

            get_result_future = self.goal_handle.get_result_async()

            def result_callback(future_result):
                self.result = future_result.result()
                self.navigation_finish_flag = True
                if self.result.status == 4:
                    self.action_status_pub(
                        "navigation_2", point_name=point_name
                    )  # 执行导航成功 /execute navigation success

                elif self.result.status == 5:
                    self.get_logger().info("Cancel navigation")
                else:
                    self.get_logger().info(
                        f"Navigation failed with status: {self.result.status}"
                    )
                    self.action_status_pub(
                        "navigation_4", point_name=point_name
                    )  # 执行导航失败 /execute_navigation_failed

            get_result_future.add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)

        while not self.navigation_finish_flag:
            if self.interrupt_flag and self.goal_handle is not None:
                self.navclient._cancel_goal(self.goal_handle)
                break
            time.sleep(0.1)
        self.stop()

    def pubSix_Arm(self, joints, id=6, angle=180.0, runtime=2000):
        while not self.TargetAngle_pub.get_subscription_count():
            self.get_logger().info("Waiting for arm_subscriber...")
            time.sleep(0.1)

        arm_joint = ArmJoints()
        arm_joint.joint1 = joints[0]
        arm_joint.joint2 = joints[1]
        arm_joint.joint3 = joints[2]
        arm_joint.joint4 = joints[3]
        arm_joint.joint5 = joints[4]
        arm_joint.joint6 = joints[5]
        arm_joint.time = runtime
        self.TargetAngle_pub.publish(arm_joint)

    def pubSingle_Arm(self, joint_id=6, joint_angle=180.0, runtime=800):
        arm_joint = ArmJoint()
        arm_joint.joint = int(joint_angle)
        arm_joint.id = int(joint_id)
        arm_joint.time = runtime
        self.SingleJoint_pub.publish(arm_joint)

    def pubCurrentJoints(self):
        cur_joints = CurJoints()
        cur_joints.joints = self.init_joints
        self.pub_cur_joints.publish(cur_joints)

    def arm_up(self):  # 机械臂向上
        self.done = False
        self.pubSix_Arm(self.up_joints)
        time.sleep(1.0)
        self.done = True
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("arm_up_done")

    def arm_down(self):  # 机械臂向下
        self.done = False
        self.pubSix_Arm(self.down_joints)
        time.sleep(1.0)
        self.done = True
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("arm_down_done")

    def arm_dance(self):  # 机械臂跳舞
        dance_moves = [
            [90, 90, 90, 90, 90, 90],
            [90, 60, 120, 60, 90, 90],
            [90, 45, 135, 45, 90, 90],
            [90, 60, 120, 60, 90, 90],
            [90, 90, 90, 90, 90, 90],
            [90, 100, 80, 80, 90, 90],
            [90, 120, 60, 60, 90, 90],
            [90, 135, 45, 45, 90, 90],
            [90, 90, 90, 90, 90, 90],
            [90, 90, 90, 20, 90, 150],
            [90, 90, 90, 90, 90, 90],
            [90, 90, 90, 20, 90, 150],
        ]
        for joints in dance_moves:
            if self.interrupt_flag:
                break
            self.pubSix_Arm(joints)
            time.sleep(1.0)
        self.pubSix_Arm(self.init_joints)

    def drift(self):
        """
        漂移动作
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.5
        twist.angular.z = 1.0
        self._execute_action(twist, durationtime=4.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("drift_done")

    def wait(self, duration):
        duration = float(duration)
        time.sleep(duration)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("wait_done", duration=duration)

    def arm_shake(self):  # 机械臂摇头
        for i in range(3):
            if self.interrupt_flag:
                break
            tar_arm_joint = [140, 130, 0, 5, 90, 0]
            self.pubSix_Arm(tar_arm_joint)
            time.sleep(1.0)
            tar_arm_joint = [40, 130, 0, 5, 90, 0]
            self.pubSix_Arm(tar_arm_joint)
            time.sleep(1.0)

        self.pubSix_Arm(self.init_joints)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("arm_shake_done")

    def arm_nod(self):  # 机械臂点头
        for i in range(3):
            if self.interrupt_flag:
                break
            tar_arm_joint = [90, 130, 0, 95, 90, 0]
            self.pubSix_Arm(tar_arm_joint)
            time.sleep(1.0)
            self.pubSix_Arm(self.init_joints)
            time.sleep(1.0)
        self.pubSix_Arm(self.init_joints)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("arm_nod_done")

    def arm_applaud(self):  # 机械臂鼓掌
        for i in range(3):
            if self.interrupt_flag:
                break
            tar_arm_joint = [90, 145, 0, 71, 90, 31]
            self.pubSix_Arm(tar_arm_joint)
            time.sleep(1.0)
            tar_arm_joint = [90, 145, 0, 71, 90, 168]
            self.pubSix_Arm(tar_arm_joint)
            time.sleep(1.0)
        self.pubSix_Arm(self.init_joints)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("arm_applaud_done")


    def check_track(self):
        """
        检查相关进程是否存活
        """
        try:
            subprocess.run(
                ["wmctrl", "-F", "-c", "KCF_Track"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "ALM_KCF_Tracker_Node"], check=False, timeout=2
            )
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            subprocess.run(["pkill", "-f", "KCF_track"])
            subprocess.run(["pkill", "-f", "ALM_KCF_Tracker_Node"])

    def track(self, x1, y1, x2, y2):
        """
        追踪物体
        x1,y1,x2,y2: 物体外边框坐标
        """
        self.check_track()
        cmd1 = "ros2 run largemodel_arm KCF_track"
        cmd2 = "ros2 run M3Pro_KCF ALM_KCF_Tracker_Node"

        subprocess.Popen( 
            [
                "gnome-terminal",
                "--title=KCF_track",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=ALM_KCF_Tracker_Node",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        time.sleep(5.0) #等待ALM_KCF_Tracker_Node启动完成

        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)
        self.object_position_pub.publish(Int16MultiArray(data=[x1, y1, x2, y2]))
        while True:
            if self.interrupt_flag:
                self.check_track()
                self.pubSix_Arm(self.init_joints)
                return
            time.sleep(0.1)
        self.pubSix_Arm(self.init_joints)

    def check_close_grasp_obj(self):
        """
        检查相关进程是否存活
        """
        try:
            subprocess.run(
                ["wmctrl", "-F", "-c", "grasp_desktop"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "KCF_follow"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "ALM_KCF_Tracker"], check=False, timeout=2
            )
        except subprocess.TimeoutExpired:
            # 如果关闭窗口超时，尝试强制杀死进程
            subprocess.run(["pkill", "-f", "grasp_desktop"])
            subprocess.run(["pkill", "-f", "KCF_follow"])
            subprocess.run(["pkill", "-f", "ALM_KCF_Tracker_Node"])


    def grasp_obj(self, x1, y1, x2, y2):
        """
        抓取物体
        x1,y1,x2,y2: 物体外边框坐标
        """
        self.check_close_grasp_obj()
        cmd1 = "ros2 run largemodel_arm grasp_desktop"
        cmd2 = "ros2 run largemodel_arm KCF_follow"
        cmd3 = "ros2 run M3Pro_KCF ALM_KCF_Tracker_Node"
        # cmd3 = "ros2 run --prefix 'gdb -ex run --args' M3Pro_KCF ALM_KCF_Tracker_Node"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=ALM_KCF_Tracker",
                "--",
                "bash",
                "-c",
                f"{cmd3}; exec bash",
            ]
        )
        time.sleep(5.0) #等待ALM_KCF_Tracker_Node启动完成
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=grasp_desktop",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=KCF_follow",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )
        time.sleep(1.0)
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)
        self.object_position_pub.publish(Int16MultiArray(data=[x1, y1, x2, y2]))

        while not self.grasp_obj_future.done():
            if self.interrupt_flag:
                self.check_close_grasp_obj()
                self.pubSix_Arm(self.init_joints)  # 机械臂收回
                self.stop()
                return
            time.sleep(0.1)

        result = self.grasp_obj_future.result()
        if not self.interrupt_flag:
            if result.data == "grasp_obj_done":
                self.action_status_pub("grasp_obj_done", x1=x1, y1=y1, x2=x2, y2=y2)
            else:
                self.action_status_pub("grasp_obj_failed", x1=x1, y1=y1, x2=x2, y2=y2)

        self.check_close_grasp_obj()
        self.grasp_obj_future = Future()  # 复位Future对象
        if self.interrupt_flag:
            time.sleep(0.5)
            self.pubSix_Arm(self.init_joints)  # 机械臂收回

    def putdown(self):
        self.pubSix_Arm(self.putsown_joints)  # 机械臂下放
        time.sleep(4)
        self.pubSingle_Arm(6, 30, 1000)  # 机械臂打开夹抓，放下物品
        time.sleep(3)
        self.pubSix_Arm(self.init_joints)  # 机械臂收回
        if not self.interrupt_flag:
            self.action_status_pub("putdown_done")


    def seewhat(self):
        self.save_single_image()
        msg = String(data="seewhat")
        self.seewhat_handle_pub.publish(
            msg
        )  # 归一化，发布seewhat话题，由model_service调用大模型

    def set_cmdvel(self, linear_x, linear_y, angular_z, duration):  # 发布cmd_vel
        # 将参数从字符串转换为浮点数
        linear_x = float(linear_x)
        linear_y = float(linear_y)
        angular_z = float(angular_z)
        duration = float(duration)
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self._execute_action(twist, durationtime=duration)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub(
                "set_cmdvel_done",
                linear_x=linear_x,
                linear_y=linear_y,
                angular_z=angular_z,
                duration=duration,
            )

    def move_left(self, angle, angular_speed):  # 左转x度
        angle = float(angle)
        angular_speed = float(angular_speed)
        angle_rad = math.radians(angle)  # 将角度转换为弧度
        duration = abs(angle_rad / angular_speed)
        angular_speed = abs(angular_speed)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        self._execute_action(twist, 1, duration)
        self.stop()
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub(
                "move_left_done",
                angle=angle,
                angular_speed=angular_speed,
            )

    def move_right(self, angle, angular_speed):  # 右转x度
        angle = float(angle)
        angular_speed = float(angular_speed)
        angle_rad = math.radians(angle)  # 将角度转换为弧度
        duration = abs(angle_rad / angular_speed)
        angular_speed = -abs(angular_speed)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        self._execute_action(twist, 1, duration)
        self.stop()
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub(
                "move_right_done",
                angle=angle,
                angular_speed=angular_speed,
            )

    def turn_left(self):  # 左转弯
        twist = Twist()
        twist.linear.x = 0.4
        twist.angular.z = 1.0
        self._execute_action(twist)
        self.stop()
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("turn_left_done")

    def turn_right(self):  # 右转弯
        twist = Twist()
        twist.linear.x = 0.4
        twist.angular.z = -1.0
        self._execute_action(twist)
        self.stop()
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("turn_right_done")

    def dance(self):  # 跳舞
        thread = Thread(target=self.arm_dance)
        thread.start()
        actions = [
            {"linear_x": 0.6, "linear_y": 0.0, "angular_z": 0.0, "durationtime": 1.5},
            {"linear_x": -0.4, "linear_y": 0.0, "angular_z": 0.0, "durationtime": 1.0},
            {"linear_x": 0.0, "linear_y": 0.3, "angular_z": 0.0, "durationtime": 1.0},
            {"linear_x": 0.0, "linear_y": -0.3, "angular_z": 0.0, "durationtime": 1.0},
            {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.6, "durationtime": 3.0},
            {"linear_x": 0.0, "linear_y": 0.0, "angular_z": -0.6, "durationtime": 3.0},
        ]

        for action in actions:
            if self.interrupt_flag:
                break
            twist = Twist()
            twist.linear.x = action["linear_x"]
            twist.linear.y = action["linear_y"]
            twist.angular.z = action["angular_z"]
            self._execute_action(twist, durationtime=action["durationtime"])

        thread.join(timeout=5.0)
        self.stop()
        self.pubSix_Arm(self.init_joints)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("dance_done")

    def stop(self):  # 停止
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def _execute_action(self, twist, num=1, durationtime=3.0):
        for _ in range(num):
            start_time = time.time()
            while (time.time() - start_time) < durationtime:
                if self.interrupt_flag:
                    self.stop()
                    return
                self.publisher.publish(twist)
                time.sleep(0.1)

    def check_apriltag_sort(self):
        try:
            subprocess.run(
                ["wmctrl", "-F", "-c", "grasp_desktop_apritag"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "apriltag_sort"], check=False, timeout=2
            )
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            subprocess.run(["pkill", "-f", "grasp_desktop"])
            subprocess.run(["pkill", "-f", "apriltag_sort"])

    def apriltag_sort(self, target_id):  # 夹取机器码
        self.check_apriltag_sort()
        target_idf = float(target_id)
        cmd1 = "ros2 run largemodel_arm grasp_desktop_apritag"
        cmd2 = f"ros2 run largemodel_arm apriltag_sort --ros-args -p target_id:={target_idf:.1f}"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=grasp_desktop_apritag",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=apriltag_sort",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )

        while not self.apriltag_sort_future.done():
            if self.interrupt_flag:
                self.check_apriltag_sort()
                self.stop()
                self.pubSix_Arm(self.init_joints)
                return
            time.sleep(0.1)

        result = self.apriltag_sort_future.result()
        if not self.interrupt_flag:
            if result.data == "apriltag_sort_done":
                self.action_status_pub("apriltag_sort_done", target_id=target_id)
            elif result.data == "apriltag_sort_failed":
                self.action_status_pub("apriltag_sort_failed", target_id=target_id)

        self.check_apriltag_sort()
        self.apriltag_sort_future = Future()  # 复位Future对象

    def check_apriltag_remove_higher(self):
        try:
            subprocess.run(
                ["wmctrl", "-F", "-c", "grasp_desktop_remove"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "apriltag_remove_higher"], check=False, timeout=2
            )
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            subprocess.run(["pkill", "-f", "grasp_desktop_remove"])
            subprocess.run(["pkill", "-f", "apriltag_remove_higher"])

    def apriltag_remove_higher(self, target_high):  # 移除指定高度的机器码
        self.check_apriltag_remove_higher()
        target_highf = float(target_high) / 100
        cmd1 = "ros2 run largemodel_arm grasp_desktop_remove"
        cmd2 = f"ros2 run largemodel_arm apriltag_remove_higher --ros-args -p target_high:={target_highf:.2f}"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=grasp_desktop_remove",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=apriltag_remove_higher",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )

        while not self.apriltag_remove_higher_future.done():
            if self.interrupt_flag:
                self.check_apriltag_remove_higher()
                self.stop()
                self.pubSix_Arm(self.init_joints)
                return
            time.sleep(0.1)
        result = self.apriltag_remove_higher_future.result()

        if not self.interrupt_flag:
            if result.data == "apriltag_remove_higher_done":
                self.action_status_pub(
                    "apriltag_remove_higher_done", target_high=target_high
                )
            elif result.data == "apriltag_remove_higher_failed":
                self.action_status_pub(
                    "apriltag_remove_higher_failed", target_high=target_high
                )

        self.check_apriltag_remove_higher()
        self.apriltag_remove_higher_future = Future()  # 复位Future对象
        self.pubSix_Arm(self.init_joints)

    def check_color_remove_higher(self):
        try:
            subprocess.run(
                ["wmctrl", "-F", "-c", "grasp_desktop_remove_color"], check=False, timeout=2
            )
            subprocess.run(["wmctrl", "-F", "-c", "color_remove_higher"], check=False, timeout=2)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            subprocess.run(["pkill", "-f", "grasp_desktop_remove_color"])
            subprocess.run(["pkill", "-f", "color_remove_higher"])

    def color_remove_higher(self, color, target_high):
        self.check_color_remove_higher()
        arm_joints = [90, 110, 0, 0, 90, 0]
        self.pubSix_Arm(arm_joints)
        color = color.strip("'\"")  # 去掉单引号和双引号
        target_highf = float(target_high) / 100
        if color == "red":
            target_color = float(1)
        elif color == "green":
            target_color = float(2)
        elif color == "blue":
            target_color = float(3)
        elif color == "yellow":
            target_color = float(4)
        else:
            self.get_logger().info(
                "Fatal ERROR:Incorrect color input,Does the AI output not meet expectations?"
            )
            self.action_status_pub(
                "color_remove_higher_failed", color=color, target_high=target_high
            )
            return
        
        cmd1 = "ros2 run largemodel_arm grasp_desktop_remove_color"
        cmd2 = f"ros2 run largemodel_arm color_remove_higher --ros-args -p target_high:={target_highf:.2f} -p target_color:={target_color:.1f}"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=grasp_desktop_remove_color",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=color_remove_higher",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )

        while not self.color_remove_higher_future.done():
            if self.interrupt_flag:
                self.check_color_remove_higher()
                self.stop()
                self.pubSix_Arm(self.init_joints)
                return
            time.sleep(0.1)

        result = self.color_remove_higher_future.result()
        if not self.interrupt_flag:
            if result.data == "color_remove_higher_done":
                self.action_status_pub(
                    "color_remove_higher_done", color=color, target_high=target_high
                )
            else:
                self.action_status_pub(
                    "color_remove_higher_failed", color=color, target_high=target_high
                )

        self.check_color_remove_higher()
        self.color_remove_higher_future = Future()  # 复位Future对象
        self.pubSix_Arm(self.init_joints)

    def check_follw_line_clear(self):
        try:
            subprocess.run(
                ["wmctrl", "-F", "-c", "grasp_desktop_remove"], check=False, timeout=2
            )
            subprocess.run(["wmctrl", "-F", "-c", "follow_line"], check=False, timeout=2)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            subprocess.run(["pkill", "-f", "grasp_desktop_remove"])
            subprocess.run(["pkill", "-f", "follow_line"])

    def follw_line_clear(self) -> None:
        self.check_follw_line_clear()
        cmd1 = "ros2 run largemodel_arm grasp_desktop_remove"
        cmd2 = "ros2 run largemodel_arm follow_line --ros-args -p start_follow:=True"
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=grasp_desktop_remove",
                "--",
                "bash",
                "-c",
                f"{cmd1}; exec bash",
            ]
        )
        subprocess.Popen(
            [
                "gnome-terminal",
                "--title=follow_line",
                "--",
                "bash",
                "-c",
                f"{cmd2}; exec bash",
            ]
        )

        while not self.follw_line_clear_future.done():
            if self.interrupt_flag:
                self.check_follw_line_clear()
                self.stop()
                self.pubSix_Arm(self.init_joints)
                return
            time.sleep(0.1)

        if not self.interrupt_flag:
            if self.follw_line_clear_future.result() is not None:
                self.action_status_pub("follw_line_clear_done")

        self.check_follw_line_clear()
        self.follw_line_clear_future = Future()  # 复位Future对象
        self.pubSix_Arm(self.init_joints)


    def check_all_process(self):
        try:
            subprocess.run(
                ["wmctrl", "-F", "-c", "KCF_Track"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "ALM_KCF_Tracker_Node"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "ALM_KCF_Tracker"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "grasp_desktop"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "KCF_follow"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "apriltag_sort"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "grasp_desktop_apritag"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "grasp_desktop_remove"], check=False, timeout=2
            )
            subprocess.run(
                ["wmctrl", "-F", "-c", "apriltag_remove_higher"], check=False, timeout=2
            )

            subprocess.run(
                ["wmctrl", "-F", "-c", "grasp_desktop_remove_color"], check=False, timeout=2
            )
            subprocess.run(["wmctrl", "-F", "-c", "color_remove_higher"], check=False, timeout=2)

            subprocess.run(["wmctrl", "-F", "-c", "follow_line"], check=False, timeout=2)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            subprocess.run(["pkill", "-f", "KCF_track"])
            subprocess.run(["pkill", "-f", "ALM_KCF_Tracker_Node"])
            subprocess.run(["pkill", "-f", "ALM_KCF_Tracker"])
            subprocess.run(["pkill", "-f", "grasp_desktop"])
            subprocess.run(["pkill", "-f", "KCF_follow"])
            subprocess.run(["pkill", "-f", "apriltag_sort"])
            subprocess.run(["pkill", "-f", "grasp_desktop_apritag"])
            subprocess.run(["pkill", "-f", "grasp_desktop_remove"])
            subprocess.run(["pkill", "-f", "apriltag_remove_higher"])
            subprocess.run(["pkill", "-f", "grasp_desktop_remove_color"])
            subprocess.run(["pkill", "-f", "color_remove_higher"])
            subprocess.run(["pkill", "-f", "follow_line"])




    # 核心程序，解析动作列表并执行  # Core program, parse and execute action list
    def execute_callback(self, goal_handle):
        """
        动作执行回调函数：分3种情况：  # Action execution callback function: divided into 3 cases:
        1. 动作列表为空  # 1. Empty action list
        2. 动作列表长度为1  # 2. Action list length is 1
        3. 动作列表长度大于1  # 3. Action list length is greater than 1
        文字交互模式下，不进行语音合成和播放  # In text interaction mode, no voice synthesis or playback is performed
        """

        if self.is_recording:
            goal_handle.succeed()
            result = Rot.Result()
            result.success = True
            return result

        feedback_msg = Rot.Feedback()
        actions = goal_handle.request.actions
        self.action_runing = True
        if not actions:  # 动作列表为空  # If the action list is empty
            if not self.text_chat_mode and (
                goal_handle.request.llm_response is not None
                or goal_handle.request.text_response != ""
            ):  # 语音模式，播放对话  # Voice mode, play dialogue
                self.model_client.voice_synthesis(
                    goal_handle.request.llm_response, self.tts_out_path
                )
                self.play_audio(self.tts_out_path, feedback=True)
            else:
                self.action_status_pub("response_done")

        elif len(actions) == 1:  # 动作列表长度为1  # If the action list length is 1

            action = actions[0]
            if not self.text_chat_mode and (
                goal_handle.request.llm_response is not None
                or goal_handle.request.text_response != ""
            ):  # 语音模式，播放对话  # Voice mode, play dialogue
                self.model_client.voice_synthesis(
                    goal_handle.request.llm_response, self.tts_out_path
                )
                self.play_audio(self.tts_out_path)

            match = re.match(r"(\w+)\((.*)\)", action)
            action_name, args_str = match.groups()
            if not hasattr(self, action_name):
                self.get_logger().warning(
                    f"action_service: {action} is invalid action,skip execution"
                )
                self.action_status_pub(
                    "failure_execute_action_function_not_exists"
                )  # Robot feedback: action function does not exist, cannot execute

            else:
                action_name, args_str = match.groups()
                args = [arg.strip() for arg in args_str.split(",")] if args_str else []
                method = getattr(self, action_name)
                method(*args)

            if self.interrupt_flag:
                self.interrupt_flag = False
        else:  # 动作列表长度大于1,使能组合模式  # If the action list length is greater than 1, enable combination mode

            self.combination_mode = True
            if not self.text_chat_mode and (
                goal_handle.request.llm_response is not None
                or goal_handle.request.text_response != ""
            ):  # 语音模式，播放对话  # Voice mode, play dialogue
                self.model_client.voice_synthesis(
                    goal_handle.request.llm_response, self.tts_out_path
                )
                self.play_audio_async(self.tts_out_path)

            for action in actions:
                if self.interrupt_flag:
                    break
                match = re.match(r"(\w+)\((.*)\)", action)
                action_name, args_str = match.groups()
                args = [arg.strip() for arg in args_str.split(",")] if args_str else []

                if not hasattr(self, action_name):
                    self.get_logger().warning(
                        f"action_service: {action} is invalid action，skip execution"  # action_service: {action} is an invalid action, skip execution
                    )
                    self.action_status_pub(
                        "failure_execute_action_function_not_exists"
                    )  # Robot feedback: action function does not exist, cannot execute
                else:
                    method = getattr(self, action_name)
                    method(*args)
                    feedback_msg.status = f"action service execute  {action}  successed"

            if not self.interrupt_flag:
                self.action_status_pub(
                    "multiple_done", actions=actions
                )  # Robot feedback: execution of {actions} completed
            self.combination_mode = (
                False  # 重置组合模式标志位  # Reset combination mode flag
            )
        self.stop()  # 执行完全部动作停止机器人  # Stop the robot after executing all actions
        self.action_runing = False  # 重置运行标志位  # Reset running flag
        self.interrupt_flag = False
        goal_handle.succeed()
        result = Rot.Result()
        result.success = True
        return result

    def finish_dialogue(self):  # 发布AI模型结束当前流程标志
        self.first_record = True  # 重置导航记录标志位 # Reset navigation record flag
        self.is_recording = False  # 重置录音标志位  # Reset recording flag
        self.pubSix_Arm(self.init_joints)
        self.action_status_pub("finish")  # 结束当前任务

    def finishtask(self):
        """
        空操作,不反馈消息，用于结束反馈
        """
        return

    def kill_process_tree(pid):
        try:
            parent = psutil.Process(pid)
            children = parent.children(recursive=True)

            # 先终止所有子进程
            for child in children:
                try:
                    child.terminate()
                except psutil.NoSuchProcess:
                    pass

            # 等待子进程终止
            gone, alive = psutil.wait_procs(children, timeout=3)

            # 强制杀死仍然存活的进程
            for p in alive:
                try:
                    p.kill()
                except psutil.NoSuchProcess:
                    pass

            # 最后终止父进程
            try:
                parent.terminate()
                parent.wait(timeout=3)
            except psutil.TimeoutExpired:
                parent.kill()
            except psutil.NoSuchProcess:
                pass

        except psutil.NoSuchProcess:
            pass

    def play_audio(self, file_path: str, feedback: Bool = False) -> None:
        """
        同步方式播放音频函数The function for playing audio in synchronous mode
        """
        if self.is_recording:
            return
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            if self.stop_event.is_set() or self.is_recording:
                pygame.mixer.music.stop()
                self.stop_event.clear()  # 清除事件
                return
            pygame.time.Clock().tick(10)
        if feedback:
            self.action_status_pub("response_done")

    def play_audio_async(self, file_path: str, feedback: Bool = False) -> None:
        """
        异步方式播放音频函数The function for playing audio in asynchronous mode
        """
        if self.is_recording:
            return

        def target():
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                if self.stop_event.is_set() or self.is_recording:
                    pygame.mixer.music.stop()
                    self.stop_event.clear()  # 清除事件
                    return
                pygame.time.Clock().tick(5)
            if feedback:
                self.action_status_pub("response_done")

        thread = threading.Thread(target=target)
        thread.daemon = True
        thread.start()

    def save_single_image(self):
        """
        保存一张图片 / Save a single image
        """
        self.IS_SAVING=True
        time.sleep(0.1)
        if self.image_msg is None:
            self.get_logger().warning("No image received yet.")  # 尚未接收到图像...
            return
        try:
            # 将ROS图像消息转换为OpenCV图像 / Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, "bgr8")
            # 保存图片 / Save the image
            cv2.imwrite(self.image_save_path, cv_image)
            display_thread = threading.Thread(target=self.display_saved_image)
            display_thread.start()

        except Exception as e:
            self.get_logger().error(f"Error saving image: {e}")  # 保存图像时出错...
        self.IS_SAVING=False

    def display_saved_image(self):
        """
        显示已保存的图片4秒后关闭窗口 / Display the saved image for 4 seconds before closing the window
        """
        try:
            img = cv2.imread(self.image_save_path)
            if img is not None:
                cv2.imshow("Saved Image", img)
                cv2.waitKey(4000)  # 等待4秒 / Wait for 4 seconds
                cv2.destroyAllWindows()
            else:
                self.get_logger().error(
                    "Failed to load saved image for display."
                )  # 加载保存的图像以供显示失败...
        except Exception as e:
            self.get_logger().error(f"Error displaying image: {e}")  # 显示图像时出错...

    def image_callback(self, msg):  # 图像回调函数 / Image callback function
        if not self.IS_SAVING:
            self.image_msg = msg
        else:
            self.get_logger().error("The image is being saved and no new information will be accepted")


def main(args=None):
    rclpy.init(args=args)
    custom_action_server = CustomActionServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(custom_action_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        custom_action_server.stop()
        pass
    finally:
        custom_action_server.stop()
        custom_action_server.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()



