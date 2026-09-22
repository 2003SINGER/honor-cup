import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros
import cv2
import numpy as np
import tf_transformations
from cv_bridge import CvBridge
from dt_apriltags import Detector
import transforms3d
import transforms3d.euler as t3d_euler
class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        # 创建 TF 广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 创建 CvBridge 用于转换图像格式
        self.bridge = CvBridge()

        # 订阅相机图像和相机信息
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

        # AprilTag 检测器
        self.detector = Detector()

        # 相机内参矩阵和畸变系数（初始化为 None，稍后从 camera_info 消息中填充）
        self.camera_matrix = None
        self.dist_coeffs = None

    def camera_info_callback(self, msg):
        # 获取相机内参矩阵和畸变系数
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn("Waiting for camera info...")
            return

        # 将 ROS 图像消息转换为 OpenCV 图像
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 转换为灰度图
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 检测 AprilTags
        results = self.detector.detect(gray)

        for result in results:
            tag_id = result.tag_id
            corners = result.corners
            center = result.center
            self.get_logger().info(f"Detected tag {tag_id} at {center}")

            # 计算标记的位姿
            # 这里假设标记的四个角点在世界坐标系中的坐标为 (x, y, z)
            tag_size = 0.05  # 标记的尺寸（单位：米）
            object_points = np.array([
                [-tag_size / 2, -tag_size / 2, 0],
                [tag_size / 2, -tag_size / 2, 0],
                [tag_size / 2, tag_size / 2, 0],
                [-tag_size / 2, tag_size / 2, 0]
            ], dtype=np.float32)

            image_points = np.array(corners, dtype=np.float32)

            # 使用 solvePnP 计算相机到 AprilTag 的位姿
            retval, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)
            

            if retval:
                # 将旋转向量转换为旋转矩阵
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                #print("rotation_matrix: ",rotation_matrix)
                if rotation_matrix.shape != (3, 3):
                    raise ValueError("输入矩阵必须是3x3旋转矩阵")

                # 将旋转矩阵转换为四元数
                quat = transforms3d.quaternions.mat2quat(rotation_matrix)
                euler_angles = t3d_euler.mat2euler(rotation_matrix)
                print("euler_angles: ",euler_angles)
                # 创建 TransformStamped 消息
                '''transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = 'DCW2'
                transform.child_frame_id = f'apriltag_{tag_id}'
                #print("tvec[0]: ",tvec[0])
                transform.transform.translation.x = float(tvec[0])
                transform.transform.translation.y = float(tvec[1]) 
                transform.transform.translation.z = float(tvec[2]) 
                transform.transform.rotation.x = float(quat[0]) 
                transform.transform.rotation.y = float(quat[1]) 
                transform.transform.rotation.z = float(quat[2]) 
                transform.transform.rotation.w = float(quat[3]) 

                # 发布变换
                self.tf_broadcaster.sendTransform(transform)'''

                # 在图像中绘制检测到的标签
                cv2.polylines(cv_image, [np.int32(corners)], isClosed=True, color=(0, 255, 0), thickness=2)
                cv2.putText(cv_image, f"ID: {tag_id}", tuple(np.int32(center)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # 显示带有标记的图像
        cv2.imshow("AprilTag Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
