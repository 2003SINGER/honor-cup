#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge
class RgbImagePublish(Node):

    def __init__(self):
        super().__init__('rgb_image_publisher')
        self.sub_rgb = self.create_subscription(Image,"/camera/color/image_raw",self.get_RGBImageCallBack,1)
        self.rgb_bridge = CvBridge()
        self.rgb_img_pub = self.create_publisher(CompressedImage,"/camera/rgb/image_raw/compressed",1)
        self.quality = 90
        
        
    def get_RGBImageCallBack(self, msg):
        cv_image = self.rgb_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        
        result, encoded_img = cv2.imencode('.jpg', cv_image, encode_param)
        
        if not result:
            self.get_logger().error("图像压缩失败")
            return
        
        compressed_msg = CompressedImage()
        compressed_msg.header = msg.header  # 保留原始时间戳和坐标系
        compressed_msg.format = 'jpeg'  # 表示这是JPEG压缩图像
        compressed_msg.data = encoded_img.tobytes()
        
        self.rgb_img_pub.publish(compressed_msg)

def main(args=None):
    rclpy.init(args=args)

    rgb_image_pub = RgbImagePublish()

    rclpy.spin(rgb_image_pub)

    rgb_image_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

