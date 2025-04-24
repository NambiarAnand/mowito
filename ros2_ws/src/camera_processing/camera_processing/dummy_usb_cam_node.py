#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np

class DummyUsbCam(Node):
    def __init__(self):
        super().__init__('dummy_usb_cam')
        
        # Configuration - modify these as needed
        self.image_filename = 'images.jpeg'  # Change this to your image file
        self.frame_rate = 30  # Hz
        self.topic_name = '/image_raw'
        
        # Get full path to image
        self.image_path = os.path.join(
            get_package_share_directory('camera_processing'),
            'assets',
            self.image_filename
        )
        self.image_path = '/home/nambiar/mowito/ros_ws/src/camera_processing/assets/images.jpeg'
        
        # Initialize image source
        if os.path.exists(self.image_path):
            self.get_logger().info(f"Loading image from: {self.image_path}")
            self.image = cv2.imread(self.image_path)
            if self.image is None:
                self.get_logger().error(f"Failed to read image: {self.image_path}")
                raise RuntimeError(f"Failed to read image: {self.image_path}")
            self.height, self.width = self.image.shape[:2]
            self.test_pattern = False
        else:
            self.get_logger().warn(f"Image not found at {self.image_path}, using test pattern")
            self.test_pattern = True
            self.width = 640
            self.height = 480
        
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, self.topic_name, 10)
        self.timer = self.create_timer(1.0/self.frame_rate, self.timer_callback)
        self.get_logger().info(f"Dummy USB Camera publishing to {self.topic_name} at {self.frame_rate}Hz")

    def timer_callback(self):
        if self.test_pattern:
            img = self.create_test_pattern()
        else:
            img = self.image.copy()
        
        try:
            ros_image = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def create_test_pattern(self):
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        cv2.putText(img, "DUMMY CAM", (50, self.height//2), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        return img

def main(args=None):
    rclpy.init(args=args)
    node = DummyUsbCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()