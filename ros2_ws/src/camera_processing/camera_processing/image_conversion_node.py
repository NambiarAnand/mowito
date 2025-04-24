#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2

class ImageConversion(Node):
    def __init__(self):
        super().__init__('image_conversion')
        
        # Parameters
        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('output_topic', '/converted_image')
        
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # State
        self.grayscale_mode = False
        self.bridge = CvBridge()
        
        # Service
        self.service = self.create_service(
            SetBool,
            'set_grayscale',
            self.set_grayscale_callback)
        
        # Subscriber and Publisher
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            Image,
            self.output_topic,
            10)
        
        self.get_logger().info(f"Image Conversion node started")
        self.get_logger().info(f"Subscribed to: {self.input_topic}")
        self.get_logger().info(f"Publishing to: {self.output_topic}")
        self.get_logger().info(f"Current mode: {'Grayscale' if self.grayscale_mode else 'Color'}")

    def set_grayscale_callback(self, request, response):
        self.grayscale_mode = request.data
        response.success = True
        response.message = f"Mode changed to {'grayscale' if self.grayscale_mode else 'color'}"
        self.get_logger().info(response.message)
        return response

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            if self.grayscale_mode:
                processed_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                # Convert back to BGR for consistent message type
                processed_image = cv2.cvtColor(processed_image, cv2.COLOR_GRAY2BGR)
            else:
                processed_image = cv_image
            
            ros_image = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
            self.publisher.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageConversion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()