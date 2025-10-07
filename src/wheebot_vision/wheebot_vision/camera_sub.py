#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Create a subscriber for the video_topic
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callbackFunction, 10)
        
        # Create an OpenCV window
        cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)

    def callbackFunction(self, msg):
        self.get_logger().info("Received Video Message!")
        
        try:
            # Convert ROS Image message to OpenCV image
            converted_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Display the image
            cv2.imshow("Camera", converted_frame)
            cv2.waitKey(1)
        except:
            self.get_logger().error(f"Failed to process image: {str()}")

def main(args=None):
    rclpy.init(args=args)
    
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()

    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()