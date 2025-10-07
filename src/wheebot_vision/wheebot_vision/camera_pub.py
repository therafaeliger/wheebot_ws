#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Define the publisher
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Set the rate
        self.timer = self.create_timer(1.0 / 30, self.timer_callback)  # 30 Hz
        
        # Initialize the video capture
        self.video_capture = cv2.VideoCapture(0)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
    
    def timer_callback(self):
        # Read a frame from the video file
        return_value, capture_frame = self.video_capture.read()
        if return_value:
            self.get_logger().info("Video frame captured and published!")
            
            # Convert the frame to a ROS Image message
            image_to_transmit = self.bridge.cv2_to_imgmsg(capture_frame, encoding='bgr8')
            self.publisher.publish(image_to_transmit)
        else:
            self.get_logger().info("End of video file reached.")
            self.video_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Loop the video

def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.video_capture.release()
    camera_publisher.destroy_node()
    
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()