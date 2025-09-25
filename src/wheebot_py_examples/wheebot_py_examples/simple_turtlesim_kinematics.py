import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math


class SimpleTurtlesimKinematics(Node):
    def __init__(self):
        super().__init__("simple_turtlesim_kinematics")

        self.turtle1_pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.turtle1PoseCallback, 10)
        self.turtle2_pose_sub = self.create_subscription(Pose, "/turtle2/pose", self.turtle2PoseCallback, 10)

        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()

    def turtle1PoseCallback(self, msg):
        self.last_turtle1_pose_ = msg


    def turtle2PoseCallback(self, msg):
        self.last_turtle2_pose_ = msg

        Tx = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x
        Ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y

        theta_rad = self.last_turtle2_pose_.theta - self.last_turtle1_pose_.theta
        theta_deg = theta_rad * 180.0 / 3.141592653589793

        self.get_logger().info("""\n
                               Translation Vector from turtle1 to turtle2:\n
                               Tx: %f\n
                               Ty: %f\n
                               Rotation Matrix from turtle1 to turtle2:\n
                               theta (rad): %f\n
                               theta (deg): %f\n
                               |R11  R12| : |%f  %f|\n
                               |R11  R12| : |%f  %f|\n
                               """ % (Tx, Ty, theta_rad, theta_deg, math.cos(theta_rad), -math.sin(theta_rad), math.sin(theta_rad), math.cos(theta_rad)))
        

def main():
    rclpy.init()
    simple_turtlesim_kinematics = SimpleTurtlesimKinematics()
    rclpy.spin(simple_turtlesim_kinematics)
    simple_turtlesim_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()