import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')

        self.sub_ = self.create_subscription(String, 'chatter', self.msgCallback, 10)
        self.get_logger().info("Waiting for messages on 'chatter' topic...")

    def msgCallback(self, msg):
        self.get_logger().info("Reveived message: %s" % msg.data)


def main():
    rclpy.init()
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    rclpy.shutdown()
    simple_subscriber.destroy_node()


if __name__ == "__main__":
    main()