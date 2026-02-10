import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist


class RotateRobot(Node):

    def __init__(self):

        # initialization things
        super().__init__('minimal_publisher')
        self.declare_parameter('publish_frequency', 20)
        self.declare_parameter('pixel_width', 640)
        self.declare_parameter('kp', 0.004)
        self.kp = self.get_parameter('kp').value

        # set up publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1/self.get_parameter('publish_frequency').value, self.publisher_callback)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        # set up subscriber
        self.point_msg = None
        self.subscription_ = self.create_subscription(Point, '/galactic_object_follower/object_coords', self.subscription_callback, qos_profile)

    def publisher_callback(self):
        if self.point_msg is None:
            return

        # get pixel distance from center of screen
        x = self.point_msg.x
        dist = x - self.get_parameter('pixel_width').value / 2

        msg = Twist()
        # linear velocity defaults to zero, don't need to modify it
        # angular velocity should be in the realm of 1 for large values of dist
        msg.angular.z = -self.kp*dist
        self.publisher_.publish(msg)

    def subscription_callback(self, msg):
        self.point_msg = msg


def main(args=None):
    rclpy.init(args=args)

    node = RotateRobot()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
