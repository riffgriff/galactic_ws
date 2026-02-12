"""
Lab 2 - Object Follower
Author: Griffin Martin & Poorvaja Veera Balaji Kumar
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


class RotateRobot(Node):

    def __init__(self):

        # initialization things
        super().__init__('minimal_publisher')
        self.declare_parameter('publish_frequency', 20)
        self.declare_parameter('pixel_width', 320)
        self.declare_parameter('kp', 0.008)
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
        msg = Twist()
        # linear velocity defaults to zero, don't need to modify it
        
        if self.point_msg is None:
            msg.angular.z = 0.0
        else:
            # get pixel distance from center of screen
            x = self.point_msg.x
            dist = x - self.get_parameter('pixel_width').value / 2
    
            
            # angular velocity should be in the realm of 1 for large values of dist
            msg.angular.z = -self.kp*dist
            
        self.publisher_.publish(msg)

    def subscription_callback(self, msg):
        if msg.z != 0:
            self.point_msg = None
        else:
            self.point_msg = msg


def main(args=None):
    rclpy.init(args=args)

    node = RotateRobot()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
