"""
Lab 4 - Galactic Navigate to Goal
Author: Griffin Martin & Poorvaja Veera Balaji Kumar
"""

import rclpy
from rclpy.node import Node
import numpy as np
from pathlib import Path
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


waypoint_path = Path(__file__).with_name('wayPoints.txt')

# if waypoint_path.exists():
#     goals = np.atleast_2d(np.loadtxt(waypoint_path, dtype=float))
#     rclpy.logging.get_logger('go_to_goal').info(f'Loaded waypoints from {waypoint_path}: {goals.tolist()}')
# else:
#     goals = np.empty((0, 2), dtype=float)
#     rclpy.logging.get_logger('go_to_goal').warning(
#         f'Waypoint file {waypoint_path} does not exist. No goals will be published.'
#     )

goals = np.array([[1.5, 0], [1.5, 1.4], [0, 1.4]])

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.declare_parameter('publish_frequency', 20)
        self.declare_parameter('goal_tolerance', 0.05)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        # set up subscriber
        self.init_pose_set = False
        self.init_angle = 0.0
        self.init_pos = Point()
        self.current_pos = Point()
        self.current_orientation = 0.0
        self.current_goal_index = 0
        self.all_goals_reached = False
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_subscription_callback, qos_profile)

        # set up publisher
        self.publisher_ = self.create_publisher(Vector3, '/goal_pos', qos_profile)
        self.timer = self.create_timer(1/self.get_parameter('publish_frequency').value, self.publisher_callback)

    def publisher_callback(self):
        msg = Vector3()

        if not self.init_pose_set or goals.shape[0] == 0 or self.all_goals_reached:
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0
        else:
            current_goal = goals[self.current_goal_index]
            goal_dx = float(current_goal[0] - self.current_pos.x)
            goal_dy = float(current_goal[1] - self.current_pos.y)

            goal_distance = np.hypot(goal_dx, goal_dy)
            goal_tolerance = float(self.get_parameter('goal_tolerance').value)

            if goal_distance <= goal_tolerance:
                self.current_goal_index += 1
                if self.current_goal_index >= goals.shape[0]:
                    self.all_goals_reached = True
                    self.get_logger().info('All goals reached.')
                    self.publisher_.publish(msg)
                    return
                current_goal = goals[self.current_goal_index]
                goal_dx = float(current_goal[0] - self.current_pos.x)
                goal_dy = float(current_goal[1] - self.current_pos.y)

            cos_yaw = np.cos(self.current_orientation)
            sin_yaw = np.sin(self.current_orientation)
            goal_x_robot = cos_yaw * goal_dx + sin_yaw * goal_dy
            goal_y_robot = -sin_yaw * goal_dx + cos_yaw * goal_dy

            msg.x = float(goal_x_robot)
            msg.y = float(goal_y_robot)
            msg.z = 0.0
        self.publisher_.publish(msg)

    def odom_subscription_callback(self, msg):
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        orientation = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        if not self.init_pose_set:
            self.init_angle = orientation
            rotation = np.array([
                [np.cos(self.init_angle), np.sin(self.init_angle)],
                [-np.sin(self.init_angle), np.cos(self.init_angle)]
            ])
            init_xy = rotation @ np.array([position.x, position.y])
            self.init_pos.x = float(init_xy[0])
            self.init_pos.y = float(init_xy[1])
            self.init_pose_set = True

        rotation = np.array([
            [np.cos(self.init_angle), np.sin(self.init_angle)],
            [-np.sin(self.init_angle), np.cos(self.init_angle)]
        ])
        current_xy = rotation @ np.array([position.x, position.y])

        self.current_pos.x = float(current_xy[0] - self.init_pos.x)
        self.current_pos.y = float(current_xy[1] - self.init_pos.y)
        self.current_orientation = float(np.arctan2(np.sin(orientation - self.init_angle), np.cos(orientation - self.init_angle)))


def main(args=None):
    rclpy.init(args=args)

    node = GoToGoal()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
