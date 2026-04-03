"""
Lab 5 - Localization, Path Planning, and Navigation
Author: Griffin Martin & Poorvaja Veera Balaji Kumar
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.subscription = self.create_subscription(
            NavigateToPose_FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self.feedback_callback,
            10
        )

        self.waypoints = [
            (0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            (1.5, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0),
            (0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        ]

        self.current_waypoint_index = 0
        self.distance_threshold = 0.2
        self.goal_sent = False

        self.timer = self.create_timer(1.0, self.timer_callback)

    def send_goal(self, waypoint):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = waypoint[0]
        msg.pose.position.y = waypoint[1]
        msg.pose.position.z = waypoint[2]

        msg.pose.orientation.x = waypoint[3]
        msg.pose.orientation.y = waypoint[4]
        msg.pose.orientation.z = waypoint[5]
        msg.pose.orientation.w = waypoint[6]

        self.publisher.publish(msg)
        self.get_logger().info(f'Sent waypoint {self.current_waypoint_index + 1}: {waypoint[:2]}')
        self.goal_sent = True

    def feedback_callback(self, msg):
        distance = msg.feedback.distance_remaining
        if distance:
            self.get_logger().info(f'Distance remaining: {distance:.2f}m')
        else:
            return

        if distance < self.distance_threshold:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached!')
            self.current_waypoint_index += 1
            self.goal_sent = False

    def timer_callback(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!')
            self.timer.cancel()
            return

        if not self.goal_sent:
            self.send_goal(self.waypoints[self.current_waypoint_index])

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()