#!/usr/bin/env python3
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan

try:
	import img_preprocessing
except Exception:
	img_preprocessing = None


LABEL_EMPTY = 0
LABEL_LEFT = 1
LABEL_RIGHT = 2
LABEL_DO_NOT_ENTER = 3
LABEL_STOP = 4
LABEL_GOAL = 5


def wrap_angle(angle):
	while angle > math.pi:
		angle -= 2.0 * math.pi
	while angle < -math.pi:
		angle += 2.0 * math.pi
	return angle


def quat_to_yaw(x, y, z, w):
	siny_cosp = 2.0 * (w * z + x * y)
	cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
	return math.atan2(siny_cosp, cosy_cosp)


class MazeNavigator(Node):
	def __init__(self):
		super().__init__('navigate_maze_node')

		# Parameters
		self.declare_parameter('model_path', str(Path(__file__).resolve().parent / 'model.xml'))
		self.declare_parameter('knn_k', 7)
		self.declare_parameter('linear_speed', 0.10)
		self.declare_parameter('angular_speed', 0.8)
		self.declare_parameter('wall_stop_dist', 0.28)
		self.declare_parameter('wall_observe_max', 0.45)
		self.declare_parameter('turn_tolerance_deg', 5.0)
		self.declare_parameter('search_timeout_s', 8.0)

		self.model_path = self.get_parameter('model_path').value
		self.knn_k = int(self.get_parameter('knn_k').value)
		self.linear_speed = float(self.get_parameter('linear_speed').value)
		self.angular_speed = float(self.get_parameter('angular_speed').value)
		self.wall_stop_dist = float(self.get_parameter('wall_stop_dist').value)
		self.wall_observe_max = float(self.get_parameter('wall_observe_max').value)
		self.turn_tolerance = math.radians(float(self.get_parameter('turn_tolerance_deg').value))
		self.search_timeout = float(self.get_parameter('search_timeout_s').value)

		model = Path(self.model_path)
		if not model.is_absolute():
			model = (Path(__file__).resolve().parent / model).resolve()
		if not model.exists():
			raise FileNotFoundError(
				f'KNN model not found: {model}. Pass --ros-args -p model_path:=<path_to_model.xml>'
			)

		self.get_logger().info(f'Loading KNN model: {model}')
		self.knn = cv2.ml.KNearest_load(str(model))

		sensor_qos = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			durability=QoSDurabilityPolicy.VOLATILE,
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=1,
		)

		self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
		self.state_pub = self.create_publisher(String, '/drive_state', 10)
		self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
		self.img_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, sensor_qos)
		self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, sensor_qos)

		self.timer = self.create_timer(0.1, self.control_loop)

		self.latest_scan = None
		self.latest_image = None
		self.current_yaw = None

		self.state = 'DRIVE_TO_WALL'
		self.turn_target_yaw = None
		self.turn_rate_sign = 1.0
		self.search_start_time = None

		self.get_logger().info('navigate_maze node started')

	def scan_callback(self, msg):
		self.latest_scan = msg

	def image_callback(self, msg):
		arr = np.frombuffer(msg.data, dtype=np.uint8)
		img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
		if img is not None:
			self.latest_image = img

	def odom_callback(self, msg):
		q = msg.pose.pose.orientation
		self.current_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

	def publish_cmd(self, linear_x=0.0, angular_z=0.0):
		cmd = Twist()
		cmd.linear.x = float(linear_x)
		cmd.angular.z = float(angular_z)
		self.cmd_pub.publish(cmd)

	def min_range_around_angle(self, angle_rad, half_width_deg=12):
		if self.latest_scan is None:
			return float('inf')

		scan = self.latest_scan
		half_width = math.radians(half_width_deg)
		mins = []
		for i, rng in enumerate(scan.ranges):
			if not math.isfinite(rng):
				continue
			beam_ang = scan.angle_min + i * scan.angle_increment
			err = wrap_angle(beam_ang - angle_rad)
			if abs(err) <= half_width:
				mins.append(rng)

		return min(mins) if mins else float('inf')

	def front_distance(self):
		return self.min_range_around_angle(0.0, half_width_deg=12)

	def classify_sign(self):
		if self.latest_image is None:
			return None

		img = self.latest_image

		if img_preprocessing is not None:
			try:
				x, y, w, h, _ = img_preprocessing.get_bounding_box(img)
				if img_preprocessing.is_reasonable_box(w,h):
					img = img[y:y + h, x:x + w]
			except Exception:
				pass

		input_size = (25, 33)
		resized = cv2.resize(img, input_size)
		sample = resized.flatten().reshape(1, input_size[0] * input_size[1] * 3).astype(np.float32)
		ret, _, _, _ = self.knn.findNearest(sample, int(self.knn_k))
		return int(ret)

	def start_turn(self, delta_rad):
		if self.current_yaw is not None:
			self.turn_target_yaw = wrap_angle(self.current_yaw + delta_rad)
		else:
			self.turn_target_yaw = None

		self.turn_rate_sign = 1.0 if delta_rad >= 0.0 else -1.0
		self.state = 'TURNING'

	def control_loop(self):
		if self.latest_scan is None:
			self.publish_cmd(0.0, 0.0)
			return

		self.state_pub.publish(String(self.state))

		d_front = self.front_distance()

		if self.state == 'DONE':
			self.publish_cmd(0.0, 0.0)
			return

		if self.state == 'DRIVE_TO_WALL':
			if d_front <= self.wall_stop_dist:
				self.publish_cmd(0.0, 0.0)
				self.state = 'CLASSIFY'
			else:
				self.publish_cmd(self.linear_speed, 0.0)
			return

		if self.state == 'CLASSIFY':
			self.publish_cmd(0.0, 0.0)

			if d_front > self.wall_observe_max:
				self.state = 'DRIVE_TO_WALL'
				return

			label = self.classify_sign()
			if label is None:
				self.state = 'SEARCH'
				self.search_start_time = time.time()
				return

			if label == LABEL_GOAL:
				self.get_logger().info('GOAL sign detected. Stopping.')
				self.state = 'DONE'
				return

			if label == LABEL_LEFT:
				self.start_turn(math.pi / 2.0)
				return

			if label == LABEL_RIGHT:
				self.start_turn(-math.pi / 2.0)
				return

			if label in (LABEL_DO_NOT_ENTER, LABEL_STOP):
				self.start_turn(math.pi)
				return

			# Empty / unknown
			self.state = 'SEARCH'
			self.search_start_time = time.time()
			return

		if self.state == 'SEARCH':
			self.publish_cmd(0.0, 0.5)

			label = self.classify_sign()
			if label == LABEL_GOAL:
				self.state = 'DONE'
				return
			if label == LABEL_LEFT:
				self.start_turn(math.pi / 2.0)
				return
			if label == LABEL_RIGHT:
				self.start_turn(-math.pi / 2.0)
				return
			if label in (LABEL_DO_NOT_ENTER, LABEL_STOP):
				self.start_turn(math.pi)
				return

			if self.search_start_time is not None and (time.time() - self.search_start_time) > self.search_timeout:
				# fallback behavior if no useful sign found
				self.start_turn(math.pi)
			return

		if self.state == 'TURNING':
			# Prefer odom-based turn completion if odom is available
			if self.current_yaw is not None and self.turn_target_yaw is not None:
				err = wrap_angle(self.turn_target_yaw - self.current_yaw)
				if abs(err) <= self.turn_tolerance:
					self.publish_cmd(0.0, 0.0)
					self.state = 'DRIVE_TO_WALL'
				else:
					direction = 1.0 if err > 0.0 else -1.0
					self.publish_cmd(0.0, direction * self.angular_speed)
			else:
				# odom unavailable fallback: keep turning slowly until wall is seen again
				self.publish_cmd(0.0, self.turn_rate_sign * self.angular_speed)
				if d_front <= self.wall_observe_max:
					self.publish_cmd(0.0, 0.0)
					self.state = 'DRIVE_TO_WALL'
			return


def main(args=None):
	rclpy.init(args=args)
	node = MazeNavigator()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.publish_cmd(0.0, 0.0)
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
