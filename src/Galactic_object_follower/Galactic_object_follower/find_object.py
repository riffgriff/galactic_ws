import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point

BLUE_MAX = np.array([115,255,255])
BLUE_MIN = np.array([90,120,40])


class find_object(Node):
    def __init__(self):
        super().__init__('find_object_node')

        # TODO: check if qos_profile is right
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        self._img_subscriber = self.create_subscription(CompressedImage, '/image_raw/compressed', self._image_callback, qos_profile)
        self._point_publish = self.create_publisher(Point, '/galactic_object_follower/object_coords', qos_profile)
        self._img_publish = self.create_publisher(CompressedImage, '/find_object/bounding_box', qos_profile)

    def _image_callback(self, msg: CompressedImage):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return

        # flip = cv2.flip(img, 1)
        blur = cv2.GaussianBlur(img, (15, 15), 100)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        thresholded = cv2.inRange(hsv, BLUE_MIN, BLUE_MAX)

        try:
            nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(thresholded, connectivity=4)
            sizes = stats[:, -1]
            max_label = 1
            max_size = sizes[max_label]
            for i in range(2, nb_components):
                if sizes[i] > max_size:
                    max_label = i
                    max_size = sizes[i]

            biggest_component = np.zeros(output.shape, dtype=np.uint8)
            biggest_component[output == max_label] = 255

            big_component_points = cv2.findNonZero(biggest_component)

            x, y, w, h = cv2.boundingRect(big_component_points)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)  # Red box with thickness 2
            point_x = x + w // 2
            point_y = y + h // 2

            # publish detected point as geometry_msgs/Point (image pixel coordinates)
            p = Point(x=float(point_x), y=float(point_y), z=float(0.0))
            self._point_publish.publish(p)

            ok, buf = cv2.imencode('.jpg', img)
            if ok:
                cim = CompressedImage()
                cim.format = 'jpeg'
                cim.data = np.array(buf).tobytes()
                self._img_publish.publish(cim)

        except (IndexError, cv2.error):
			# negative Z for the case with no object
            p = Point(x=0.0, y=0.0, z=-1.0)
			self._point_publish.publish(p)

        
def main(args=None):
    rclpy.init(args=args)
    node = find_object()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
	main()
