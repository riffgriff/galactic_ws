"""
Lab 4 - Navigate To Goal
Author: Griffin Martin & Poorvaja Veera Balaji Kumar
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from dataclasses import dataclass
import math


class DriveCarefully(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        #region parameter handling
        self.declare_parameter('publish_frequency', 5) # in Hz
        self.declare_parameter('avoidance_dist_threshold', 0.2) # in meters
        self.declare_parameter('avoidance_angular_threshold', 15) # in degrees
        self.declare_parameter('pivot_threshold', 5) # in degrees
        self.declare_parameter('avoidance_speed', 0.05) # m/s
        self.declare_parameter('max_speed_r', 1.5)
        self.declare_parameter('max_speed_t', 0.1)
        # rotational PID constants
        self.declare_parameter('kp_r', 1.8)
        self.declare_parameter('ki_r', 0.0)
        self.declare_parameter('kd_r', 0.0)
        # translational PID constants
        self.declare_parameter('kp_t', 0.5)
        self.declare_parameter('ki_t', 0.0)
        self.declare_parameter('kd_t', 0.0)
        # object avoidance PID constants
        self.declare_parameter('kp_obj', 2.0)
        self.declare_parameter('ki_obj', 0.0)
        self.declare_parameter('kd_obj', 0.0)
        # angle pivoting PID constants
        self.declare_parameter('kp_pivot', 2.0)
        self.declare_parameter('ki_pivot', 0.0)
        self.declare_parameter('kd_pivot', 0.0)

        # store parameters for computing speed
        kp_r = self.get_parameter('kp_r').value
        ki_r = self.get_parameter('ki_r').value
        kd_r = self.get_parameter('kd_r').value
        kp_t = self.get_parameter('kp_t').value
        ki_t = self.get_parameter('ki_t').value
        kd_t = self.get_parameter('kd_t').value
        kp_obj = self.get_parameter('kp_obj').value
        ki_obj = self.get_parameter('ki_obj').value
        kd_obj = self.get_parameter('kd_obj').value
        kp_pivot = self.get_parameter('kp_pivot').value
        ki_pivot = self.get_parameter('ki_pivot').value
        kd_pivot = self.get_parameter('kd_pivot').value
        
        publish_period = 1/self.get_parameter('publish_frequency').value
        self.setpoint_t = self.get_parameter('distance_setpoint').value
        self.max_speed_r = self.get_parameter('max_speed_r').value
        self.max_speed_t = self.get_parameter('max_speed_t').value
        self.avoidance_dist_threshold = self.get_parameter('avoidance_dist_threshold').value
        self.avoidance_angular_threshold = math.pi*self.get_parameter('avoidance_angular_threshold').value/180.0 # in radians
        self.avoidance_speed = self.get_parameter('avoidance_speed').value
        self.pivot_threshold = self.get_parameter('pivot_threshold').value
        #endregion

        # set up PID controllers
        self.direct_controller_r = PID_controller(kp_r, ki_r, kd_r, publish_period)
        self.direct_controller_t = PID_controller(kp_t, ki_t, kd_t, publish_period)
        self.avoidance_controller = PID_controller(kp_obj, ki_obj, kd_obj, publish_period)
        self.pivot_controller = PID_controller(kp_pivot, ki_pivot, kd_pivot, publish_period)

        # set up QOS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        # set up publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(publish_period, self.publisher_callback)
        
        # set up goal subscriber
        self.goal_x = None
        self.goal_y = None
        self.goal_subscription = self.create_subscription(Vector3, '/goal_pos', self.goal_callback, qos_profile)

        # set up range subscriber
        self.obj_x = None
        self.obj_y = None
        self.range_subscription = self.create_subscription(Vector3, '/obj_pos', self.range_callback, qos_profile)

        # set up state machine
        self.near_object = False
        self.pivoting = False
        self.object_on_left = True
        self.cycles_without_lidar = 0

    def goal_callback(self, msg):
        if msg.z != 0:
            self.goal_x = None
            self.goal_y = None
        else:
            self.goal_x = msg.x
            self.goal_y = msg.y

    def range_callback(self, msg):
        if msg.z != 0:
            self.obj_x = None
            self.obj_y = None
        else:
            self.obj_x = msg.x
            self.obj_y = msg.y

    def publisher_callback(self):
        msg = Twist()
        
        if self.goal_x is None or self.goal_y is None:
            # shut it down if there's no goal given
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            self.publisher_.publish(msg)
            return
        
        if self.obj_x is None or self.obj_y is None:
            # allow three cycles with no lidar until e-stop
            self.cycles_without_lidar += 1
            if self.cycles_without_lidar >= 3:
                msg.angular.z = 0.0
                msg.linear.x = 0.0
                self.publisher_.publish(msg)
            # if it hasn't been three cycles, do not publish - robot maintains course
            return
        else:
            self.cycles_without_lidar = 0

        goal_r, goal_ang = cart_to_polar(self.goal_x, self.goal_y)
        obj_r, obj_ang = cart_to_polar(self.obj_x, self.obj_y)
        angle_effort = 0
        dist_effort = 0

        ### state machine ###

        # case 1 - recently pivoted - check if safe, continue to go around if not
        if self.near_object:

            # check if safe - goal is in a window from avoidance_angular_threshold to 90 degrees outward of object
            if (self.object_on_left and goal_ang < -self.avoidance_angular_threshold and goal_ang > -math.pi/2) or \
               (self.object_on_right and goal_ang > self.avoidance_angular_threshold and goal_ang < math.pi/2):
                # clear out the old terms from the direct controller
                self.direct_controller_r.reset()
                self.direct_controller_t.reset()
                self.near_object = False
            else:
                # side of object decides sign of PID
                PID_sign = 1
                if self.object_on_left: PID_sign = -1

                # PID to stay a certain distance from the object
                err = self.avoidance_dist_threshold - obj_r
                angle_effort = PID_sign*self.avoidance_controller.get_effort(err)
                dist_effort = self.avoidance_speed

        
        # case 2 - recently encountered object - check if pivoted enough, continue pivoting if not
        elif self.pivoting:
            ang_err = 0
            if self.object_on_left:
               ang_err = math.pi/2 - obj_ang
            else:
               ang_err = -math.pi/2 - obj_ang
            
            if abs(ang_err) < self.pivot_threshold:
                # close enough to perpendicular - initiate the near-object avoidance
                self.near_object = True
                self.pivoting = False
                self.avoidance_controller.reset()
            else:
                # keep pivoting
                angle_effort = self.pivot_controller.get_effort(ang_err)
                dist_effort = 0

        # case 3 - newly detected object in driving field - start pivoting to perpendicular the LIDAR vector
        # add some logic here to figure out which objects in front of the robot we care about? What defines in front of?
        elif obj_r <= self.avoidance_dist_threshold: # ADD SAUCIER LOGIC HERE
            self.pivoting = True
            self.object_on_left = obj_ang > 0
            # clear out the pivot controller
            self.pivot_controller.reset()

        # case 4 - no object close enough, proceed directly to goal
        else:
            angle_effort = self.direct_controller_r.get_effort(goal_ang)
            dist_effort = self.direct_controller_t.get_effort(goal_r)
        
        ### end of state machine ###

        # bound outputs for all cases
        if abs(dist_effort) > self.max_speed_t:
            msg.linear.x = math.copysign(self.max_speed_t, dist_effort)
        else:
            msg.linear.x = dist_effort

        if abs(angle_effort) > self.max_speed_r:
            msg.angular.z = math.copysign(self.max_speed_r, angle_effort)
        else:
            msg.angular.z = angle_effort

        # publish the message no matter what (except for bad subscriptions)
        self.publisher_.publish(msg)


### helper classes/functions ###

@dataclass
class PID_controller:
    # implements a PID controller with a fixed period
    kp: float
    ki: float
    kd: float
    period: float

    def __post_init__(self):
        self.reset()

    def get_effort(self, error):
        self.error_sum += error
        output = -self.kp*error - self.ki*self.error_sum*self.period - self.kd*(error-self.prev_error)/self.period
        self.prev_error = error
        return output
    
    def reset(self):
        self.error_sum = 0.0
        self.prev_error = 0.0
    
def cart_to_polar(x, y):
    # convert cartesian coordinates to polar coordinates
    r = math.sqrt(pow(x,2) + pow(y,2))
    theta = math.atan2(y, x)
    return r, theta

def main(args=None):
    rclpy.init(args=args)

    node = DriveCarefully()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
