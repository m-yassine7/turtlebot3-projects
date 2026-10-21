# ECE 7425, Assignment 3 - AprilTag Follower

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformException


class TagFollower(Node):
    def __init__(self):
        super().__init__('tag_follower')

        # read settings from controller.yaml
        self.target_id = self.setting('target_id', 0)
        self.target_frame = self.setting('target_frame', 'target_tag')
        self.base_frame = self.setting('base_frame', 'base_link')
        self.target_distance = self.setting('target_distance', 0.50)
        self.near_distance = self.setting('near_distance', 0.45)
        self.far_distance = self.setting('far_distance', 0.55)
        self.align_tolerance_deg = self.setting('align_tolerance_deg', 5.0)
        self.distance_gain = self.setting('distance_gain', 0.6)
        self.angular_gain = self.setting('angular_gain', 1.4)
        self.max_linear_speed = self.setting('max_linear_speed', 0.10)
        self.max_angular_speed = self.setting('max_angular_speed', 0.40)
        self.search_speed = self.setting('search_speed', 0.20)
        self.search_timeout = self.setting('search_timeout', 10.0)
        self.lost_timeout = self.setting('lost_timeout', 0.50)
        self.scan_timeout = self.setting('scan_timeout', 0.50)
        self.obstacle_distance = self.setting('obstacle_distance', 0.25)
        self.front_angle_deg = self.setting('front_angle_deg', 20.0)

        # Store the current robot state and sensor information
        self.enabled = False
        self.state = 'DISABLED'
        self.seen_tag = False
        self.tag_time = None
        self.scan_time = None
        self.scan_clear = False
        self.camera_frame = ''
        self.search_start = None

        # Publish velocity cmds and the current controller state
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/matador/state', 10)
        
        # Receive AprilTag detections and LiDAR measurements
        self.create_subscription(AprilTagDetectionArray, '/detections', self.on_tags, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        
        # Allow the operator to enable or stop the controller
        self.create_service(SetBool, '/matador/enable', self.on_enable)
        
        # Use TF to find the tag's position relative to the robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Run the movement controller every 0.1 seconds
        self.create_timer(0.1, self.control_loop)

    def setting(self, name, default):
        return self.declare_parameter(name, default).value

    def move(self, forward, turn):
        command = Twist()
        command.linear.x = float(forward)
        command.angular.z = float(turn)
        self.velocity_pub.publish(command)

    def stop(self, reason):
        self.enabled = False
        self.state = reason
        self.move(0.0, 0.0)

    def scan_is_safe(self):
        return (self.scan_time is not None and self.scan_clear and
                time.monotonic() - self.scan_time <= self.scan_timeout)

    def on_enable(self, request, response):
        if not request.data:
            self.stop('SAFETY_STOP')
            response.success = True
            response.message = 'Controller stopped'
        elif not self.scan_is_safe():
            self.stop('SAFETY_STOP')
            response.success = False
            response.message = 'LiDAR data missing or obstacle ahead'
        else:
            self.enabled = True
            self.state = 'SEARCH'
            self.seen_tag = False
            self.tag_time = None
            self.search_start = time.monotonic()
            response.success = True
            response.message = 'Searching for AprilTag'
        return response

    def on_tags(self, message):
        if not self.enabled:
            return
        for tag in message.detections:
            if tag.id == self.target_id:
                self.seen_tag = True
                self.tag_time = time.monotonic()
                self.camera_frame = message.header.frame_id
                return
        # stop if a tag we were following disappears
        if self.seen_tag:
            self.stop('LOST_TARGET')

    def on_scan(self, message):
        self.scan_time = time.monotonic()
        self.scan_clear = False
        if (not math.isfinite(message.angle_min) or
                not math.isfinite(message.angle_increment) or
                message.angle_increment == 0 or
                not math.isfinite(message.range_min) or
                not math.isfinite(message.range_max) or
                message.range_max <= message.range_min):
            if self.enabled:
                self.stop('SAFETY_STOP')
            return

        distances = []
        for i, distance in enumerate(message.ranges):
            angle = message.angle_min + i * message.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= math.radians(self.front_angle_deg):
                if distance == float('inf'):
                    distances.append(distance)  # No obstacle return
                elif math.isfinite(distance) and message.range_min <= distance <= message.range_max:
                    distances.append(distance)
                else:
                    # invalid front reading: stop instead of guessing
                    if self.enabled:
                        self.stop('SAFETY_STOP')
                    return

        self.scan_clear = bool(distances) and min(distances) >= self.obstacle_distance
        if self.enabled and not self.scan_clear:
            self.stop('SAFETY_STOP')

    def read_tag(self):
        if not self.camera_frame:
            raise ValueError('Camera frame missing')
        
        # Get the tag's position relative to the robot and camera
        robot_tf = self.tf_buffer.lookup_transform(self.base_frame, self.target_frame, Time())
        camera_tf = self.tf_buffer.lookup_transform(self.camera_frame, self.target_frame, Time())

        # Reject missing/old/invalid poses
        for transform in (robot_tf, camera_tf):
            stamp = Time.from_msg(transform.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if age > self.lost_timeout or age < -0.2:
                raise ValueError('Old tag transform')

        x = robot_tf.transform.translation.x
        y = robot_tf.transform.translation.y
        point = camera_tf.transform.translation
        if not all(math.isfinite(v) for v in (x, y, point.x, point.y, point.z)):
            raise ValueError('Invalid tag position')

        # Calculate the direction to the tag and its distance from the camera
        bearing = math.atan2(y, x)
        distance = math.sqrt(point.x**2 + point.y**2 + point.z**2)
        if distance <= 0:
            raise ValueError('Invalid distance')
        return distance, bearing

    def control_loop(self):
        # Publish current state of robot
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        # Stop if the robot is disabled
        if not self.enabled:
            self.move(0.0, 0.0)
            return
        
        # Stop if LiDAR data is missing/outdated/unsafe
        if not self.scan_is_safe():
            self.stop('SAFETY_STOP')
            return
        
        # Stop if AprilTag detection is too old
        if self.seen_tag and time.monotonic() - self.tag_time > self.lost_timeout:
            self.stop('LOST_TARGET')
            return

        # Search for tag by rotating for up to 10 sec
        if self.state == 'SEARCH':
            if time.monotonic() - self.search_start >= self.search_timeout:
                self.stop('SEARCH_TIMEOUT')
                return
            if not self.seen_tag:
                self.move(0.0, self.search_speed)
                return

        # Get distance and direction of the detected tag
        try:
            distance, bearing = self.read_tag()
        except (TransformException, ValueError) as error:
            self.get_logger().warn(str(error))
            self.stop('SAFETY_STOP')
            return

        # Stop when tag is too close (less than 0.45 m)
        if distance < self.near_distance:
            self.state = 'HOLD'
            self.move(0.0, 0.0)
        
        # Hold position when the tag is between 0.45 m and 0.55 m
        elif distance <= self.far_distance:
            self.state = 'HOLD'
            self.move(0.0, 0.0)

        else:
            # Calculate turning speed using proportional control
            turn = self.angular_gain * bearing

            # Limit the turning speed
            turn = max(-self.max_angular_speed, min(turn, self.max_angular_speed))
           
            # Rotate until tag is within 5 degrees of center
            if abs(bearing) > math.radians(self.align_tolerance_deg):
                self.state = 'ALIGN'
                self.move(0.0, turn)

            else:
                # Move toward the tag using proportional distance control
                self.state = 'APPROACH'
                error = distance - self.target_distance
                forward = self.distance_gain * error

                # Limit the forward speed
                forward = max(0.0, min(forward, self.max_linear_speed))
                self.move(forward, turn)


def main(args=None):
    rclpy.init(args=args)
    node = TagFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.move(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    