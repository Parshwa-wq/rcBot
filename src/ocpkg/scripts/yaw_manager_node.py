#!/usr/bin/python3
"""
Yaw Manager Node for Mecanum Drive
==================================
[MASTER CONTROL MODE]

It converts IMU orientation into /imu_deg and owns heading lock behavior:
- hold the startup yaw until the operator commands rotation
- unlock while the right stick rotation axis is active
- relock when stick is released
- handle 90-degree step rotation requests
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

class YawManagerNode(Node):
    def __init__(self):
        super().__init__('yaw_manager_node')

        # Parameters
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('imu_yaw_offset', 0.0)
        self.declare_parameter('angular_deadzone', 0.01)
        self.declare_parameter('yaw_kp', 3.0)
        self.declare_parameter('yaw_ki', 0.5)
        self.declare_parameter('yaw_kd', 0.25)
        self.declare_parameter('yaw_output_limit', 3.0)
        self.declare_parameter('yaw_min_output', 0.0)
        self.declare_parameter('yaw_correction_sign', 1.0)
        self.declare_parameter('yaw_hold_deadband_deg', 0.8)
        self.declare_parameter('rotation_kp', 2.8)
        self.declare_parameter('rotation_kd', 0.12)
        self.declare_parameter('rotation_speed_limit', 2.0)
        self.declare_parameter('rotation_tolerance_deg', 1.0)
        self.declare_parameter('rotation_slowdown_deg', 10.0)
        self.declare_parameter('rotation_timeout', 999.0)
        self.declare_parameter('imu_timeout', 0.5)

        self.control_freq = self.get_parameter('control_frequency').value
        self.imu_yaw_offset = self.get_parameter('imu_yaw_offset').value
        self.angular_deadzone = self.get_parameter('angular_deadzone').value
        self.yaw_kp = self.get_parameter('yaw_kp').value
        self.yaw_ki = self.get_parameter('yaw_ki').value
        self.yaw_kd = self.get_parameter('yaw_kd').value
        self.yaw_output_limit = self.get_parameter('yaw_output_limit').value
        self.yaw_min_output = self.get_parameter('yaw_min_output').value
        self.yaw_correction_sign = self.get_parameter('yaw_correction_sign').value
        self.yaw_hold_deadband = math.radians(self.get_parameter('yaw_hold_deadband_deg').value)
        self.rotation_kp = self.get_parameter('rotation_kp').value
        self.rotation_kd = self.get_parameter('rotation_kd').value
        self.rotation_speed_limit = self.get_parameter('rotation_speed_limit').value
        self.rotation_tolerance = math.radians(self.get_parameter('rotation_tolerance_deg').value)
        self.rotation_slowdown = math.radians(self.get_parameter('rotation_slowdown_deg').value)
        self.rotation_timeout = self.get_parameter('rotation_timeout').value
        self.imu_timeout = self.get_parameter('imu_timeout').value

        # State
        self.imu_active = False
        self.last_imu_time = self.get_clock().now()
        self.current_yaw = 0.0
        self.current_angular_velocity = 0.0
        self.locked_yaw = None
        self.raw_cmd = Twist()
        self.manual_rotation_active = False
        self.auto_rotating = False
        self.target_yaw = 0.0
        self.rotation_start_time = None
        self.prev_rotation_error = 0.0
        self.prev_yaw_error = 0.0
        self.yaw_integral = 0.0
        self.prev_time = self.get_clock().now()

        # Use BEST_EFFORT QoS for real-time control to minimize lag
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.imu_deg_pub = self.create_publisher(Float32, '/imu_deg', qos)

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_callback, qos)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, qos)
        self.create_subscription(Float32, '/rotation_request', self.rotation_request_callback, qos)

        self.timer = self.create_timer(1.0 / self.control_freq, self.control_loop)
        self.get_logger().info('MASTER CONTROL MODE: Yaw Manager ENABLED')

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def quaternion_to_yaw(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def imu_callback(self, msg: Imu):
        self.imu_active = True
        self.last_imu_time = self.get_clock().now()
        self.current_yaw = self.quaternion_to_yaw(msg.orientation)
        self.current_angular_velocity = msg.angular_velocity.z
        
        deg_msg = Float32()
        deg_msg.data = math.degrees(self.current_yaw)
        self.imu_deg_pub.publish(deg_msg)

    def cmd_callback(self, msg: Twist):
        self.raw_cmd = msg

    def rotation_request_callback(self, msg: Float32):
        if not self.imu_active: return
        self.target_yaw = self.normalize_angle(self.current_yaw + math.radians(msg.data))
        self.locked_yaw = self.target_yaw
        self.auto_rotating = True
        self.rotation_start_time = self.get_clock().now()
        self.prev_rotation_error = self.normalize_angle(self.target_yaw - self.current_yaw)
        self.reset_yaw_pid()

    def reset_yaw_pid(self):
        self.yaw_integral = 0.0
        self.prev_yaw_error = 0.0

    def lock_current_yaw(self):
        self.locked_yaw = self.current_yaw
        self.reset_yaw_pid()

    def clamp(self, val, limit):
        return max(-limit, min(limit, val))

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now
        if dt <= 0.0: return

        imu_ok = self.imu_active and (now - self.last_imu_time).nanoseconds / 1e9 <= self.imu_timeout

        out = Twist()
        out.linear.x = self.raw_cmd.linear.x
        out.linear.y = self.raw_cmd.linear.y

        if not imu_ok:
            out.angular.z = self.raw_cmd.angular.z
            self.cmd_pub.publish(out)
            return

        manual_rotation = abs(self.raw_cmd.angular.z) > self.angular_deadzone
        
        if manual_rotation:
            self.auto_rotating = False
            self.manual_rotation_active = True
            self.reset_yaw_pid() # <--- CRITICAL: Reset memory while manually turning
            out.angular.z = self.raw_cmd.angular.z
            self.cmd_pub.publish(out)
            return

        if self.manual_rotation_active:
            # Wait for physical momentum to stop before locking new heading
            if abs(self.current_angular_velocity) < 0.05:
                self.manual_rotation_active = False
                self.lock_current_yaw()
            else:
                out.angular.z = 0.0
                self.cmd_pub.publish(out)
                return

        if self.auto_rotating:
            error = self.normalize_angle(self.target_yaw - self.current_yaw)
            if abs(error) <= self.rotation_tolerance:
                self.auto_rotating = False
                self.lock_current_yaw()
            else:
                deriv = -self.current_angular_velocity
                cmd = self.rotation_kp * error + self.rotation_kd * deriv
                slowdown = max(0.4, min(1.0, abs(error) / self.rotation_slowdown))
                out.angular.z = self.clamp(cmd, self.rotation_speed_limit * slowdown)
                self.prev_rotation_error = error
            self.cmd_pub.publish(out)
            return

        # Heading Hold Logic
        if self.locked_yaw is None: self.lock_current_yaw()
        error = self.normalize_angle(self.locked_yaw - self.current_yaw)
        
        # Smooth deadband mapping
        if abs(error) <= self.yaw_hold_deadband:
            active_error = 0.0
            self.yaw_integral *= 0.95  # Slowly decay integral term in deadband
        else:
            active_error = error - math.copysign(self.yaw_hold_deadband, error)
            self.yaw_integral = self.clamp(self.yaw_integral + active_error * dt, 1.0)

        deriv = -self.current_angular_velocity
        correction = (self.yaw_kp * active_error + self.yaw_ki * self.yaw_integral + self.yaw_kd * deriv) * self.yaw_correction_sign
        self.prev_yaw_error = error
        out.angular.z = self.clamp(correction, self.yaw_output_limit)

        self.cmd_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = YawManagerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
