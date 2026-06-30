#!/usr/bin/python3
"""
PID Controller Node for Mecanum Drive
======================================
[MASTER CONTROL MODE]

Handles velocity control, yaw correction, and odometry calculation.
Features:
- Omni inverse/forward kinematics
- Per-wheel velocity PID with Feed-Forward
- Automatic Deadband compensation
"""

import math
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool
from nav_msgs.msg import Odometry
import tf2_ros

from ocpkg.msg import WheelDistances


class PIDController:
    """Basic PID controller class with integral limiting."""

    def __init__(self, kp: float, ki: float, kd: float, max_integral: float, output_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.output_limit = output_limit

        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        i_term = self.ki * self.integral

        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt

        self.prev_error = error

        output = p_term + i_term + d_term
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class OmniKinematics:
    """
    Omni wheel kinematics calculations.
    Matches User Hardware: rotateLeft signs (1, 1, 1, 1)
    """

    def __init__(self, wheel_radius: float, wheel_base: float, wheel_track: float):
        self.r = wheel_radius
        self.lx = wheel_base / 2.0
        self.ly = wheel_track / 2.0
        self.L = self.lx + self.ly

        # Inverse Kinematics Matrix (MATCHING USER HARDWARE)
        self.inv_kinematics = np.array([
            [-1,  1,  self.L],  # FL
            [ 1,  1,  self.L],  # FR
            [-1, -1,  self.L],  # BL
            [ 1, -1,  self.L],  # BR
        ]) / self.r

        self.fwd_kinematics = np.linalg.pinv(self.inv_kinematics)

    def inverse(self, vx: float, vy: float, wz: float) -> np.ndarray:
        body_vel = np.array([vx, vy, wz])
        return self.inv_kinematics @ body_vel

    def forward(self, wheel_vels: np.ndarray) -> Tuple[float, float, float]:
        body_vel = self.fwd_kinematics @ wheel_vels
        return body_vel[0], body_vel[1], body_vel[2]


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Parameters
        self.declare_parameter('wheel_radius', 0.11)
        self.declare_parameter('wheel_base', 0.2175)
        self.declare_parameter('wheel_track', 0.2175)
        self.declare_parameter('encoder_ticks_per_rev', 1620)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('max_wheel_velocity', 6.0)
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('velocity_deadzone', 0.01)
        self.declare_parameter('angular_deadzone', 0.01)
        self.declare_parameter('motor_deadband_offset', 0.85)
        self.declare_parameter('max_accel_linear', 3.0)
        self.declare_parameter('max_accel_angular', 5.0)
        self.declare_parameter('max_decel_linear', 50.0)
        self.declare_parameter('max_decel_angular', 50.0)
        self.declare_parameter('wheel_pid_target_deadzone', 0.05)
        
        # Encoder Direction
        self.declare_parameter('encoder_direction.front_left', 1)
        self.declare_parameter('encoder_direction.front_right', 1)
        self.declare_parameter('encoder_direction.back_left', 1)
        self.declare_parameter('encoder_direction.back_right', 1)
        
        # Directional trim scaling parameters
        self.declare_parameter('trim_forward', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('trim_backward', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('trim_left', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('trim_right', [1.0, 1.0, 1.0, 1.0])
        # Front-Left diagonal trim (vx>0, vy>0) — activates FR (index 1) and BL (index 2)
        # FR spins anti-clockwise from motor POV; BL spins clockwise from motor POV
        self.declare_parameter('trim_front_left', [1.0, 1.0, 1.0, 1.0])
        # Front-Right diagonal trim (vx>0, vy<0) — activates FL (index 0) and BR (index 3)
        # FL spins clockwise from motor POV; BR spins anti-clockwise from motor POV
        self.declare_parameter('trim_front_right', [1.0, 1.0, 1.0, 1.0])
        
        # Wheel PID Params
        self.declare_parameter('wheel_velocity_pid.kp', 0.1)
        self.declare_parameter('wheel_velocity_pid.ki', 0.0)
        self.declare_parameter('wheel_velocity_pid.kd', 0.0)
        self.declare_parameter('wheel_velocity_pid.max_integral', 0.2)
        self.declare_parameter('wheel_velocity_pid.output_limit', 6.0)
        
        self.declare_parameter('assembly_fixed_wheel_speed', 0.3)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_track = self.get_parameter('wheel_track').value
        self.ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.max_wheel_vel = self.get_parameter('max_wheel_velocity').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.vel_deadzone = self.get_parameter('velocity_deadzone').value
        self.ang_deadzone = self.get_parameter('angular_deadzone').value
        self.deadband_offset = self.get_parameter('motor_deadband_offset').value
        self.max_accel_linear = self.get_parameter('max_accel_linear').value
        self.max_accel_angular = self.get_parameter('max_accel_angular').value
        self.max_decel_linear = self.get_parameter('max_decel_linear').value
        self.max_decel_angular = self.get_parameter('max_decel_angular').value
        self.wheel_pid_target_deadzone = self.get_parameter('wheel_pid_target_deadzone').value

        self.enc_dirs = [
            self.get_parameter('encoder_direction.front_left').value,
            self.get_parameter('encoder_direction.front_right').value,
            self.get_parameter('encoder_direction.back_left').value,
            self.get_parameter('encoder_direction.back_right').value,
        ]

        self.trim_forward = self.get_parameter('trim_forward').value
        self.trim_backward = self.get_parameter('trim_backward').value
        self.trim_left = self.get_parameter('trim_left').value
        self.trim_right = self.get_parameter('trim_right').value
        self.trim_front_left = self.get_parameter('trim_front_left').value
        self.trim_front_right = self.get_parameter('trim_front_right').value
        self.assembly_fixed_speed = self.get_parameter('assembly_fixed_wheel_speed').value

        self.kinematics = OmniKinematics(self.wheel_radius, self.wheel_base, self.wheel_track)

        # Initialize Wheel PIDs
        kp = self.get_parameter('wheel_velocity_pid.kp').value
        ki = self.get_parameter('wheel_velocity_pid.ki').value
        kd = self.get_parameter('wheel_velocity_pid.kd').value
        mi = self.get_parameter('wheel_velocity_pid.max_integral').value
        ol = self.get_parameter('wheel_velocity_pid.output_limit').value
        
        self.wheel_pids = [PIDController(kp, ki, kd, mi, ol) for _ in range(4)]

        # State
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_wz = 0.0

        self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        self.prev_ticks = [0, 0, 0, 0]
        self.prev_encoder_time = None
        self.wheel_distances_cm = [0.0, 0.0, 0.0, 0.0]
        
        self.imu_yaw = 0.0
        self.initial_yaw = None
        self.imu_active = False
        self.assembly_mode_active = False

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        self.prev_control_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.15

        # Use BEST_EFFORT QoS for real-time control and high-frequency sensor data
        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Publishers
        self.motor_cmd_pub = self.create_publisher(Float32MultiArray, '/motor_cmds', 10)
        self.dist_pub = self.create_publisher(WheelDistances, '/wheel_distances', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, control_qos)
        self.create_subscription(Int32MultiArray, '/encoder_ticks', self.encoder_callback, control_qos)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, control_qos)
        self.create_subscription(Bool, '/assembly_mode_active', self.assembly_callback, control_qos)

        self.timer = self.create_timer(1.0 / self.control_freq, self.control_loop)
        self.get_logger().info('MASTER CONTROL MODE: PID and Deadband compensation ENABLED')

    def clip(self, val, min_val, max_val):
        return max(min_val, min(max_val, val))

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def cmd_callback(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def assembly_callback(self, msg: Bool):
        self.assembly_mode_active = msg.data

    def encoder_callback(self, msg: Int32MultiArray):
        if len(msg.data) < 4: return
        current_time = self.get_clock().now()
        ticks = list(msg.data)
        
        wheel_circum = 2 * math.pi * self.wheel_radius
        m_per_tick = wheel_circum / (self.ticks_per_rev * self.gear_ratio)

        if self.prev_encoder_time is not None:
            dt = (current_time - self.prev_encoder_time).nanoseconds / 1e9
            if dt > 0:
                for i in range(4):
                    # NOTE: Ticks must be cumulative from the ESP32. 
                    # If ESP32 publishes absolute counts, delta is correct.
                    delta = ticks[i] - self.prev_ticks[i]
                    # Preserve the sign for velocity!
                    dist = delta * m_per_tick
                    self.wheel_velocities[i] = dist / dt
                    self.wheel_distances_cm[i] += abs(dist) * 100

        self.prev_ticks = ticks
        self.prev_encoder_time = current_time

    def imu_callback(self, msg: Imu):
        self.imu_active = True
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)
        if self.initial_yaw is None: self.initial_yaw = raw_yaw
        self.imu_yaw = self.normalize_angle(raw_yaw - self.initial_yaw)

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_control_time).nanoseconds / 1e9
        self.prev_control_time = current_time
        if dt <= 0: return

        # Timeout Check
        time_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        is_idle = time_since_cmd > self.cmd_timeout or (abs(self.cmd_vx) < self.vel_deadzone and abs(self.cmd_vy) < self.vel_deadzone and abs(self.cmd_wz) < self.ang_deadzone)
        
        if is_idle:
            # ROBOCON FIX: Instant Stop (Bypass Ramp)
            self.filtered_vx = 0.0; self.filtered_vy = 0.0; self.filtered_wz = 0.0
            for pid in self.wheel_pids: pid.reset()
        else:
            # ROBOCON FIX: Separate Accel and Decel rates
            accel_lin = self.max_accel_linear * dt
            decel_lin = self.max_decel_linear * dt
            accel_ang = self.max_accel_angular * dt
            decel_ang = self.max_decel_angular * dt
            
            # X-Axis Logic
            diff_x = self.cmd_vx - self.filtered_vx
            is_decel_x = (abs(self.cmd_vx) < abs(self.filtered_vx)) or (self.cmd_vx * self.filtered_vx < 0)
            rate_x = decel_lin if is_decel_x else accel_lin
            self.filtered_vx += self.clip(diff_x, -rate_x, rate_x)
            
            # Y-Axis Logic
            diff_y = self.cmd_vy - self.filtered_vy
            is_decel_y = (abs(self.cmd_vy) < abs(self.filtered_vy)) or (self.cmd_vy * self.filtered_vy < 0)
            rate_y = decel_lin if is_decel_y else accel_lin
            self.filtered_vy += self.clip(diff_y, -rate_y, rate_y)
            
            # Yaw Logic - INSTANT REFLEXES (Bypass Accel Limit)
            self.filtered_wz = self.cmd_wz

        # Inverse Kinematics
        target_ang_vels = self.kinematics.inverse(self.filtered_vx, self.filtered_vy, self.filtered_wz)

        # Closed-Loop Control
        output_cmds = [0.0] * 4
        
        for i in range(4):
            target_vel = target_ang_vels[i] * self.wheel_radius
            
            # Apply directional trim scaling
            # Front-left diagonal (vx>0 AND vy>0): FR (index 1) anti-clockwise + BL (index 2) clockwise
            # Front-right diagonal (vx>0 AND vy<0): FL (index 0) clockwise + BR (index 3) anti-clockwise
            total_command = abs(self.filtered_vx) + abs(self.filtered_vy)
            if total_command > 0.01:
                w_forward = max(0.0, self.filtered_vx) / total_command
                w_backward = max(0.0, -self.filtered_vx) / total_command
                w_left = max(0.0, self.filtered_vy) / total_command
                w_right = max(0.0, -self.filtered_vy) / total_command

                # Front-left diagonal weight: both vx>0 and vy>0 simultaneously
                w_front_left = min(w_forward, w_left)
                # Front-right diagonal weight: vx>0 and vy<0 simultaneously
                w_front_right = min(w_forward, w_right)
                # Reduce cardinal weights proportionally so the blend sum stays consistent
                w_forward  = max(0.0, w_forward - w_front_left - w_front_right)
                w_left     = max(0.0, w_left    - w_front_left)
                w_right    = max(0.0, w_right   - w_front_right)

                scale = (w_forward     * self.trim_forward[i] +
                         w_backward    * self.trim_backward[i] +
                         w_left        * self.trim_left[i] +
                         w_right       * self.trim_right[i] +
                         w_front_left  * self.trim_front_left[i] +
                         w_front_right * self.trim_front_right[i])
                target_vel *= scale

            # =========================================
            # ASYMMETRIC THRUST FOR ANALOG ASSEMBLY
            # =========================================
            # If Shift is held (Assembly Active), the 3rd and 4th wheels (index 2 and 3) completely 
            # ignore the joystick magnitude and lock to a fixed speed (default 0.0).
            if self.assembly_mode_active and (i == 0 or i == 1):
                target_vel = self.assembly_fixed_speed

            # Original old bot hardcoded scaling (kept commented out for reference)
            # if i == 3 and (self.filtered_vx < -0.01 or self.filtered_vy < -0.01):
            #     target_vel *= 0.6

            # if (i == 0) and (self.filtered_vx < -0.01 or self.filtered_vy > 0.01):
            #    target_vel *= 0.6

            # if (i == 0) and (self.filtered_vx < -0.01 or self.filtered_vy < -0.01):
            #    target_vel *= 0.6
            
            # if (i == 1) and (self.filtered_vx > 0.01 or self.filtered_vy < -0.01):
            #    target_vel *= 0.6

            actual_vel = self.wheel_velocities[i]
            
            if is_idle or abs(target_vel) < self.wheel_pid_target_deadzone:
                output_cmds[i] = 0.0
                self.wheel_pids[i].reset()
                continue
                
            error = target_vel - actual_vel
            correction = self.wheel_pids[i].compute(error, dt)
            
            # Final Output = Target + PID Correction
            raw_output = target_vel + correction
            
            # Smooth Deadband Fade-in to prevent the "Hammer" effect on sensitive wheels
            if abs(raw_output) > 0.01:
                fade = min(1.0, abs(raw_output) / 0.5) # Fades in from 0.0 to 0.5 m/s
                sign = 1.0 if raw_output >= 0 else -1.0
                output_cmds[i] = (sign * self.deadband_offset * fade) + raw_output
            else:
                output_cmds[i] = 0.0

        # Power Scaling
        max_val = max(abs(v) for v in output_cmds) if output_cmds else 0.0
        if max_val > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_val
            output_cmds = [v * scale for v in output_cmds]

        self.motor_cmd_pub.publish(Float32MultiArray(data=output_cmds))

        # Odometry update
        v_kin = self.kinematics.forward(np.array(self.wheel_velocities) / self.wheel_radius)
        self.odom_theta = self.imu_yaw if self.imu_active else self.normalize_angle(self.odom_theta + v_kin[2] * dt)
        self.odom_x += (v_kin[0] * math.cos(self.odom_theta) - v_kin[1] * math.sin(self.odom_theta)) * dt
        self.odom_y += (v_kin[0] * math.sin(self.odom_theta) + v_kin[1] * math.cos(self.odom_theta)) * dt
        
        self.publish_odom(current_time)
        self.publish_wheel_distances()

    def publish_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.z = math.sin(self.odom_theta/2.0)
        odom.pose.pose.orientation.w = math.cos(self.odom_theta/2.0)
        self.odom_pub.publish(odom)
        t = TransformStamped()
        t.header = odom.header; t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = self.odom_x; t.transform.translation.y = self.odom_y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def publish_wheel_distances(self):
        msg = WheelDistances()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.front_left_cm = float(self.wheel_distances_cm[0])
        msg.front_right_cm = float(self.wheel_distances_cm[1])
        msg.back_left_cm = float(self.wheel_distances_cm[2])
        msg.back_right_cm = float(self.wheel_distances_cm[3])
        msg.total_distance_cm = sum(self.wheel_distances_cm) / 4.0
        
        # Optionally populate velocities
        msg.front_left_vel = self.wheel_velocities[0] * 100.0
        msg.front_right_vel = self.wheel_velocities[1] * 100.0
        msg.back_left_vel = self.wheel_velocities[2] * 100.0
        msg.back_right_vel = self.wheel_velocities[3] * 100.0
        
        self.dist_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.motor_cmd_pub.publish(Float32MultiArray(data=[0.0,0.0,0.0,0.0])); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
