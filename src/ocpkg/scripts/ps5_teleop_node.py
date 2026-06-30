#!/usr/bin/python3
"""
PS5 Teleop Node for Mecanum Drive
==================================
Converts PS5 DualSense controller input to velocity commands.

Features:
- Standard joystick control (linear X, Y, angular Z)
- 90-degree rotation request buttons
- Turbo and slow modes (L2/R2)
- Emergency stop (PS button)

Subscribes:
    /joy (sensor_msgs/Joy) - Joystick input from controller

Publishes:
    /cmd_vel_raw (geometry_msgs/Twist) - Raw joystick velocity commands
    /rotation_request (std_msgs/Float32) - Relative rotation request in degrees
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool


class PS5TeleopNode(Node):
    """PS5 Controller to raw velocity and rotation request converter."""

    def __init__(self):
        super().__init__('ps5_teleop_node')

        # =========================================
        # DECLARE PARAMETERS
        # =========================================

        # Axis mappings
        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_linear_y', 0)
        self.declare_parameter('axis_angular_z', -1)
        self.declare_parameter('axis_angular_z_fallback', -1)
        
        # Axis scaling
        self.declare_parameter('axis_linear_x_scale', 1.0)
        self.declare_parameter('axis_linear_y_scale', -1.0)
        self.declare_parameter('axis_angular_z_scale', -1.0)

        # Button mappings
        self.declare_parameter('button_rotate_right_90', 1)  # Circle
        self.declare_parameter('button_rotate_left_90', 2)   # Square
        self.declare_parameter('button_rotate_left_hold', 4)  # L1
        self.declare_parameter('button_rotate_right_hold', 5) # R1
        self.declare_parameter('button_turbo', 7)            # R2 - turbo mode (faster)
        self.declare_parameter('button_brake', 6)            # Fixed speed decreaser (30%)
        self.declare_parameter('button_emergency_stop', -1)  # Disabled; L3 used by mechanisms

        # Speed settings
        self.declare_parameter('speed_linear', 3.0)          # Significantly increased
        self.declare_parameter('speed_angular', 4.0)
        self.declare_parameter('turbo_linear', 5.0)
        self.declare_parameter('turbo_angular', 6.0)
        self.declare_parameter('slow_linear', 0.5)
        self.declare_parameter('slow_angular', 1.0)


        # Other settings
        self.declare_parameter('deadzone', 0.40)
        self.declare_parameter('strict_threshold', 0.50)
        self.declare_parameter('assembly_deadzone', 0.5)
        self.declare_parameter('axis_snap_ratio', 1.8)
        # Front-left diagonal zone: joystick pushed to front-left corner.
        # Wheels: FR (index 1, anti-clockwise) + BL (index 2, clockwise).
        self.declare_parameter('front_left_diag_threshold', 0.25)
        # Front-right diagonal zone: joystick pushed to front-right corner.
        # Wheels: FL (index 0, clockwise) + BR (index 3, anti-clockwise).
        self.declare_parameter('front_right_diag_threshold', 0.25)
        self.declare_parameter('smoothing_factor', 1.0)
        self.declare_parameter('joy_timeout', 0.3)

        # =========================================
        # GET PARAMETERS
        # =========================================

        self.axis_linear_x = self.get_parameter('axis_linear_x').value
        self.axis_linear_y = self.get_parameter('axis_linear_y').value
        self.axis_angular_z = self.get_parameter('axis_angular_z').value
        self.axis_angular_z_fallback = self.get_parameter('axis_angular_z_fallback').value

        self.scale_linear_x = self.get_parameter('axis_linear_x_scale').value
        self.scale_linear_y = self.get_parameter('axis_linear_y_scale').value
        self.scale_angular_z = self.get_parameter('axis_angular_z_scale').value

        self.btn_rotate_right_90 = self.get_parameter('button_rotate_right_90').value
        self.btn_rotate_left_90 = self.get_parameter('button_rotate_left_90').value
        self.btn_rotate_left_hold = self.get_parameter('button_rotate_left_hold').value
        self.btn_rotate_right_hold = self.get_parameter('button_rotate_right_hold').value
        self.btn_turbo = self.get_parameter('button_turbo').value
        self.button_brake = self.get_parameter('button_brake').value
        self.btn_estop = self.get_parameter('button_emergency_stop').value

        self.speed_linear = self.get_parameter('speed_linear').value
        self.speed_angular = self.get_parameter('speed_angular').value
        self.turbo_linear = self.get_parameter('turbo_linear').value
        self.turbo_angular = self.get_parameter('turbo_angular').value

        self.deadzone = self.get_parameter('deadzone').value
        self.assembly_deadzone = self.get_parameter('assembly_deadzone').value
        self.strict_threshold = self.get_parameter('strict_threshold').value
        self.axis_snap_ratio = self.get_parameter('axis_snap_ratio').value
        self.front_left_diag_threshold = self.get_parameter('front_left_diag_threshold').value
        self.front_right_diag_threshold = self.get_parameter('front_right_diag_threshold').value
        self.smoothing = self.get_parameter('smoothing_factor').value
        self.joy_timeout = self.get_parameter('joy_timeout').value
        # =========================================
        # STATE VARIABLES
        # =========================================

        self.emergency_stopped = False
        self.assembly_active = False
        self.assembly_hotkey_locked_out = False

        # Target velocity values (from joystick)
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0

        # Smoothed velocity values (actually published)
        self.smoothed_linear_x = 0.0
        self.smoothed_linear_y = 0.0
        self.smoothed_angular_z = 0.0

        # Previous button states (for edge detection)
        self.prev_buttons = []
        self.last_joy_time = self.get_clock().now()
        self.active_angular_axis = None

        # =========================================
        # PUBLISHERS & SUBSCRIBERS
        # =========================================

        # Use BEST_EFFORT QoS for real-time control to minimize lag
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', qos)
        self.rotation_request_pub = self.create_publisher(Float32, '/rotation_request', qos)
        self.assembly_mode_pub = self.create_publisher(Bool, '/assembly_mode_active', qos)

        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, qos
        )

        # Publish at fixed rate even when no joy messages
        self.timer = self.create_timer(0.02, self.update_and_publish)  # 50Hz

        self.get_logger().info('PS5 Teleop Node started')
        self.get_logger().info(f'  Linear axes: X={self.axis_linear_x}, Y={self.axis_linear_y}')
        self.get_logger().info(
            f'  Hold rotation: L={self.btn_rotate_left_hold}, R={self.btn_rotate_right_hold}'
        )
        self.get_logger().info(f'  90° rotation: Circ={self.btn_rotate_right_90}, X={self.btn_rotate_left_90}')

    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to joystick axis value."""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale remaining range to 0-1
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def smooth_value(self, current: float, target: float) -> float:
        """Apply low-pass filter for smoother control and snap to zero."""
        new_val = current + self.smoothing * (target - current)
        # Snap to zero if very close to prevent asymptotic drift
        if abs(new_val) < 0.001:
            return 0.0
        return new_val

    def button_pressed(self, buttons: list, button_id: int) -> bool:
        """Check if button was just pressed (rising edge)."""
        if button_id < 0 or button_id >= len(buttons):
            return False
        if button_id >= len(self.prev_buttons):
            return buttons[button_id] == 1
        return buttons[button_id] == 1 and self.prev_buttons[button_id] == 0

    def button_held(self, buttons: list, button_id: int) -> bool:
        """Check if button is currently held."""
        if button_id < 0 or button_id >= len(buttons):
            return False
        return buttons[button_id] == 1

    def joy_callback(self, msg: Joy):
        """Process joystick input."""

        self.last_joy_time = self.get_clock().now()
        axes = msg.axes
        buttons = msg.buttons

        # =========================================
        # EMERGENCY STOP CHECK
        # =========================================

        if self.button_pressed(buttons, self.btn_estop):
            self.emergency_stopped = not self.emergency_stopped
            if self.emergency_stopped:
                self.get_logger().warn('EMERGENCY STOP ACTIVATED!')
            else:
                self.get_logger().info('Emergency stop released')

        if self.emergency_stopped:
            self.target_linear_x = 0.0
            self.target_linear_y = 0.0
            self.target_angular_z = 0.0
            self.prev_buttons = list(buttons)
            return

        # =========================================
        # AUTOMATIC ROTATION BUTTONS
        # =========================================

        if not self.button_held(buttons, self.btn_turbo):
            if self.button_pressed(buttons, self.btn_rotate_right_90):
                self.get_logger().info('Rotating 90° RIGHT')
                self.publish_rotation_request(-90.0)

            elif self.button_pressed(buttons, self.btn_rotate_left_90):
                self.get_logger().info('Rotating 90° LEFT')
                self.publish_rotation_request(90.0)

        # =========================================
        # DETERMINE SPEED MULTIPLIERS
        # =========================================

        if self.button_held(buttons, self.btn_turbo):
            linear_mult = self.turbo_linear
            angular_mult = self.turbo_angular
        else:
            linear_mult = self.speed_linear
            angular_mult = self.speed_angular

        # =========================================
        # PROCESS JOYSTICK AXES
        # =========================================

        raw_linear_x = axes[self.axis_linear_x] if self.axis_linear_x < len(axes) else 0.0
        raw_linear_y = axes[self.axis_linear_y] if self.axis_linear_y < len(axes) else 0.0
        raw_angular_z = 0.0

        threshold = self.strict_threshold
        
        def apply_strict_threshold(val, scale):
            if abs(val) < threshold:
                return 0.0
            # Scale remaining range
            sign = 1.0 if val > 0 else -1.0
            return sign * (abs(val) - threshold) / (1.0 - threshold) * scale

        linear_x = apply_strict_threshold(raw_linear_x, self.scale_linear_x)
        linear_y = apply_strict_threshold(raw_linear_y, self.scale_linear_y)
        angular_z = 0.0

        # =========================================
        # SHIFT (BTN_TURBO) GLOBAL OVERRIDE LOGIC
        # =========================================
        shift_held_now = self.button_held(buttons, self.btn_turbo)
        shift_held_prev = self.button_held(self.prev_buttons, self.btn_turbo)
        
        # Determine if joystick was already moving BEFORE shift was pressed
        if shift_held_now and not shift_held_prev:
            joy_mag = math.sqrt(raw_linear_x**2 + raw_linear_y**2)
            if joy_mag > self.deadzone:
                self.assembly_hotkey_locked_out = True
            else:
                self.assembly_hotkey_locked_out = False
        elif not shift_held_now:
            self.assembly_hotkey_locked_out = False

        if shift_held_now and not self.assembly_hotkey_locked_out:
            # SHIFT PRESSED FIRST: Left Joystick forgets normal driving, transforms into Analog Assembly
            
            assembly_magnitude = math.sqrt(raw_linear_x**2 + raw_linear_y**2)
            
            if assembly_magnitude > self.assembly_deadzone:
                self.assembly_active = True
                # Force perfect left strafe based strictly on magnitude (Analog)
                target_linear_y = min(1.0, assembly_magnitude) * self.speed_linear
                target_linear_x = 0.0
                target_angular_z = 0.0
            else:
                self.assembly_active = False
                target_linear_y = 0.0
                target_linear_x = 0.0
                target_angular_z = 0.0
        else:
            # SHIFT NOT HELD (Or Locked Out): Normal Driving Duty
            self.assembly_active = False
            
            if self.button_held(buttons, self.btn_rotate_left_hold):
                angular_z += self.scale_angular_z
            if self.button_held(buttons, self.btn_rotate_right_hold):
                angular_z -= self.scale_angular_z

            # ── Front-Left Diagonal Zone ─────────────────────────────────────────
            # Joystick front-left corner: linear_x > 0 AND linear_y > 0.
            # Mecanum: FR (index 1, anti-clockwise) + BL (index 2, clockwise) active.
            in_front_left_diag = (
                linear_x > self.front_left_diag_threshold and
                linear_y > self.front_left_diag_threshold
            )

            # ── Front-Right Diagonal Zone ────────────────────────────────────────
            # Joystick front-right corner: linear_x > 0 AND linear_y < 0.
            # Mecanum: FL (index 0, clockwise) + BR (index 3, anti-clockwise) active.
            in_front_right_diag = (
                linear_x > self.front_right_diag_threshold and
                linear_y < -self.front_right_diag_threshold
            )

            if in_front_left_diag or in_front_right_diag:
                # Preserve both axes — no snap, no zeroing.
                pass
            elif abs(linear_x) > abs(linear_y) * self.axis_snap_ratio:
                linear_y = 0.0
            elif abs(linear_y) > abs(linear_x) * self.axis_snap_ratio:
                linear_x = 0.0

            # If Shift is held (but Assembly Mode is locked out), act as Turbo Mode
            if shift_held_now:
                linear_mult = self.turbo_linear
                angular_mult = self.turbo_angular
            else:
                linear_mult = self.speed_linear
                angular_mult = self.speed_angular

            target_linear_x = linear_x * linear_mult
            target_linear_y = linear_y * linear_mult
            target_angular_z = angular_z * angular_mult

            # Straighten forward/backward motion (suppress small Y drift).
            # Skipped while in any diagonal zone to preserve the strafe component.
            in_diag_zone = in_front_left_diag or in_front_right_diag
            if not in_diag_zone and abs(target_linear_x) > 0.05:
                if abs(target_linear_y) < 0.15: target_linear_y = 0.0
                if abs(target_angular_z) < 0.15: target_angular_z = 0.0

        self.target_linear_x = target_linear_x
        self.target_linear_y = target_linear_y
        self.target_angular_z = target_angular_z

        # Fixed speed decreaser (30% speed) when button is held
        if self.button_held(buttons, self.button_brake):
            speed_reduction = 0.30
            self.target_linear_x *= speed_reduction
            self.target_linear_y *= speed_reduction
            self.target_angular_z *= speed_reduction
            self.assembly_active = False

        # Save button states for edge detection
        self.prev_buttons = list(buttons)

    def publish_rotation_request(self, degrees: float):
        """Publish a one-shot relative rotation request."""
        self.target_linear_x = self.smoothed_linear_x = 0.0
        self.target_linear_y = self.smoothed_linear_y = 0.0
        self.target_angular_z = self.smoothed_angular_z = 0.0

        msg = Float32()
        msg.data = degrees
        self.rotation_request_pub.publish(msg)

    def update_and_publish(self):
        """Apply smoothing and publish velocity command."""

        time_since_joy = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if time_since_joy > self.joy_timeout:
            self.target_linear_x = 0.0
            self.target_linear_y = 0.0
            self.target_angular_z = 0.0

        # Apply smoothing in the timer loop for consistent decay
        self.smoothed_linear_x = self.smooth_value(self.smoothed_linear_x, self.target_linear_x)
        self.smoothed_linear_y = self.smooth_value(self.smoothed_linear_y, self.target_linear_y)
        self.smoothed_angular_z = self.smooth_value(self.smoothed_angular_z, self.target_angular_z)

        twist = Twist()

        twist.linear.x = self.smoothed_linear_x
        twist.linear.y = self.smoothed_linear_y
        twist.angular.z = self.smoothed_angular_z

        self.cmd_vel_pub.publish(twist)

        # Publish assembly mode state
        assembly_msg = Bool()
        assembly_msg.data = self.assembly_active
        self.assembly_mode_pub.publish(assembly_msg)



def main(args=None):
    rclpy.init(args=args)
    node = PS5TeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before shutting down
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
