#!/usr/bin/python3
"""
Mechanism control node.
Converted to use standard Float32MultiArray for ESP32 compatibility.

Subscribes:
    /joy (sensor_msgs/Joy)

Publishes:
    /mechanism_cmds (std_msgs/Float32MultiArray)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Bool


class MechanismNode(Node):
    def __init__(self):
        super().__init__('mechanism_node')

        self.declare_parameter('servo_min_deg', 0)
        self.declare_parameter('servo_max_deg', 180)
        self.declare_parameter('servo1_2_initial_deg', 120)
        self.declare_parameter('servo3_initial_deg', 123)
        self.declare_parameter('servo4_initial_deg', 91)
        self.declare_parameter('servo5_initial_deg', 152)
        self.declare_parameter('servo_step_deg', 1)
        self.declare_parameter('update_period_ms', 25)
        self.declare_parameter('right_stick_threshold', 0.35)

        self.declare_parameter('axis_servo4', 2)
        self.declare_parameter('axis_servo5', 3)

        self.declare_parameter('button_servo1_2_decrease', 0)
        self.declare_parameter('button_servo1_2_increase', 3)
        self.declare_parameter('button_servo3_decrease', 11)
        self.declare_parameter('button_servo3_increase', 10)
        self.declare_parameter('button_pneumatic_gripper_toggle', 9)
        self.declare_parameter('button_linear_actuator_toggle', 8)
        self.declare_parameter('button_lead_screw_positive', 12)
        self.declare_parameter('button_lead_screw_negative', 13)
        self.declare_parameter('button_turbo', 7)            # R2 - shift / lead screw turbo speed
        self.declare_parameter('button_linear_rhino', -1)    # Disabled to use L2 (6) as brake

        self.declare_parameter('lead_screw_speed', 1.5)
        self.declare_parameter('lead_screw_turbo_speed', 3.0)
        self.declare_parameter('linear_rhino_speed', 1.5)   # signed command magnitude

        self.servo_min = int(self.get_parameter('servo_min_deg').value)
        self.servo_max = int(self.get_parameter('servo_max_deg').value)
        servo1_2_initial = int(self.get_parameter('servo1_2_initial_deg').value)
        servo3_initial = int(self.get_parameter('servo3_initial_deg').value)
        servo4_initial = int(self.get_parameter('servo4_initial_deg').value)
        servo5_initial = int(self.get_parameter('servo5_initial_deg').value)
        self.servo_step = int(self.get_parameter('servo_step_deg').value)
        self.right_stick_threshold = float(self.get_parameter('right_stick_threshold').value)

        self.axis_servo4 = int(self.get_parameter('axis_servo4').value)
        self.axis_servo5 = int(self.get_parameter('axis_servo5').value)

        self.btn_servo1_2_dec = int(self.get_parameter('button_servo1_2_decrease').value)
        self.btn_servo1_2_inc = int(self.get_parameter('button_servo1_2_increase').value)
        self.btn_servo3_dec = int(self.get_parameter('button_servo3_decrease').value)
        self.btn_servo3_inc = int(self.get_parameter('button_servo3_increase').value)
        self.btn_gripper = int(self.get_parameter('button_pneumatic_gripper_toggle').value)
        self.btn_linear_actuator = int(self.get_parameter('button_linear_actuator_toggle').value)
        self.btn_lead_screw_pos = int(self.get_parameter('button_lead_screw_positive').value)
        self.btn_lead_screw_neg = int(self.get_parameter('button_lead_screw_negative').value)
        self.btn_turbo = int(self.get_parameter('button_turbo').value)
        self.btn_linear_rhino = int(self.get_parameter('button_linear_rhino').value)

        self.lead_screw_speed = float(self.get_parameter('lead_screw_speed').value)
        self.lead_screw_turbo_speed = float(self.get_parameter('lead_screw_turbo_speed').value)
        self.linear_rhino_speed = float(self.get_parameter('linear_rhino_speed').value)

        self.servo1_2 = self.clamp_servo(servo1_2_initial)
        self.servo3 = self.clamp_servo(servo3_initial)
        self.servo4 = self.clamp_servo(servo4_initial)
        self.servo5 = self.clamp_servo(servo5_initial)
        self.servo5 = self.clamp_servo(servo5_initial)
        self.linear_actuator = 0.0
        self.display_status = False
        self.pneumatic_gripper = False
        self.lead_screw = 0.0
        self.linear_rhino = 0.0   # hold btn_linear_rhino -> speed, shift-first combo -> negative speed
        self.linear_rhino_reverse_combo = False

        self.SERVO_PRESETS = {
            'pick_staff':   {'servo4': 176, 'servo5': 81},
            'box_pick_200': {'servo1_2': 55, 'servo3': 123},
            'box_pick_400': {'servo1_2': 78, 'servo3': 118},
            'box_pick_600': {'servo1_2': 102, 'servo3': 88},
            'box_place':    {'servo1_2': 93, 'servo3': 102},
            'box_store':    {'servo1_2': 122, 'servo3': 180},
        }

        self.axes = []
        self.buttons = []
        self.prev_buttons = []
        self.last_joy_time = self.get_clock().now()
        self.joy_timeout = 0.3

        # /mechanism_cmds publisher — RELIABLE to match ESP3 rclc_subscription_init_default.
        pub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.cmds_pub = self.create_publisher(Float32MultiArray, '/mechanism_cmds', pub_qos)
        self.display_pub = self.create_publisher(Bool, '/display_status', pub_qos)

        # /joy subscriber — BEST_EFFORT to match ESP1 (Standalone.ino rclc_publisher_init_default).
        # depth=10 is critical: Standalone.ino only publishes on state change, so press+release
        # arrive as two rapid messages. depth=1 drops the press, killing button_pressed() edge
        # detection (linear_actuator, pneumatic_gripper toggles). depth=10 queues both in order.
        joy_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.create_subscription(Joy, '/joy', self.joy_callback, joy_qos)

        update_period = float(self.get_parameter('update_period_ms').value) / 1000.0
        self.timer = self.create_timer(update_period, self.update_and_publish)

        self.get_logger().info('Mechanism Node started: /joy -> /mechanism_cmds')
        self.get_logger().info(
            f'[BTN MAP] servo1_2={self.btn_servo1_2_dec}/{self.btn_servo1_2_inc} '
            f'servo3={self.btn_servo3_dec}/{self.btn_servo3_inc} '
            f'linear_act={self.btn_linear_actuator} '
            f'gripper={self.btn_gripper} '
            f'lead_screw+={self.btn_lead_screw_pos} '
            f'lead_screw-={self.btn_lead_screw_neg} '
            f'rhino={self.btn_linear_rhino} '
            f'turbo={self.btn_turbo}'
        )

    def clamp_servo(self, value: int) -> int:
        return max(self.servo_min, min(self.servo_max, int(value)))

    def axis_value(self, axis_id: int) -> float:
        if axis_id < 0 or axis_id >= len(self.axes):
            return 0.0
        return float(self.axes[axis_id])

    def button_held(self, button_id: int) -> bool:
        if button_id < 0 or button_id >= len(self.buttons):
            return False
        return self.buttons[button_id] == 1

    def button_pressed(self, button_id: int) -> bool:
        if button_id < 0 or button_id >= len(self.buttons):
            return False
        if button_id >= len(self.prev_buttons):
            return self.buttons[button_id] == 1
        return self.buttons[button_id] == 1 and self.prev_buttons[button_id] == 0

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)

        # --- Diagnostic: log every button press (rising edge) ---
        if self.prev_buttons:
            pressed = [
                i for i, (b, pb) in enumerate(zip(self.buttons, self.prev_buttons))
                if b == 1 and pb == 0
            ]
            if pressed:
                self.get_logger().info(f'[JOY] Button(s) pressed: {pressed}')

        def trigger_macro_if_shifted(button_idx, macro_name):
            if self.button_pressed(button_idx):
                shift_held = self.button_held(self.btn_turbo)
                shift_was_already_held = (
                    self.btn_turbo >= 0
                    and self.btn_turbo < len(self.prev_buttons)
                    and self.prev_buttons[self.btn_turbo] == 1
                )
                if shift_held and shift_was_already_held:
                    preset = self.SERVO_PRESETS.get(macro_name)
                    if preset:
                        if 'servo1_2' in preset: self.servo1_2 = self.clamp_servo(preset['servo1_2'])
                        if 'servo3'   in preset: self.servo3   = self.clamp_servo(preset['servo3'])
                        if 'servo4'   in preset: self.servo4   = self.clamp_servo(preset['servo4'])
                        if 'servo5'   in preset: self.servo5   = self.clamp_servo(preset['servo5'])
                        self.get_logger().info(f'[MACRO] {macro_name} triggered!')
                    return True
            return False

        # Apply new box pick/place macros
        trigger_macro_if_shifted(10, 'box_pick_200')
        trigger_macro_if_shifted(12, 'box_pick_400')
        trigger_macro_if_shifted(13, 'box_pick_600')
        trigger_macro_if_shifted(11, 'box_place')
        trigger_macro_if_shifted(4, 'box_store')

        if self.button_pressed(self.btn_gripper):
            shift_held = self.button_held(self.btn_turbo)
            shift_was_already_held = (
                self.btn_turbo >= 0
                and self.btn_turbo < len(self.prev_buttons)
                and self.prev_buttons[self.btn_turbo] == 1
            )
            
            if shift_held and shift_was_already_held:
                preset = self.SERVO_PRESETS.get('pick_staff')
                if preset:
                    if 'servo4' in preset: self.servo4 = self.clamp_servo(preset['servo4'])
                    if 'servo5' in preset: self.servo5 = self.clamp_servo(preset['servo5'])
                    self.get_logger().info(f'[MACRO] pick_staff triggered: s4={self.servo4}, s5={self.servo5}')
            else:
                self.pneumatic_gripper = not self.pneumatic_gripper
                self.get_logger().info(f'[TOGGLE] pneumatic_gripper -> {self.pneumatic_gripper}')

        actuator_pressed = self.button_pressed(self.btn_linear_actuator)
        if actuator_pressed:
            shift_held = self.button_held(self.btn_turbo)
            shift_was_already_held = (
                self.btn_turbo >= 0
                and self.btn_turbo < len(self.prev_buttons)
                and self.prev_buttons[self.btn_turbo] == 1
            )
            if shift_held and shift_was_already_held:
                self.display_status = not self.display_status
                display_msg = Bool()
                display_msg.data = self.display_status
                self.display_pub.publish(display_msg)
                self.get_logger().info(f'[TOGGLE] Display Status -> {self.display_status}')

        rhino_held = self.button_held(self.btn_linear_rhino)
        if not rhino_held:
            self.linear_rhino_reverse_combo = False
        elif self.button_pressed(self.btn_linear_rhino):
            shift_held = self.button_held(self.btn_turbo)
            shift_was_already_held = (
                self.btn_turbo >= 0
                and self.btn_turbo < len(self.prev_buttons)
                and self.prev_buttons[self.btn_turbo] == 1
            )
            self.linear_rhino_reverse_combo = shift_held and shift_was_already_held

        self.prev_buttons = list(self.buttons)

    def update_servo_from_buttons(self, value: int, dec_button: int, inc_button: int) -> int:
        if self.button_held(self.btn_turbo):
            return self.clamp_servo(value)

        if self.button_held(dec_button):
            value -= self.servo_step
        if self.button_held(inc_button):
            value += self.servo_step
        return self.clamp_servo(value)

    def update_servo_from_axis(self, value: int, axis: float) -> int:
        if abs(axis) < self.right_stick_threshold:
            return value
        if axis < 0.0:
            value -= self.servo_step
        else:
            value += self.servo_step
        return self.clamp_servo(value)

    # linear_actuator is a signed toggle: +1.0 forward, -1.0 reverse, 0.0 stop

    def update_lead_screw(self):
        if self.button_held(self.btn_turbo):
            self.lead_screw = 0.0
            return

        speed = self.lead_screw_speed
        positive = self.button_held(self.btn_lead_screw_pos)
        negative = self.button_held(self.btn_lead_screw_neg)

        if positive and not negative:
            self.lead_screw = speed
        elif negative and not positive:
            self.lead_screw = -speed
        else:
            self.lead_screw = 0.0

    def update_linear_actuator(self):
        actuator_held = self.button_held(self.btn_linear_actuator)
        shift_held = self.button_held(self.btn_turbo)

        if not actuator_held or shift_held:
            self.linear_actuator = 0.0
            return

        self.linear_actuator = 1.0

    def update_linear_rhino(self):
        """Run rhino forward normally, or reverse with shift held before rhino is pressed."""
        rhino_held = self.button_held(self.btn_linear_rhino)
        shift_held = self.button_held(self.btn_turbo)

        if not rhino_held:
            self.linear_rhino_reverse_combo = False
            self.linear_rhino = 0.0
            return

        if self.linear_rhino_reverse_combo and shift_held:
            self.linear_rhino = -self.linear_rhino_speed
        else:
            self.linear_rhino = self.linear_rhino_speed

    def update_and_publish(self):
        time_since_joy = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if time_since_joy > self.joy_timeout:
            # Timeout: stop all DC motors, keep toggle states
            self.lead_screw = 0.0
            self.linear_rhino = 0.0   # safety: stop rhino if controller disconnects
            self.linear_actuator = 0.0
            self.publish_cmds()
            return

        self.servo1_2 = self.update_servo_from_buttons(
            self.servo1_2,
            self.btn_servo1_2_dec,
            self.btn_servo1_2_inc,
        )
        self.servo3 = self.update_servo_from_buttons(
            self.servo3,
            self.btn_servo3_dec,
            self.btn_servo3_inc,
        )
        self.servo4 = self.update_servo_from_axis(self.servo4, -self.axis_value(self.axis_servo4))
        self.servo5 = self.update_servo_from_axis(self.servo5, -self.axis_value(self.axis_servo5))
        self.update_lead_screw()
        self.update_linear_actuator()
        self.update_linear_rhino()
        self.publish_cmds()

    def publish_cmds(self):
        msg = Float32MultiArray()
        msg.data = [
            float(self.servo1_2),                            # [0] Servo 1+2 angle
            float(self.servo3),                              # [1] Servo 3 angle
            float(self.servo4),                              # [2] Servo 4 angle
            float(self.servo5),                              # [3] Servo 5 angle
            float(self.linear_actuator),                     # [4] Linear Actuator (+/-1.0, 0 stop)
            1.0 if self.pneumatic_gripper else 0.0,          # [5] Pneumatic Gripper (MDDRC10 bool)
            float(self.lead_screw),                          # [6] Lead Screw vel (±3.0)
            float(self.linear_rhino),                        # [7] Linear Rhino vel (+/-)
        ]
        self.cmds_pub.publish(msg)

        # Publish display status continuously for reliability
        display_msg = Bool()
        display_msg.data = self.display_status
        self.display_pub.publish(display_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MechanismNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
