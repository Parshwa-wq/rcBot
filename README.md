# rcBot — Mecanum Drive ROS2 Control Stack

A ROS2 (`ament_cmake`) package for a 4-wheel mecanum-drive robot, featuring PS5 DualSense teleoperation, IMU-corrected heading control, and closed-loop per-wheel PID velocity control with odometry. The Jetson/PC side (this repo) communicates with a Teensy/ESP32 microcontroller over **micro-ROS** (serial).

## Requirements

- ROS2 (Humble or Jazzy recommended)
- `ros-<distro>-joy` package (for PS5 controller input)
- `micro_ros_agent` (bridges the ESP32/Teensy to ROS2 over serial)
- Python packages — install via:
  ```bash
  pip install -r requirements.txt
  ```

## Package Overview (`ocpkg`)

| Component | Type | Purpose |
|---|---|---|
| `ps5_teleop_node.py` | Node | Converts `/joy` (PS5 DualSense) input into raw velocity commands and discrete rotation requests |
| `yaw_manager_node.py` | Node | Uses IMU yaw data to lock heading and apply precise rotation corrections |
| `pid_controller_node.py` | Node | Per-wheel PID velocity control using encoder feedback; publishes motor commands and odometry |
| `mechanism_node.py` | Node | Controls servos, pneumatic gripper, linear actuator, and lead screw from joystick input |
| `signal_display_node.py` | Node | Tkinter-based GUI for visual robot status/signal display |
| `encoder_calibration.py` | Script | Standalone encoder calibration utility |
| `imu_accuracy_diagnostic.py` | Script | Standalone IMU accuracy diagnostic |
| `motor_voltage_diagnostic.py` | Script | Standalone motor voltage diagnostic |
| `natural_bias_test.py` | Script | Standalone bias/drift test |
| `EncoderTicks.msg`, `MotorCmds.msg`, `WheelDistances.msg`, `MechanismState.msg` | Custom messages | ESP32 ↔ ROS2 data interfaces |

## Execution Flow

1. **Build the workspace**
   ```bash
   cd ~/rcBot
   colcon build
   source install/setup.bash
   ```

2. **Start the micro-ROS agent** (bridges ESP32/Teensy over serial — run manually, not part of the launch file):
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
   ```

3. **Start the ROS2 joy node** (publishes `/joy` from the PS5 controller):
   ```bash
   ros2 run joy joy_node
   ```

4. **Launch the main control stack**:
   ```bash
   ros2 launch ocpkg macnum_control.launch.py
   ```
   This brings up 5 nodes together:
   - `ps5_teleop_node` → reads `/joy` → publishes `/cmd_vel_raw`, `/rotation_request`
   - `yaw_manager_node` → reads `/cmd_vel_raw`, `/rotation_request`, `/imu/data` → publishes corrected `/cmd_vel`, `/imu_deg`
   - `pid_controller_node` → reads `/cmd_vel`, `/encoder_ticks`, `/imu/data` → publishes `/motor_cmds` (to ESP32), `/wheel_distances` (debug), `/odom`
   - `mechanism_node` → reads `/joy` → publishes `/mechanism_cmds` (arm/gripper subsystem)
   - `signal_display_node` → Tkinter GUI, launched in `--mode color`

### Data Flow Diagram
```
PS5 Controller
      │
      ▼
   /joy ──────────────┬───────────────────┐
      │                │                   │
      ▼                ▼                   ▼
ps5_teleop_node   mechanism_node      (joy also feeds
      │                │               yaw/rotation reqs)
      ▼                ▼
/cmd_vel_raw     /mechanism_cmds
/rotation_request        │
      │                  ▼
      ▼              ESP32 (servos, gripper,
yaw_manager_node      linear actuator, lead screw)
      │  ▲
      │  └── /imu/data (from ESP32 via micro-ROS)
      ▼
   /cmd_vel
      │
      ▼
pid_controller_node ◄── /encoder_ticks (from ESP32)
      │
      ├──► /motor_cmds ──► ESP32 (wheel motors)
      ├──► /wheel_distances (debug)
      └──► /odom
```

## Notes

- The micro-ROS agent is intentionally **not** included in the launch file — run it manually before launching, since it depends on your specific serial port/baud rate.
- Diagnostic scripts (`encoder_calibration.py`, `imu_accuracy_diagnostic.py`, `motor_voltage_diagnostic.py`, `natural_bias_test.py`) are meant to be run standalone for calibration/tuning, not as part of normal operation.
- 
