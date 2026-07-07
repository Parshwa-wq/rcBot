# rcBot — Mecanum Drive ROS2 Control Stack

A ROS2 (`ament_cmake`) package for a 4-wheel mecanum-drive robot, featuring PS5 DualSense teleoperation (via Bluepad32), IMU-corrected heading control, and closed-loop per-wheel PID velocity control with odometry. The host (Jetson/PC) side (this repo's `ocpkg` package) communicates with **three separate ESP32 microcontrollers** over **micro-ROS**, each connected on its own USB serial port.

## Firmware (`INOs/`)

| Firmware File | MCU Role | micro-ROS Node Name | Publishes | Subscribes |
|---|---|---|---|---|
| `DriveSpec.ino` | **ESP1 — Drive/Main ESP.** Reads PS5 DualSense over Bluetooth (Bluepad32), reads wheel encoders, drives wheel motors | `esp32_mecanum_node` | `/joy`, `/encoder_ticks` | `/motor_cmds` |
| `IMUspec.ino` | **ESP2 — IMU ESP.** MPU6050 IMU, publishes orientation at 50 Hz | `imu_esp32_node` | `/imu/data` | — |
| `Mech.ino` | **ESP3 — Mechanism ESP.** Drives servos, pneumatic gripper, linear actuator, lead screw, and linear rhino motor | `esp32_mechanism_node` | — | `/mechanism_cmds` (control), `/joy` (debug only) |

Because there are **three ESP32s**, there are **three independent micro-ROS agents**, one per serial port:

| ESP | Serial Port | Baud |
|---|---|---|
| ESP1 — Drive (`DriveSpec.ino`) | `/dev/ttyUSB0` | 115200 |
| ESP2 — IMU (`IMUspec.ino`) | `/dev/ttyUSB1` | 115200 |
| ESP3 — Mechanism (`Mech.ino`) | `/dev/ttyUSB2` | 115200 |

> Port numbers are examples — confirm actual `/dev/ttyUSBx` assignment on your machine with `ls /dev/ttyUSB*` or `dmesg | grep tty` after plugging in, since Linux assigns them by enumeration order, not by ESP identity.

## Requirements

- ROS2 (Humble or Jazzy recommended)
- `micro_ros_agent` (bridges each ESP32 to ROS2 over serial — **three instances needed**, one per ESP)
- Python packages — install via:
  ```bash
  pip install -r requirements.txt
  ```

> Note: The `ros2 joy` package is **no longer required on the host** — PS5 controller input is now read directly on ESP1 (`DriveSpec.ino`) via Bluepad32 over Bluetooth, and `/joy` is published straight into ROS2 through micro-ROS.

## Package Overview (`ocpkg`)

| Component | Type | Purpose |
|---|---|---|
| `ps5_teleop_node.py` | Node | Converts `/joy` (relayed from ESP1) into raw velocity commands and discrete rotation requests |
| `yaw_manager_node.py` | Node | Uses IMU yaw data (from ESP2) to lock heading and apply precise rotation corrections |
| `pid_controller_node.py` | Node | Per-wheel PID velocity control using encoder feedback (from ESP1); publishes motor commands and odometry |
| `mechanism_node.py` | Node | Converts `/joy` (relayed from ESP1) into `/mechanism_cmds` for ESP3 |
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

2. **Flash the firmware** — each `.ino` in `INOs/` goes to its own physical ESP32:
   - `DriveSpec.ino` → ESP1 (Drive)
   - `IMUspec.ino` → ESP2 (IMU)
   - `Mech.ino` → ESP3 (Mechanism)

3. **Start three micro-ROS agents**, one per ESP32 (run each in its own terminal, or as separate background processes):
   ```bash
   # ESP1 — Drive (motors, encoders, PS5 controller via Bluepad32)
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

   # ESP2 — IMU
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -b 115200

   # ESP3 — Mechanism (servos, gripper, actuator, lead screw)
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB2 -b 115200
   ```
   All three must be up and connected (agent state `AGENT_CONNECTED`) before launching the main stack, otherwise the corresponding topics simply won't populate.

4. **Launch the main control stack**:
   ```bash
   ros2 launch ocpkg macnum_control.launch.py
   ```
   This brings up 5 host-side nodes together:
   - `ps5_teleop_node` → reads `/joy` (from ESP1) → publishes `/cmd_vel_raw`, `/rotation_request`
   - `yaw_manager_node` → reads `/cmd_vel_raw`, `/rotation_request`, `/imu/data` (from ESP2) → publishes corrected `/cmd_vel`, `/imu_deg`
   - `pid_controller_node` → reads `/cmd_vel`, `/encoder_ticks` (from ESP1), `/imu/data` (from ESP2) → publishes `/motor_cmds` (to ESP1), `/wheel_distances` (debug), `/odom`
   - `mechanism_node` → reads `/joy` (from ESP1) → publishes `/mechanism_cmds` (to ESP3)
   - `signal_display_node` → Tkinter GUI, launched in `--mode color`

### Data Flow Diagram
```
                          PS5 Controller (Bluetooth)
                                  │
                                  ▼
                    ┌──────────────────────────────┐
                    │ ESP1 — Drive (DriveSpec.ino)  │
                    │ Bluepad32 + Encoders + Motors │
                    └──────────────────────────────┘
                       │            │           ▲
                  /joy │   /encoder_ticks   /motor_cmds
                       │            │           │
        (micro-ROS agent @ /dev/ttyUSB0, 115200)│
                       ▼            ▼           │
        ┌──────────────────┐  ┌─────────────────┴────┐
        │  ps5_teleop_node │  │  pid_controller_node  │
        │  mechanism_node  │  │ (also reads /imu/data)│
        └──────────────────┘  └───────────────────────┘
             │        │                  │      │
   /cmd_vel_raw   /mechanism_cmds   /motor_cmds  /odom, /wheel_distances
   /rotation_req       │             (loop back to ESP1)
             │         ▼
             │   (micro-ROS agent @ /dev/ttyUSB2, 115200)
             │         │
             │         ▼
             │   ┌──────────────────────────────┐
             │   │ ESP3 — Mechanism (Mech.ino)   │
             │   │ Servos / Gripper / Actuator   │
             │   │ Lead Screw / Linear Rhino     │
             │   └──────────────────────────────┘
             ▼
      yaw_manager_node ◄── /imu/data
             │                 ▲
             ▼                 │
          /cmd_vel   (micro-ROS agent @ /dev/ttyUSB1, 115200)
             │                 │
             ▼        ┌──────────────────────────┐
    pid_controller_node   │ ESP2 — IMU (IMUspec.ino) │
                           │ MPU6050 @ 50 Hz          │
                           └──────────────────────────┘
```

## Notes

- Three micro-ROS agents are intentionally **not** included in the launch file — start all three manually before launching, since each depends on its own serial port.
- `ros2 run joy joy_node` is **no longer required**: PS5 input is read directly on ESP1 via Bluepad32 and published as `/joy` through micro-ROS.
- `Mech.ino`'s `/joy` subscription is debug-only (empty callback) — mechanism control is actually driven by `/mechanism_cmds` from the host-side `mechanism_node.py`.
- Diagnostic scripts (`encoder_calibration.py`, `imu_accuracy_diagnostic.py`, `motor_voltage_diagnostic.py`, `natural_bias_test.py`) are meant to be run standalone for calibration/tuning, not as part of normal operation.
- Confirm actual `/dev/ttyUSBx` port assignment per ESP with `ls /dev/ttyUSB*` before starting the agents — enumeration order isn't guaranteed to match physical plug order across reboots.
