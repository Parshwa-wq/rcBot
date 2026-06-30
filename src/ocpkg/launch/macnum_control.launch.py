#!/usr/bin/python3
"""
Mecanum Bot Control Launch File
================================
Launches all required nodes for mecanum drive control:
- PS5 Teleop Node: Joystick to cmd_vel conversion
- Yaw Manager Node: IMU yaw degrees, heading lock, and precise rotation requests
- PID Controller Node: Velocity control with yaw correction
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('ocpkg')

    # ============================================
    # LAUNCH ARGUMENTS - TUNE THESE AS NEEDED
    # ============================================

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # ============================================
    # CONFIG FILE PATHS
    # ============================================

    pid_config = os.path.join(pkg_share, 'config', 'pid_gains.yaml')
    robot_config = os.path.join(pkg_share, 'config', 'robot_params.yaml')
    teleop_config = os.path.join(pkg_share, 'config', 'teleop_params.yaml')

    # ============================================
    # PS5 TELEOP NODE
    # ============================================

    ps5_teleop_node = Node(
        package='ocpkg',
        executable='ps5_teleop_node.py',
        name='ps5_teleop_node',
        output='screen',
        parameters=[
            teleop_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/joy', '/joy'),           # Input: joystick data
            ('/cmd_vel_raw', '/cmd_vel_raw'),   # Output: raw joystick velocity commands
            ('/rotation_request', '/rotation_request'),   # Output: requested yaw step in degrees
        ],
        # respawn=True,                 # Uncomment to auto-restart on crash
        # respawn_delay=2.0,            # Delay before respawn (seconds)
    )

    yaw_manager_node = Node(
        package='ocpkg',
        executable='yaw_manager_node.py',
        name='yaw_manager_node',
        output='screen',
        parameters=[
            pid_config,
            robot_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel_raw', '/cmd_vel_raw'),       # Input: raw joystick velocity commands
            ('/rotation_request', '/rotation_request'),  # Input: discrete rotation requests
            ('/imu/data', '/imu/data'),             # Input: IMU orientation
            ('/cmd_vel', '/cmd_vel'),               # Output: yaw-corrected velocity commands
            ('/imu_deg', '/imu_deg'),               # Output: current yaw in degrees
        ],
    )

    mechanism_node = Node(
        package='ocpkg',
        executable='mechanism_node.py',
        name='mechanism_node',
        output='screen',
        parameters=[
            teleop_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/joy', '/joy'),
            ('/mechanism_cmds', '/mechanism_cmds'),
        ],
    )

    # ============================================
    # PID CONTROLLER NODE
    # ============================================

    pid_controller_node = Node(
        package='ocpkg',
        executable='pid_controller_node.py',
        name='pid_controller_node',
        output='screen',
        parameters=[
            pid_config,
            robot_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),                   # Input: velocity commands
            ('/encoder_ticks', '/encoder_ticks'),       # Input: encoder data from ESP32
            ('/imu/data', '/imu/data'),                 # Input: IMU data from ESP32
            ('/motor_cmds', '/motor_cmds'),             # Output: motor velocities to ESP32
            ('/wheel_distances', '/wheel_distances'),   # Output: debug info
            ('/odom', '/odom'),                         # Output: odometry
        ],
        # respawn=True,
        # respawn_delay=2.0,
    )

    # ============================================
    # MICRO-ROS AGENT (Optional - run separately)
    # ============================================
    # Note: Usually run manually with:
    # ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

    signal_display_node = Node(
        package='ocpkg',
        executable='signal_display_node.py',
        name='signal_display_node',
        output='screen',
        arguments=['--mode', 'color']
    )

    # ============================================
    # LAUNCH DESCRIPTION
    # ============================================

    return LaunchDescription([
        declare_use_sim_time,
        ps5_teleop_node,
        mechanism_node,
        yaw_manager_node,
        pid_controller_node,
        signal_display_node,
    ])
