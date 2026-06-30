#!/usr/bin/python3
"""
Natural Bias Diagnostic Tool (V1)
==================================
Part 4 of the PID Journey: "The Weight & Balance"

This script commands all motors at the EXACT SAME velocity (Open-Loop)
to measure how much the physical hardware veers or rotates on its own.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import Imu
import math
import time
import threading

class BiasDiagnostic(Node):
    def __init__(self):
        super().__init__('bias_diagnostic_tool')
        
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/motor_cmds', 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Int32MultiArray, '/encoder_ticks', self.encoder_callback, 10)
        
        self.current_yaw = 0.0
        self.current_ticks = [0,0,0,0]
        self.is_active = False
        
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

        print("\n" + "="*60)
        print(" NATURAL BIAS & WEIGHT DIAGNOSTIC ")
        print("="*60)
        print("Ensure you have 2 meters of clear space ahead.")
        input(">>> Press ENTER to start the 3-second run...")

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def encoder_callback(self, msg):
        if len(msg.data) >= 4:
            self.current_ticks = list(msg.data)

    def run_test(self):
        # 1. Capture Start State
        start_yaw = self.current_yaw
        start_ticks = self.current_ticks.copy()
        
        print("\nRunning... (Motors at 1.5 m/s)")
        
        # 2. Command Forward (Standard 4-wheel Omni signs for Forward)
        # Using same magnitude for all wheels to test raw electrical/mechanical sync
        # Forward Mapping: FL=-1, FR=+1, BL=-1, BR=+1
        msg = Float32MultiArray()
        msg.data = [-1.5, 1.5, -1.5, 1.5]
        
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < 3.0:
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
            
        # 3. Stop and Analyze
        stop_msg = Float32MultiArray()
        stop_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.cmd_pub.publish(stop_msg)
        
        end_yaw = self.current_yaw
        end_ticks = self.current_ticks
        
        # Calculate angular drift
        total_drift_deg = math.degrees(self.normalize_angle(end_yaw - start_yaw))
        drift_rate = total_drift_deg / 3.0 # deg/sec
        
        # Calculate wheel distance variance
        diffs = [abs(end_ticks[i] - start_ticks[i]) for i in range(4)]
        avg_ticks = sum(diffs) / 4
        variance = [round((d / avg_ticks - 1.0) * 100, 1) for d in diffs]

        print("\n" + "="*40)
        print(" BIAS REPORT ")
        print("="*40)
        print(f"Total Angular Drift  : {total_drift_deg:.2f} degrees")
        print(f"Angular Drift Rate   : {drift_rate:.2f} deg/sec")
        print("-" * 40)
        print("Wheel Power Sync (Difference from Average):")
        names = ["FL", "FR", "BL", "BR"]
        for i in range(4):
            print(f"  {names[i]:<4}: {variance[i]:>+5}% ticks")
        
        print("\n[ANALYSIS]")
        if abs(drift_rate) < 1.0:
            print("1. Your robot is physically balanced. Excellent base!")
        else:
            side = "Left" if drift_rate > 0 else "Right"
            print(f"1. Robot naturally veers to the {side}.")
            print(f"   Bias offset: {drift_rate:.3f} rad/s")
            
        print("="*40 + "\n")

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

def main():
    rclpy.init()
    node = BiasDiagnostic()
    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
