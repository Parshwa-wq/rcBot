#!/usr/bin/python3
"""
Encoder Distance Calibration Tool (V1)
======================================
Part 2 of the PID Journey: "The Distance"

Instructions:
1. Mark a START line and a FINISH line exactly 1.0 meter apart on the floor.
2. Align the robot's center or a wheel with the START line.
3. Run this script.
4. MANUALLY push the robot slowly to the FINISH line.
5. Press Ctrl+C to see the final calibration report.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import math

class EncoderCalibration(Node):
    def __init__(self):
        super().__init__('encoder_calibration_tool')
        
        self.create_subscription(Int32MultiArray, '/encoder_ticks', self.encoder_callback, 10)
        
        # Physical Parameters updated for calibration
        self.declare_parameter('wheel_radius', 0.1121)
        self.declare_parameter('encoder_ticks_per_rev', 1620)
        
        self.r = self.get_parameter('wheel_radius').value
        self.ppr = self.get_parameter('encoder_ticks_per_rev').value
        
        self.start_ticks = None
        self.current_ticks = [0, 0, 0, 0]
        self.names = ["FL", "FR", "BL", "BR"]
        
        print("\n" + "="*60)
        print(" ENCODER DISTANCE CALIBRATION ")
        print("="*60)
        print(f"Current Config: Radius={self.r}m, PPR={self.ppr}")
        print("1. Place robot at the 0cm mark.")
        print("2. Slowly push the robot to the 100cm (1 meter) mark.")
        print("3. Press Ctrl+C when finished to see results.")
        print("="*60 + "\n")

    def encoder_callback(self, msg):
        if len(msg.data) >= 4:
            self.current_ticks = list(msg.data)
            if self.start_ticks is None:
                self.start_ticks = self.current_ticks.copy()
                print(">>> Baseline captured. Start pushing now!")

    def calculate_results(self):
        if self.start_ticks is None:
            print("Error: No encoder data received!")
            return

        print("\n" + "-"*40)
        print(" FINAL CALIBRATION REPORT (1.0m Test) ")
        print("-"*40)
        print(f"{'MOTOR':<10} | {'TICKS':<10} | {'CALC DISTANCE'}")
        
        wheel_circum = 2 * math.pi * self.r
        
        results = []
        for i in range(4):
            total_ticks = abs(self.current_ticks[i] - self.start_ticks[i])
            # Distance = (Ticks / PPR) * Circumference
            dist = (total_ticks / self.ppr) * wheel_circum
            print(f"{self.names[i]:<10} | {total_ticks:<10} | {dist:.3f} meters")
            results.append(dist)

        print("-"*40)
        avg_dist = sum(results) / 4
        print(f"Average Measured Distance: {avg_dist:.3f} meters")
        
        if abs(1.0 - avg_dist) > 0.05:
            correction = 1.0 / avg_dist
            new_radius = self.r * correction
            print(f"\n[ACTION REQUIRED]")
            print(f"Your encoders are off by {abs(1.0-avg_dist)*100:.1f}%.")
            print(f"Suggested new wheel_radius: {new_radius:.5f}")
        else:
            print("\n[RESULT] Your encoder distance is within 5% accuracy. Good job!")
        print("="*40 + "\n")

def main():
    rclpy.init()
    node = EncoderCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.calculate_results()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
