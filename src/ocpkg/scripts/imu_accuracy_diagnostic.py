#!/usr/bin/python3
"""
IMU Scale Diagnostic Tool (V2)
===============================
Listens directly to /imu/data to verify rotation accuracy.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time
import threading

class IMUScaleTest(Node):
    def __init__(self):
        super().__init__('imu_scale_tool')
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.current_yaw = 0.0
        
        # Start a background thread to keep rclpy spinning
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

        print("\n" + "="*60)
        print(" IMU 90-DEGREE SCALE TEST ")
        print("="*60)

    def imu_callback(self, msg):
        # Convert Quaternion to Yaw (Degrees)
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def run_test(self):
        print("\n1. Align robot against Wall A.")
        input(">>> Press ENTER when ready...")
        
        baseline = self.current_yaw
        print(f"  Baseline: {baseline:.2f}°")
        
        print("\n2. Rotate robot exactly 90 degrees to Wall B.")
        input(">>> Press ENTER when rotation is complete...")
        
        final = self.current_yaw
        diff = abs(final - baseline)
        if diff > 180: diff = 360 - diff
        
        print(f"\nRESULTS:")
        print(f"  Physical Turn: 90.00°")
        print(f"  IMU Reported : {diff:.2f}°")
        print(f"  Error        : {abs(90.0 - diff):.2f}°")
        
        if abs(90.0 - diff) > 1.5:
            print(f"\n[SCALE MULTIPLIER]: {90.0 / diff:.4f}")
        else:
            print("\n[RESULT]: Perfect Calibration.")

def main():
    rclpy.init()
    node = IMUScaleTest()
    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
