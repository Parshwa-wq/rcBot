#!/usr/bin/python3
"""
Motor Voltage & Deadband Diagnostic Tool (V4)
==============================================
Instructions: 
1. STOP ALL ROS NODES (Close the launch file).
2. Start ONLY the micro-ROS agent.
3. Run this script.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import time
import threading

class MotorDiagnostic(Node):
    def __init__(self):
        super().__init__('motor_diagnostic_tool')
        
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/motor_cmds', 10)
        self.create_subscription(Int32MultiArray, '/encoder_ticks', self.encoder_callback, 10)
        
        self.current_ticks = [0, 0, 0, 0]
        self.names = ["FL (Idx 0)", "FR (Idx 1)", "BL (Idx 2)", "BR (Idx 3)"]
        
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

        print("\n" + "!"*60)
        print(" IMPORTANT: ENSURE YOUR LAUNCH FILE IS STOPPED ")
        print("!"*60)
        input("\n>>> Press ENTER to start the test...")

    def encoder_callback(self, msg):
        if len(msg.data) >= 4:
            self.current_ticks = list(msg.data)

    def send_cmd(self, motor_idx, val):
        msg = Float32MultiArray()
        # We send EXACTLY 4 values to satisfy the ESP32 msg->data.size >= 4 check
        data = [0.0, 0.0, 0.0, 0.0]
        data[motor_idx] = float(val)
        msg.data = data
        self.cmd_pub.publish(msg)

    def run_diagnostic(self):
        results = []
        
        for i in range(4):
            print(f"\n[TESTING {self.names[i]}]")
            
            # Reset
            self.send_cmd(0, 0.0)
            time.sleep(1.0)
            
            vel_cmd = 0.0
            start_ticks = self.current_ticks[i]
            woke_up = False
            
            # Increased limit to 2.5 m/s (approx 40% power)
            while rclpy.ok() and vel_cmd < 2.5:
                self.send_cmd(i, vel_cmd)
                time.sleep(0.1)
                
                # Check for movement
                diff = abs(self.current_ticks[i] - start_ticks)
                if diff > 15:
                    print(f"  >>> {self.names[i]} WOKE UP AT: {vel_cmd:.2f} m/s")
                    results.append((self.names[i], round(vel_cmd, 2)))
                    woke_up = True
                    break
                
                vel_cmd += 0.05
                print(f"  Commanding {vel_cmd:.2f} m/s... (Ticks: {self.current_ticks[i]})", end="\r")
            
            if not woke_up:
                print(f"  FAILED: {self.names[i]} never moved.")
                results.append((self.names[i], "FAILED"))
            
            # Stop motor and wait
            self.send_cmd(i, 0.0)
            time.sleep(1.0)

        # FINAL TABLE
        print("\n" + "="*40)
        print(" RESULTS ")
        print("="*40)
        for name, val in results:
            print(f"{name:<15}: {val}")
        print("="*40)

def main():
    rclpy.init()
    node = MotorDiagnostic()
    try:
        node.run_diagnostic()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
