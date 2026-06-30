#!/usr/bin/python3
import argparse
import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import threading

try:
    import cv2
    import numpy as np
    from PIL import Image, ImageTk
    _HAVE_CV = True
except ImportError:
    _HAVE_CV = False

IDLE_COLOR = "#101010"
DONE_COLOR = "#FF0000"

class SignalDisplay(Node):
    def __init__(self, mode="marker", marker_id=23, blink_hz=5.0):
        super().__init__('signal_display_node')
        self.mode = mode
        self.marker_id = marker_id
        self.blink_hz = blink_hz
        self.state = "idle"
        self.target_state = "idle"
        self._blink_on = False
        self.ui_available = False

        # Create ROS subscriber
        self.subscription = self.create_subscription(
            Bool,
            '/display_status',
            self.listener_callback,
            10
        )
        self.get_logger().info('Signal Display Node started, listening to /display_status')

        # Try to initialize the GUI (will fail if display is not connected)
        try:
            self.root = tk.Tk()
            self.root.attributes("-fullscreen", True)
            self.root.configure(bg=IDLE_COLOR)

            self.canvas = tk.Canvas(self.root, highlightthickness=0, bg=IDLE_COLOR)
            self.canvas.pack(fill="both", expand=True)

            self._marker_image = None
            if self.mode == "marker":
                self._marker_image = self._make_marker_image()

            self.root.bind("<Key>", self._on_key)
            
            # Hide the window at startup until the signal is True
            self.root.withdraw()

            self.ui_available = True
            self._render()
            self._tick()
        except Exception as e:
            self.get_logger().error(f"Failed to initialize display: {e}")
            self.get_logger().warn("Running in headless mode! Display commands will be ignored.")
            self.ui_available = False

    def listener_callback(self, msg):
        if not self.ui_available:
            return
            
        if msg.data:
            self.target_state = "done"
        else:
            self.target_state = "idle"

    # ---- marker generation -------------------------------------------------
    def _make_marker_image(self):
        if not _HAVE_CV:
            raise RuntimeError("opencv-python and pillow are required for marker mode")
        screen_h = self.root.winfo_screenheight()
        side = int(screen_h * 0.8)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            marker_img = cv2.aruco.generateImageMarker(aruco_dict, self.marker_id, side)
        except AttributeError:
            marker_img = cv2.aruco.drawMarker(aruco_dict, self.marker_id, side)
        margin = int(side * 0.15)
        canvas_img = 255 * np.ones((side + 2 * margin, side + 2 * margin), dtype=np.uint8)
        canvas_img[margin:margin + side, margin:margin + side] = marker_img
        pil_img = Image.fromarray(canvas_img)
        return ImageTk.PhotoImage(pil_img)

    # ---- state control ----------------------------------------------------
    def set_done(self):
        if not self.ui_available: return
        self.state = "done"
        self.root.deiconify()
        self.root.attributes("-topmost", True)
        self._render()

    def set_idle(self):
        if not self.ui_available: return
        self.state = "idle"
        self._blink_on = False
        self.root.withdraw()
        self._render()

    # ---- internals ----------------------------------------------------------
    def _on_key(self, event):
        if event.char == "q":
            self.root.destroy()
            rclpy.shutdown()

    def _render(self):
        if not self.ui_available: return
        self.canvas.delete("all")
        if self.mode == "color":
            color = DONE_COLOR if self.state == "done" else IDLE_COLOR
            self.canvas.configure(bg=color)

        elif self.mode == "blink":
            if self.state == "done":
                color = DONE_COLOR if self._blink_on else "#000000"
            else:
                color = IDLE_COLOR
            self.canvas.configure(bg=color)

        elif self.mode == "marker":
            self.canvas.configure(bg="#000000")
            if self.state == "done":
                w = self.root.winfo_screenwidth()
                h = self.root.winfo_screenheight()
                self.canvas.create_image(w // 2, h // 2, image=self._marker_image)

    def _tick(self):
        if not self.ui_available: return
        
        # Apply target_state changes safely on the main thread
        if self.state != self.target_state:
            if self.target_state == "done":
                self.set_done()
            else:
                self.set_idle()
                
        if self.mode == "blink" and self.state == "done":
            self._blink_on = not self._blink_on
            self._render()
            delay_ms = int(1000 / (2 * self.blink_hz))
        else:
            delay_ms = 100
        self.root.after(delay_ms, self._tick)

    def run(self):
        if self.ui_available:
            self.root.mainloop()
        else:
            # Sleep endlessly so the background ROS thread stays alive
            import time
            while rclpy.ok():
                time.sleep(1)

def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["color", "blink", "marker"], default="marker")
    parser.add_argument("--marker-id", type=int, default=23)
    parser.add_argument("--blink-hz", type=float, default=5.0)
    args, unknown = parser.parse_known_args()

    display = SignalDisplay(mode=args.mode, marker_id=args.marker_id, blink_hz=args.blink_hz)
    
    # Run ROS spin in a background thread so it doesn't block tkinter
    thread = threading.Thread(target=ros_spin_thread, args=(display,), daemon=True)
    thread.start()
    
    display.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
