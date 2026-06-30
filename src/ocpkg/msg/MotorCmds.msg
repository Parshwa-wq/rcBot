# MotorCmds.msg
# Motor velocity commands sent to ESP32
# Values are in m/s (wheel surface velocity)
# ESP32 converts to PWM duty cycle

# Header for timestamp
std_msgs/Header header

# Wheel velocity commands (m/s)
# Positive = wheel rotates to move robot forward
# Negative = wheel rotates to move robot backward
#
# Wheel positions:
#   front_left (FL)  |  front_right (FR)
#   -----------------|-----------------
#   back_left (BL)   |  back_right (BR)

float32 front_left   # Front left wheel velocity (m/s)
float32 front_right  # Front right wheel velocity (m/s)
float32 back_left    # Back left wheel velocity (m/s)
float32 back_right   # Back right wheel velocity (m/s)

# Optional: Enable/disable motors
bool motors_enabled
