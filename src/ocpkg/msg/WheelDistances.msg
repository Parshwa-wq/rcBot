# WheelDistances.msg
# Distance traveled by each wheel in centimeters
# Used for odometry debugging and PID tuning
# Published at same rate as encoder updates

# Header for timestamp
std_msgs/Header header

# Cumulative distance traveled (cm)
# Resets when node starts or on request
#
# Wheel positions:
#   front_left (FL)  |  front_right (FR)
#   -----------------|-----------------
#   back_left (BL)   |  back_right (BR)

float32 front_left_cm    # Front left wheel distance (cm)
float32 front_right_cm   # Front right wheel distance (cm)
float32 back_left_cm     # Back left wheel distance (cm)
float32 back_right_cm    # Back right wheel distance (cm)

# Total distance (average of all wheels)
float32 total_distance_cm

# Individual wheel velocities (cm/s) - for debugging
float32 front_left_vel
float32 front_right_vel
float32 back_left_vel
float32 back_right_vel
