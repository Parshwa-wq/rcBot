# Header for timestamp
std_msgs/Header header

int32 front_left    # Front left wheel encoder ticks
int32 front_right   # Front right wheel encoder ticks  
int32 back_left     # Back left wheel encoder ticks
int32 back_right    # Back right wheel encoder ticks

# Optional: timestamp from ESP32 (microseconds since boot)
uint32 esp_timestamp_us
