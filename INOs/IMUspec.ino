/*
 * ╔══════════════════════════════════════════════════╗
 * ║      IMU ESP32 — MPU6050 WITH MICRO-ROS          ║
 * ║        Publishing /imu/data at 50 Hz             ║
 * ╚══════════════════════════════════════════════════╝
 */

#include <Wire.h>
#include <math.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

// ─── MPU6050 ────────────────────────────────────────
#define MPU6050_ADDR        0x68
#define ACCEL_SENSITIVITY   16384.0f   // ±2g
#define GYRO_SENSITIVITY    129.0f     // ±250 deg/s
#define CALIBRATION_SAMPLES 3000
#define CALIBRATION_LOOP_MS 3
#define FILTER_COEFF        0.95f
#define IMU_PUBLISH_HZ      200

// ─── I2C ─────────────────────────────────────────────
#define I2C_SDA  21
#define I2C_SCL  22

// ─── Pre-computed constants ──────────────────────────
const float G_TO_MS2     = 9.81f;
const float DEG_TO_RAD_F = M_PI / 180.0f;
const float RAD_TO_DEG_F = 180.0f / M_PI;
const float GYRO_SCALE   = (1.0f / GYRO_SENSITIVITY) * DEG_TO_RAD_F;
const float ACCEL_SCALE  = (1.0f / ACCEL_SENSITIVITY) * G_TO_MS2;

// ─── State ───────────────────────────────────────────
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float ax_offset = 0, ay_offset = 0, az_offset = 0;

unsigned long last_time = 0;
float dt = 0.01f;

static int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
bool calibrated    = false;
bool microros_ready = false;

// ─── micro-ROS ───────────────────────────────────────
rcl_allocator_t allocator;
rclc_support_t  support;
rcl_node_t      node;
rclc_executor_t executor;
rcl_publisher_t imu_publisher;
rcl_timer_t     imu_timer;
sensor_msgs__msg__Imu imu_msg;

// ─── MPU6050 Helpers ─────────────────────────────────
inline void readRawData() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (bool)true);
    ax_r = (Wire.read() << 8) | Wire.read();
    ay_r = (Wire.read() << 8) | Wire.read();
    az_r = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();  // skip temp
    gx_r = (Wire.read() << 8) | Wire.read();
    gy_r = (Wire.read() << 8) | Wire.read();
    gz_r = (Wire.read() << 8) | Wire.read();
}

void calibrateMPU6050() {
    Serial.println("Calibrating MPU6050...");
    delay(2000);
    long ax_s=0, ay_s=0, az_s=0, gx_s=0, gy_s=0, gz_s=0;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        readRawData();
        ax_s += ax_r; ay_s += ay_r; az_s += az_r;
        gx_s += gx_r; gy_s += gy_r; gz_s += gz_r;
        delay(CALIBRATION_LOOP_MS);
    }
    ax_offset = (float)ax_s / CALIBRATION_SAMPLES;
    ay_offset = (float)ay_s / CALIBRATION_SAMPLES;
    az_offset = (float)az_s / CALIBRATION_SAMPLES - 16384.0f;
    gx_offset = (float)gx_s / CALIBRATION_SAMPLES;
    gy_offset = (float)gy_s / CALIBRATION_SAMPLES;
    gz_offset = (float)gz_s / CALIBRATION_SAMPLES;
    calibrated = true;
    Serial.println("Calibration DONE!");
}

// ─── IMU Timer Callback ──────────────────────────────
void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (!calibrated || !microros_ready) return;

    readRawData();
    float ax = (ax_r - ax_offset) * ACCEL_SCALE;
    float ay = (ay_r - ay_offset) * ACCEL_SCALE;
    float az = (az_r - az_offset) * ACCEL_SCALE;
    float gx = (gx_r - gx_offset) * GYRO_SCALE;
    float gy = (gy_r - gy_offset) * GYRO_SCALE;
    float gz = (gz_r - gz_offset) * GYRO_SCALE;

    unsigned long now = micros();
    dt = (now - last_time) * 1e-6f;
    if (dt > 0.05f) dt = 0.005f;
    last_time = now;

    // Complementary filter
    float acc_roll  = atan2(ay, az) * RAD_TO_DEG_F;
    float acc_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG_F;
    roll  = FILTER_COEFF * (roll  + (gx * RAD_TO_DEG_F) * dt) + (1.0f - FILTER_COEFF) * acc_roll;
    pitch = FILTER_COEFF * (pitch + (gy * RAD_TO_DEG_F) * dt) + (1.0f - FILTER_COEFF) * acc_pitch;
    yaw  += (gz * RAD_TO_DEG_F) * dt;

    // Euler → quaternion
    float r = roll  * 0.5f * DEG_TO_RAD_F;
    float p = pitch * 0.5f * DEG_TO_RAD_F;
    float y = yaw   * 0.5f * DEG_TO_RAD_F;
    float cr=cos(r), sr=sin(r), cp=cos(p), sp=sin(p), cy=cos(y), sy=sin(y);
    imu_msg.orientation.w = cr*cp*cy + sr*sp*sy;
    imu_msg.orientation.x = sr*cp*cy - cr*sp*sy;
    imu_msg.orientation.y = cr*sp*cy + sr*cp*sy;
    imu_msg.orientation.z = cr*cp*sy - sr*sp*cy;

    imu_msg.angular_velocity.x    = gx;
    imu_msg.angular_velocity.y    = gy;
    imu_msg.angular_velocity.z    = gz;
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    uint32_t t = millis();
    imu_msg.header.stamp.sec     = t / 1000;
    imu_msg.header.stamp.nanosec = (t % 1000) * 1000000UL;

    rcl_publish(&imu_publisher, &imu_msg, NULL);
}

// ─── micro-ROS Setup ─────────────────────────────────
bool setup_micro_ros() {
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;

    rclc_node_init_default(&node, "imu_esp32_node", "", &support);

    static char frame_id[] = "imu_link";
    imu_msg.header.frame_id.data     = frame_id;
    imu_msg.header.frame_id.size     = strlen(frame_id);
    imu_msg.header.frame_id.capacity = sizeof(frame_id);

    rclc_publisher_init_best_effort(
        &imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"
    );

    rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(1000 / IMU_PUBLISH_HZ), imu_timer_callback);
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &imu_timer);

    microros_ready = true;
    return true;
}

// ─── Setup & Loop ────────────────────────────────────
void setup() {
    Serial.begin(115200); // Reverted to 115200 to match default micro-ros agent baud rate
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); Wire.write(0x00);
    Wire.endTransmission();

    calibrateMPU6050();
    setup_micro_ros();
    last_time = micros();
}

void loop() {
    if (microros_ready) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    }
}
