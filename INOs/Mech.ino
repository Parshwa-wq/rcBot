/**
 * ╔══════════════════════════════════════════════════════════════╗
 * ║           ESP3 — MECHANISMS CONTROLLER (Attempt 3)          ║
 * ║                                                              ║
 * ║  Changes from Attempt 2:                                     ║
 * ║    1. SERVO1 and SERVO2 separated onto individual pins.      ║
 * ║       Attempt 2: both on pin 13 (shared ch 0).              ║
 * ║       Attempt 3: SERVO1 = pin 13 (ch 0)                     ║
 * ║                  SERVO2 = pin 12 (ch 6) — new               ║
 * ║       data[0] still drives BOTH at same angle.              ║
 * ║    2. Added LINEAR_RHINO motor (MD30C, MCPWM Unit 1 A).     ║
 * ║       Pins: PWM=32, DIR=33. data[7] accepts signed velocity. ║
 * ║       Normal hold runs forward; shift-first hotkey reverses. ║
 * ║       Release stops.                                        ║
 * ║    3. MECH_NUM_CMDS: 7 → 8                                  ║
 * ║                                                              ║
 * ║  /mechanism_cmds data[] index map:                           ║
 * ║  ─────────────────────────────────────────────────────────── ║
 * ║  [0] Servo 1+2 angle  (0-180 deg)   — 160 kg pair           ║
 * ║  [1] Servo 3 angle    (0-180 deg)   — 160 kg single         ║
 * ║  [2] Servo 4 angle    (0-180 deg)   — 60 kg #1              ║
 * ║  [3] Servo 5 angle    (0-180 deg)   — 60 kg #2              ║
 * ║  [4] Linear Actuator  (-1/0/+1)     — MDDRC10 ch 4 pin 23   ║
 * ║  [5] Pneumatic Gripper(>0.5=ON)     — MDDRC10 ch 5 pin 26   ║
 * ║  [6] Lead Screw vel   (-1.0 to +1.0)— MD30C MCPWM Unit0 A   ║
 * ║  [7] Linear Rhino vel (-1.5 to 1.5) — MD30C MCPWM Unit1 A   ║
 * ║                                                              ║
 * ║  LEDC Channel Map:                                           ║
 * ║    ch 0 → SERVO1    pin 13  (6N137 inverted)                ║
 * ║    ch 1 → SERVO3    pin 14  (6N137 inverted)                ║
 * ║    ch 2 → SERVO4    pin 25  (6N137 inverted)                ║
 * ║    ch 3 → SERVO5    pin 27  (6N137 inverted)                ║
 * ║    ch 4 → LINEAR_ACT pin 23 (MDDRC10 direct)                ║
 * ║    ch 5 → GRIPPER   pin 26  (MDDRC10 direct)                ║
 * ║    ch 6 → SERVO2    pin 12  (6N137 inverted) — NEW          ║
 * ╚══════════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/joy.h>
#include "driver/mcpwm.h"

// ════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ════════════════════════════════════════════════════════════════

// Servos (6N137 optocoupler — signal inverted in writeServo())
#define SERVO1_PIN      13   // 160 kg — individual pin (was shared in Attempt 2)
#define SERVO2_PIN      12   // 160 kg — new separate pin (NEW in Attempt 3)
#define SERVO3_PIN      4   // 160 kg single
#define SERVO4_PIN      25   // 60 kg #1
#define SERVO5_PIN      22   // 60 kg #2

// MDDRC10 — RC signal direct (no invert)
#define LINEAR_ACT_PIN  23   // Linear Actuator
#define GRIPPER_PIN     26   // Pneumatic Gripper

// MD30C — Lead Screw (MCPWM Unit 0 A)
#define LEAD_SCREW_PWM  18
#define LEAD_SCREW_DIR  19

// MD30C — Linear Rhino (MCPWM Unit 1 A) — NEW in Attempt 3
#define RHINO_PWM       32
#define RHINO_DIR       33

// Status LED
#define LED_PIN         2

// ════════════════════════════════════════════════════════════════
// LEDC CHANNEL MAP
// ════════════════════════════════════════════════════════════════
#define SERVO1_CH       0    // pin 13 — 6N137 inverted
#define SERVO3_CH       1    // pin 14 — 6N137 inverted
#define SERVO4_CH       2    // pin 25 — 6N137 inverted
#define SERVO5_CH       3    // pin 27 — 6N137 inverted
#define LINEAR_ACT_CH   4    // pin 23 — MDDRC10 direct
#define GRIPPER_CH      5    // pin 26 — MDDRC10 direct
#define SERVO2_CH       6    // pin 12 — 6N137 inverted (NEW)

// ════════════════════════════════════════════════════════════════
// SHARED PWM CONFIG
// ════════════════════════════════════════════════════════════════
#define LEDC_FREQ_HZ    50
#define LEDC_RES_BITS   16
#define LEDC_PERIOD_US  20000   // 20 ms at 50 Hz

// ════════════════════════════════════════════════════════════════
// MDDRC10 RC PULSE VALUES
//   2000 us -> forward / ON
//   1500 us -> stop / OFF
//   1000 us -> reverse
// ════════════════════════════════════════════════════════════════
#define MDDRC10_FWD_US  2000
#define MDDRC10_OFF_US  1500
#define MDDRC10_REV_US  1000

// ════════════════════════════════════════════════════════════════
// SERVO SLEW RATE
// ════════════════════════════════════════════════════════════════
#define SERVO_TICK_MS   15     // ms between servo updates
#define SERVO_SLEW_DEG  6      // max degrees moved per tick  (2× from default 3 → 400 deg/sec)

// ════════════════════════════════════════════════════════════════
// LEAD SCREW (MCPWM Unit 0 A)
// ════════════════════════════════════════════════════════════════
#define LEAD_SCREW_PWM_FREQ   20000
#define LEAD_SCREW_MAX_VEL    3.0f
#define LEAD_SCREW_DEADBAND   0.05f

// ════════════════════════════════════════════════════════════════
// LINEAR RHINO (MCPWM Unit 1 A) — NEW
// Signed command (-RHINO_MAX_VEL to +RHINO_MAX_VEL).
// Normal button hold is positive; shift-first hotkey is negative.
// ════════════════════════════════════════════════════════════════
#define RHINO_PWM_FREQ    20000   // 20 kHz, MD30C compatible
#define RHINO_MAX_VEL     1.5f    // matches Python linear_rhino_speed
#define RHINO_DEADBAND    0.05f

// ════════════════════════════════════════════════════════════════
// WATCHDOG
// ════════════════════════════════════════════════════════════════
#define CMD_TIMEOUT_MS  500

// /mechanism_cmds now carries 8 values
#define MECH_NUM_CMDS   8

// ════════════════════════════════════════════════════════════════
// DUAL-CORE SPINLOCK
// ════════════════════════════════════════════════════════════════
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;

// ════════════════════════════════════════════════════════════════
// GLOBAL STATE
// ════════════════════════════════════════════════════════════════

// Servo targets — written Core 1, read Core 0 slew
int target_s1_2 = 90;    // single target drives BOTH servo1 and servo2
int target_s3   = 90;
int target_s4   = 90;
int target_s5   = 90;

// Servo currents — owned exclusively by Core 0
int current_s1   = 90;
int current_s2   = 90;
int current_s3   = 90;
int current_s4   = 90;
int current_s5   = 90;

float linear_actuator_cmd = 0.0f;
bool  pneumatic_on       = false;
float lead_screw_vel     = 0.0f;
float rhino_vel          = 0.0f;   // NEW: signed -1.5 to +1.5

unsigned long last_cmd_ms = 0;

// micro-ROS handles
rcl_allocator_t    allocator;
rclc_support_t     support;
rcl_node_t         node;
rclc_executor_t    executor;
rcl_subscription_t mech_subscriber;
rcl_subscription_t joy_subscriber;

std_msgs__msg__Float32MultiArray mech_msg;
sensor_msgs__msg__Joy            joy_msg;

// Static backing buffers
static float   mech_data[MECH_NUM_CMDS]  = {90.0f, 90.0f, 90.0f, 90.0f,
                                              0.0f,  0.0f,  0.0f, 0.0f};
static float   joy_axes_data[6]          = {0};
static int32_t joy_buttons_data[16]      = {0};

volatile bool microros_ready = false;
volatile bool mech_buf_ready = false;

TaskHandle_t microros_task_handle = NULL;

#define RCCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) return false; }

// ════════════════════════════════════════════════════════════════
// HELPERS
// ════════════════════════════════════════════════════════════════

// us → 16-bit LEDC duty at 50 Hz (period = 20 000 us)
static inline uint32_t usToDuty(uint32_t us) {
    return (us * 65535UL) / LEDC_PERIOD_US;
}

// ════════════════════════════════════════════════════════════════
// SERVO WRITE  (6N137 optocoupler inversion)
// Inverted: duty = 65535 - normalDuty
// ════════════════════════════════════════════════════════════════
void writeServo(uint8_t ch, int deg) {
    deg = constrain(deg, 0, 180);
    uint32_t normalDuty = usToDuty(500 + ((uint32_t)deg * 2000UL / 180UL));
    ledcWrite(ch, 65535UL - normalDuty);
}

// ════════════════════════════════════════════════════════════════
// SERVO SETUP
// Channels 0-3 for SERVO1/3/4/5, channel 6 for SERVO2 (NEW)
// ════════════════════════════════════════════════════════════════
void setupServos() {
    ledcSetup(SERVO1_CH, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcSetup(SERVO2_CH, LEDC_FREQ_HZ, LEDC_RES_BITS);   // NEW ch 6
    ledcSetup(SERVO3_CH, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcSetup(SERVO4_CH, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcSetup(SERVO5_CH, LEDC_FREQ_HZ, LEDC_RES_BITS);

    ledcAttachPin(SERVO1_PIN, SERVO1_CH);
    ledcAttachPin(SERVO2_PIN, SERVO2_CH);   // NEW pin 12
    ledcAttachPin(SERVO3_PIN, SERVO3_CH);
    ledcAttachPin(SERVO4_PIN, SERVO4_CH);
    ledcAttachPin(SERVO5_PIN, SERVO5_CH);

    // Park at 90 deg on startup
    writeServo(SERVO1_CH, 90);
    writeServo(SERVO2_CH, 90);
    writeServo(SERVO3_CH, 90);
    writeServo(SERVO4_CH, 90);
    writeServo(SERVO5_CH, 90);

    Serial.println("[INIT] Servos: ch0 pin13, ch6 pin12, ch1 pin14, ch2 pin25, ch3 pin27");
}

// ════════════════════════════════════════════════════════════════
// MDDRC10 SETUP  (ch 4 pin 23, ch 5 pin 26 — raw LEDC direct)
// ════════════════════════════════════════════════════════════════
void setupMDDRC10() {
    ledcSetup(LINEAR_ACT_CH, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(LINEAR_ACT_PIN, LINEAR_ACT_CH);
    ledcWrite(LINEAR_ACT_CH, usToDuty(MDDRC10_OFF_US));

    ledcSetup(GRIPPER_CH, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(GRIPPER_PIN, GRIPPER_CH);
    ledcWrite(GRIPPER_CH, usToDuty(MDDRC10_OFF_US));

    Serial.println("[INIT] MDDRC10: ch4 pin23 (LinearAct), ch5 pin26 (Gripper)");
}

void setLinearActuator(float cmd) {
    cmd = constrain(cmd, -1.0f, 1.0f);
    uint32_t pulse_us = MDDRC10_OFF_US;
    if (cmd > 0.5f) {
        pulse_us = MDDRC10_FWD_US;
    } else if (cmd < -0.5f) {
        pulse_us = MDDRC10_REV_US;
    }
    ledcWrite(LINEAR_ACT_CH, usToDuty(pulse_us));
    Serial.printf("[ACT]  Linear Actuator -> %.1f\n", cmd);
}

void setPneumaticGripper(bool on) {
    ledcWrite(GRIPPER_CH, usToDuty(on ? MDDRC10_FWD_US : MDDRC10_OFF_US));
    Serial.printf("[GRIP] Pneumatic Gripper -> %s\n", on ? "ON" : "OFF");
}

// ════════════════════════════════════════════════════════════════
// LEAD SCREW — MD30C, MCPWM Unit 0 A, pins 18/19, 20 kHz
// ════════════════════════════════════════════════════════════════
void setupLeadScrew() {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LEAD_SCREW_PWM);
    pinMode(LEAD_SCREW_DIR, OUTPUT);

    mcpwm_config_t cfg;
    cfg.frequency    = LEAD_SCREW_PWM_FREQ;
    cfg.cmpr_a       = 0; cfg.cmpr_b = 0;
    cfg.counter_mode = MCPWM_UP_COUNTER;
    cfg.duty_mode    = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

    Serial.println("[INIT] Lead Screw: MCPWM Unit0 A, pins 18/19");
}

void setLeadScrew(float vel) {
    vel = constrain(vel, -LEAD_SCREW_MAX_VEL, LEAD_SCREW_MAX_VEL);
    if (fabsf(vel) < LEAD_SCREW_DEADBAND) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        return;
    }
    float duty = (fabsf(vel) / LEAD_SCREW_MAX_VEL) * 100.0f;
    duty = constrain(duty, 0.0f, 100.0f);
    digitalWrite(LEAD_SCREW_DIR, vel >= 0.0f ? HIGH : LOW);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

// ════════════════════════════════════════════════════════════════
// LINEAR RHINO — MD30C, MCPWM Unit 1 A, pins 32/33, 20 kHz (NEW)
//
// Uses MCPWM_UNIT_1 so it is completely independent of the lead
// screw (MCPWM_UNIT_0). Same MD30C driver, same PWM frequency.
// Signed direction: positive DIR HIGH, negative DIR LOW.
// ════════════════════════════════════════════════════════════════
void setupLinearRhino() {
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, RHINO_PWM);
    pinMode(RHINO_DIR, OUTPUT);
    digitalWrite(RHINO_DIR, HIGH);   // idle default direction

    mcpwm_config_t cfg;
    cfg.frequency    = RHINO_PWM_FREQ;
    cfg.cmpr_a       = 0; cfg.cmpr_b = 0;
    cfg.counter_mode = MCPWM_UP_COUNTER;
    cfg.duty_mode    = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &cfg);
    mcpwm_start(MCPWM_UNIT_1, MCPWM_TIMER_0);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

    Serial.println("[INIT] Linear Rhino: MCPWM Unit1 A, pins 32/33 (signed)");
}

void setLinearRhino(float vel) {
    vel = constrain(vel, -RHINO_MAX_VEL, RHINO_MAX_VEL);

    if (fabsf(vel) < RHINO_DEADBAND) {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        return;
    }

    float duty = (fabsf(vel) / RHINO_MAX_VEL) * 100.0f;
    duty = constrain(duty, 0.0f, 100.0f);
    digitalWrite(RHINO_DIR, vel >= 0.0f ? HIGH : LOW);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

// ════════════════════════════════════════════════════════════════
// /mechanism_cmds CALLBACK
// Index map (matches mechanism_node.py publish_cmds()):
//   [0] Servo 1+2 angle   (float deg 0-180) → drives SERVO1 AND SERVO2
//   [1] Servo 3 angle     (float deg 0-180)
//   [2] Servo 4 angle     (float deg 0-180)
//   [3] Servo 5 angle     (float deg 0-180)
//   [4] Linear Actuator   (+1.0 forward / -1.0 reverse / 0.0 stop)
//   [5] Pneumatic Gripper (1.0=ON / 0.0=OFF)
//   [6] Lead Screw vel    (-1.0 to +1.0, scaled to ±3.0 m/s)
//   [7] Linear Rhino vel  (-1.5 to +1.5 m/s)                NEW
// ════════════════════════════════════════════════════════════════
void mech_callback(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;

    if (!msg || msg->data.size < MECH_NUM_CMDS) return;

    last_cmd_ms = millis();

    // Servo targets — lroundf() prevents float truncation jitter
    int new_s1_2 = constrain((int)lroundf(msg->data.data[0]), 0, 180);
    int new_s3   = constrain((int)lroundf(msg->data.data[1]), 0, 180);
    int new_s4   = constrain((int)lroundf(msg->data.data[2]), 0, 180);
    int new_s5   = constrain((int)lroundf(msg->data.data[3]), 0, 180);

    taskENTER_CRITICAL(&state_mux);
    target_s1_2 = new_s1_2;   // both SERVO1 and SERVO2 track this same target
    target_s3   = new_s3;
    target_s4   = new_s4;
    target_s5   = new_s5;
    taskEXIT_CRITICAL(&state_mux);

    // Linear Actuator signed command — only write on state change
    float new_linear = constrain(msg->data.data[4], -1.0f, 1.0f);
    if (fabsf(new_linear - linear_actuator_cmd) > 0.001f) {
        linear_actuator_cmd = new_linear;
        setLinearActuator(linear_actuator_cmd);
    }

    // Pneumatic Gripper toggle — only write on state change
    bool new_pneumatic = (msg->data.data[5] > 0.5f);
    if (new_pneumatic != pneumatic_on) {
        pneumatic_on = new_pneumatic;
        setPneumaticGripper(pneumatic_on);
    }

    // Lead Screw — only call hardware when value changes
    float new_ls = msg->data.data[6];
    taskENTER_CRITICAL(&state_mux);
    float cached_ls = lead_screw_vel;
    taskEXIT_CRITICAL(&state_mux);
    if (fabsf(new_ls - cached_ls) > 0.001f) {
        taskENTER_CRITICAL(&state_mux);
        lead_screw_vel = new_ls;
        taskEXIT_CRITICAL(&state_mux);
        setLeadScrew(new_ls);
    }

    // Linear Rhino — signed, only write on change  (NEW)
    float new_rh = constrain(msg->data.data[7], -RHINO_MAX_VEL, RHINO_MAX_VEL);
    taskENTER_CRITICAL(&state_mux);
    float cached_rh = rhino_vel;
    taskEXIT_CRITICAL(&state_mux);
    if (fabsf(new_rh - cached_rh) > 0.001f) {
        taskENTER_CRITICAL(&state_mux);
        rhino_vel = new_rh;
        taskEXIT_CRITICAL(&state_mux);
        setLinearRhino(new_rh);
    }
}

// ════════════════════════════════════════════════════════════════
// /joy CALLBACK — debug only
// ════════════════════════════════════════════════════════════════
void joy_callback(const void *msgin) {
    (void)msgin;
}

// ════════════════════════════════════════════════════════════════
// micro-ROS INIT
// ════════════════════════════════════════════════════════════════
bool setup_micro_ros() {
    set_microros_transports();
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_mechanism_node", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &mech_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/mechanism_cmds"
    ));

    RCCHECK(rclc_subscription_init_best_effort(
        &joy_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
        "/joy"
    ));

    mech_msg.data.data     = mech_data;
    mech_msg.data.size     = MECH_NUM_CMDS;
    mech_msg.data.capacity = MECH_NUM_CMDS;

    joy_msg.axes.data        = joy_axes_data;
    joy_msg.axes.size        = 6;
    joy_msg.axes.capacity    = 6;
    joy_msg.buttons.data     = joy_buttons_data;
    joy_msg.buttons.size     = 16;
    joy_msg.buttons.capacity = 16;

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &mech_subscriber, &mech_msg, mech_callback, ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &joy_subscriber, &joy_msg, joy_callback, ON_NEW_DATA
    ));

    mech_buf_ready = true;
    Serial.println("[UROS] micro-ROS ready — /mechanism_cmds (8 values, RELIABLE)");
    return true;
}

// ════════════════════════════════════════════════════════════════
// micro-ROS TASK  (Core 1 — 200 Hz)
// ════════════════════════════════════════════════════════════════
void microros_task(void *pvParameters) {
    const TickType_t period    = pdMS_TO_TICKS(5);
    TickType_t       last_wake = xTaskGetTickCount();
    while (true) {
        if (microros_ready && mech_buf_ready)
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        vTaskDelayUntil(&last_wake, period);
    }
}

// ════════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n+----------------------------------------------+");
    Serial.println("|  ESP3 - MECHANISMS CONTROLLER (Att. 3)      |");
    Serial.println("|  Servo1+2 separated | Linear Rhino added    |");
    Serial.println("+----------------------------------------------+\n");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    setupServos();       // ch 0,1,2,3,6 — pins 13,14,25,27,12
    setupMDDRC10();      // ch 4,5       — pins 23,26
    setupLeadScrew();    // MCPWM Unit 0 — pins 18,19
    setupLinearRhino();  // MCPWM Unit 1 — pins 32,33  (NEW)

    if (setup_micro_ros()) {
        microros_ready = true;
        digitalWrite(LED_PIN, HIGH);
    } else {
        Serial.println("WARNING: micro-ROS init failed");
    }

    xTaskCreatePinnedToCore(microros_task, "uros_mech", 4096, NULL, 2,
                            &microros_task_handle, 1);

    last_cmd_ms = millis();
    Serial.println("[READY] Waiting for /mechanism_cmds (8 values)...");
}

// ════════════════════════════════════════════════════════════════
// LOOP  (Core 0) — servo slew + watchdog
// ════════════════════════════════════════════════════════════════
void loop() {
    unsigned long now = millis();

    // Watchdog: stop DC motors if topic goes silent
    taskENTER_CRITICAL(&state_mux);
    float c_ls = lead_screw_vel;
    float c_rh = rhino_vel;
    taskEXIT_CRITICAL(&state_mux);

    if ((now - last_cmd_ms) > CMD_TIMEOUT_MS) {
        if (fabsf(c_ls) > 0.0f) {
            taskENTER_CRITICAL(&state_mux); lead_screw_vel = 0.0f; taskEXIT_CRITICAL(&state_mux);
            setLeadScrew(0.0f);
            Serial.println("WARNING: Timeout — Lead Screw stopped");
        }
        if (fabsf(c_rh) > 0.0f) {
            taskENTER_CRITICAL(&state_mux); rhino_vel = 0.0f; taskEXIT_CRITICAL(&state_mux);
            setLinearRhino(0.0f);
            Serial.println("WARNING: Timeout — Linear Rhino stopped");
        }
    }

    // Servo slew — fires every SERVO_TICK_MS
    static unsigned long last_servo_tick = 0;
    if (now - last_servo_tick >= SERVO_TICK_MS) {
        last_servo_tick = now;

        int t1_2, t3, t4, t5;
        taskENTER_CRITICAL(&state_mux);
        t1_2 = target_s1_2;
        t3   = target_s3;
        t4   = target_s4;
        t5   = target_s5;
        taskEXIT_CRITICAL(&state_mux);

        auto slew = [](int &cur, int tgt) {
            if (cur == tgt) return;
            int d = tgt - cur;
            if (d >  SERVO_SLEW_DEG) d =  SERVO_SLEW_DEG;
            if (d < -SERVO_SLEW_DEG) d = -SERVO_SLEW_DEG;
            cur += d;
        };

        slew(current_s1, t1_2);
        slew(current_s2, t1_2);   // SERVO2 tracks same target as SERVO1
        slew(current_s3, t3);
        slew(current_s4, t4);
        slew(current_s5, t5);

        writeServo(SERVO1_CH, current_s1);
        writeServo(SERVO2_CH, current_s2);   // NEW — individual channel
        writeServo(SERVO3_CH, current_s3);
        writeServo(SERVO4_CH, current_s4);
        writeServo(SERVO5_CH, current_s5);
    }

    // Heartbeat LED
    static unsigned long last_blink = 0;
    if (now - last_blink >= 1000) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        last_blink = now;
    }
}
