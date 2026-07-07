/**
 * ╔══════════════════════════════════════════════════════╗
 * ║         MAIN ESP32 — MECANUM ROBOT FIRMWARE          ║
 * ║                      FIXED VERSION                   ║
 * ╚══════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <Bluepad32.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joy.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include "driver/mcpwm.h"
#include "driver/pcnt.h"

// ════════════════════════════════════════════════════
// PIN DEFINITIONS
// ════════════════════════════════════════════════════

#define MOTOR_FL_PWM  18
#define MOTOR_FL_DIR  19
#define MOTOR_FR_PWM  21
#define MOTOR_FR_DIR  22
#define MOTOR_BL_PWM  32
#define MOTOR_BL_DIR  33
#define MOTOR_BR_PWM  26
#define MOTOR_BR_DIR  27

#define ENC_FL_A  4
#define ENC_FL_B  13
#define ENC_FR_A  14
#define ENC_FR_B  15
#define ENC_BL_A  2
#define ENC_BL_B  5
#define ENC_BR_A  23
#define ENC_BR_B  25

#define LED_PIN  2

// ════════════════════════════════════════════════════
// CONFIGURATION
// ════════════════════════════════════════════════════

#define PWM_FREQUENCY  20000
#define MOTOR_DEADZONE 0.05f
#define ENCODER_PPR    360
#define MAX_VELOCITY   3.3f  // m/s
#define ENCODER_PUB_HZ 50
#define JOY_PUB_HZ     50

#define JOY_NUM_AXES    6
#define JOY_NUM_BUTTONS 16
#define AXIS_SCALE   512.0f
#define TRIG_SCALE   1023.0f

// ════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ════════════════════════════════════════════════════

rcl_allocator_t allocator;
rclc_support_t  support;
rcl_node_t      node;
rclc_executor_t executor;
rcl_publisher_t    joy_publisher;
rcl_publisher_t    encoder_publisher;
rcl_subscription_t motor_cmd_subscriber;

sensor_msgs__msg__Joy            joy_msg;
std_msgs__msg__Int32MultiArray   encoder_msg;
std_msgs__msg__Float32MultiArray motor_cmd_msg;
rcl_timer_t encoder_timer;

volatile int32_t encoder_counts[4] = {0, 0, 0, 0};
float target_velocities[4] = {0, 0, 0, 0};
bool  motors_enabled = false;
unsigned long last_motor_cmd_time = 0;

// Shadow copy for delta-check
static float   prev_axes[JOY_NUM_AXES]      = {0};
static int32_t prev_buttons[JOY_NUM_BUTTONS] = {0};

ControllerPtr myController = nullptr;
volatile bool microros_ready = false;
volatile bool joy_buffers_ready = false;

TaskHandle_t microros_task_handle = NULL;

// ════════════════════════════════════════════════════
// FIXED MOTOR CONTROL
// ════════════════════════════════════════════════════

void setup_motors() {
    // Configure MCPWM Unit 0 (Front motors)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_FL_PWM);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_FR_PWM);
    
    // Configure MCPWM Unit 1 (Back motors)
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, MOTOR_BL_PWM);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, MOTOR_BR_PWM);

    // Direction pins
    pinMode(MOTOR_FL_DIR, OUTPUT);
    pinMode(MOTOR_FR_DIR, OUTPUT);
    pinMode(MOTOR_BL_DIR, OUTPUT);
    pinMode(MOTOR_BR_DIR, OUTPUT);

    // Configure PWM parameters for Unit 0
    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQUENCY;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    
    // Initialize Unit 0 with Timer 0
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    
    // Initialize Unit 1 with Timer 0 (FIXED: was missing proper init)
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
    
    // IMPORTANT: Enable MCPWM signals
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_start(MCPWM_UNIT_1, MCPWM_TIMER_0);
    
    Serial.println("✅ Motors initialized (MCPWM fixed)");
}

void set_motor_velocity(int motor_id, float velocity_mps) {
    // Calculate duty cycle (0-100%)
    float duty = (fabs(velocity_mps) / MAX_VELOCITY) * 100.0f;
    
    // Apply deadzone
    if (fabs(velocity_mps) < MOTOR_DEADZONE * MAX_VELOCITY) {
        duty = 0.0f;
    }
    
    // Limit to 100%
    if (duty > 100.0f) duty = 100.0f;
    
    // Direction: positive = forward, negative = backward
    bool forward = (velocity_mps >= 0.0f);
    
    switch(motor_id) {
        case 0: // Front Left
            digitalWrite(MOTOR_FL_DIR, forward ? HIGH : LOW);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
            break;
            
        case 1: // Front Right
            digitalWrite(MOTOR_FR_DIR, forward ? HIGH : LOW);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty);
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
            break;
            
        case 2: // Back Left
            digitalWrite(MOTOR_BL_DIR, forward ? HIGH : LOW);
            mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
            mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
            break;
            
        case 3: // Back Right
            digitalWrite(MOTOR_BR_DIR, forward ? HIGH : LOW);
            mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, duty);
            mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
            break;
    }
}

void stop_all_motors() {
    for (int i = 0; i < 4; i++) {
        set_motor_velocity(i, 0.0f);
    }
    Serial.println("🛑 Motors stopped");
}

// ════════════════════════════════════════════════════
// FIXED ENCODER SETUP (Quadrature decoding)
// ════════════════════════════════════════════════════

void setup_encoder(pcnt_unit_t unit, int pin_a, int pin_b) {
    pcnt_config_t pcnt_config;
    
    // Configure PCNT channel 0
    pcnt_config.pulse_gpio_num = pin_a;
    pcnt_config.ctrl_gpio_num = pin_b;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = unit;
    pcnt_config.pos_mode = PCNT_COUNT_INC;   // Count on positive edge
    pcnt_config.neg_mode = PCNT_COUNT_DEC;   // Decrement on negative edge
    pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse counting when low
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep when high
    pcnt_config.counter_h_lim = 32767;
    pcnt_config.counter_l_lim = -32768;
    pcnt_unit_config(&pcnt_config);
    
    // Configure PCNT channel 1 for quadrature decoding
    pcnt_config.pulse_gpio_num = pin_b;
    pcnt_config.ctrl_gpio_num = pin_a;
    pcnt_config.channel = PCNT_CHANNEL_1;
    pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_unit_config(&pcnt_config);
    
    // Enable filters to reduce noise
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);
    
    // Initialize counter
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

void setup_encoders() {
    setup_encoder(PCNT_UNIT_0, ENC_FL_A, ENC_FL_B);
    setup_encoder(PCNT_UNIT_1, ENC_FR_A, ENC_FR_B);
    setup_encoder(PCNT_UNIT_2, ENC_BL_A, ENC_BL_B);
    setup_encoder(PCNT_UNIT_3, ENC_BR_A, ENC_BR_B);
    Serial.println("✅ Encoders initialized");
}

void read_encoders() {
    int16_t count;
    
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    encoder_counts[0] = count;
    
    pcnt_get_counter_value(PCNT_UNIT_1, &count);
    encoder_counts[1] = count;
    
    pcnt_get_counter_value(PCNT_UNIT_2, &count);
    encoder_counts[2] = count;
    
    pcnt_get_counter_value(PCNT_UNIT_3, &count);
    encoder_counts[3] = count;
}

// ════════════════════════════════════════════════════
// JOYSTICK FUNCTIONS
// ════════════════════════════════════════════════════

void zero_joy_msg() {
    if (!joy_buffers_ready) return;
    for (int i = 0; i < JOY_NUM_AXES; i++) 
        joy_msg.axes.data[i] = 0.0f;
    for (int i = 0; i < JOY_NUM_BUTTONS; i++) 
        joy_msg.buttons.data[i] = 0;
}

void populate_joy_msg() {
    joy_msg.axes.data[0] = (float)myController->axisX() / AXIS_SCALE;
    joy_msg.axes.data[1] = -(float)myController->axisY() / AXIS_SCALE;
    joy_msg.axes.data[2] = (float)myController->axisRX() / AXIS_SCALE;
    joy_msg.axes.data[3] = -(float)myController->axisRY() / AXIS_SCALE;
    joy_msg.axes.data[4] = (float)myController->brake() / TRIG_SCALE;
    joy_msg.axes.data[5] = (float)myController->throttle() / TRIG_SCALE;
    
    // Clamp axes
    for (int i = 0; i < JOY_NUM_AXES; i++) {
        if (joy_msg.axes.data[i] > 1.0f) joy_msg.axes.data[i] = 1.0f;
        if (joy_msg.axes.data[i] < -1.0f) joy_msg.axes.data[i] = -1.0f;
    }
    
    uint32_t b = myController->buttons();
    uint8_t d = myController->dpad();
    
    joy_msg.buttons.data[0] = (b & 0x0001) ? 1 : 0;
    joy_msg.buttons.data[1] = (b & 0x0002) ? 1 : 0;
    joy_msg.buttons.data[2] = (b & 0x0004) ? 1 : 0;
    joy_msg.buttons.data[3] = (b & 0x0008) ? 1 : 0;
    joy_msg.buttons.data[4] = (b & 0x0010) ? 1 : 0;
    joy_msg.buttons.data[5] = (b & 0x0020) ? 1 : 0;
    joy_msg.buttons.data[6] = (b & 0x0040) ? 1 : 0;
    joy_msg.buttons.data[7] = (b & 0x0080) ? 1 : 0;
    joy_msg.buttons.data[8] = (b & 0x0100) ? 1 : 0;
    joy_msg.buttons.data[9] = (b & 0x0200) ? 1 : 0;
    joy_msg.buttons.data[10] = (d & 0x01) ? 1 : 0;
    joy_msg.buttons.data[11] = (d & 0x02) ? 1 : 0;
    joy_msg.buttons.data[12] = (d & 0x08) ? 1 : 0;
    joy_msg.buttons.data[13] = (d & 0x04) ? 1 : 0;
    joy_msg.buttons.data[14] = (b & 0x0400) ? 1 : 0;
    joy_msg.buttons.data[15] = (b & 0x0800) ? 1 : 0;
}

bool joy_msg_changed() {
    for (int i = 0; i < JOY_NUM_AXES; i++) {
        if (fabsf(joy_msg.axes.data[i] - prev_axes[i]) > 0.001f) return true;
    }
    for (int i = 0; i < JOY_NUM_BUTTONS; i++) {
        if (joy_msg.buttons.data[i] != prev_buttons[i]) return true;
    }
    return false;
}

void snapshot_prev_joy() {
    for (int i = 0; i < JOY_NUM_AXES;    i++) prev_axes[i]    = joy_msg.axes.data[i];
    for (int i = 0; i < JOY_NUM_BUTTONS; i++) prev_buttons[i] = joy_msg.buttons.data[i];
}

// ════════════════════════════════════════════════════
// BLUEPAD32
// ════════════════════════════════════════════════════

void onConnectedController(ControllerPtr ctl) {
    myController = ctl;
    ctl->setColorLED(0, 255, 0);
    Serial.println("✅ Controller connected");
}

void onDisconnectedController(ControllerPtr ctl) {
    if (myController == ctl) {
        myController = nullptr;
        motors_enabled = false;
        zero_joy_msg();
        stop_all_motors();
        Serial.println("⚠️ Controller disconnected");
    }
}

void setup_bluepad32() {
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.enableVirtualDevice(false);
    BP32.enableNewBluetoothConnections(true);
    Serial.println("✅ Bluepad32 initialized");
}

void process_controller() {
    BP32.update();
    if (!myController || !myController->isConnected()) {
        zero_joy_msg();
        return;
    }
    populate_joy_msg();
}

// ════════════════════════════════════════════════════
// MICRO-ROS CALLBACKS
// ════════════════════════════════════════════════════

void motor_cmd_callback(const void* msgin) {
    const auto* msg = (const std_msgs__msg__Float32MultiArray*)msgin;
    if (msg->data.size >= 4) {
        for (int i = 0; i < 4; i++) {
            target_velocities[i] = msg->data.data[i];
        }
        motors_enabled = true;
        last_motor_cmd_time = millis();
        
        // DEBUG: Print received commands
        static unsigned long last_print = 0;
        if (millis() - last_print > 1000) {
            Serial.printf("Motor cmds: FL=%.2f FR=%.2f BL=%.2f BR=%.2f\n",
                         target_velocities[0], target_velocities[1],
                         target_velocities[2], target_velocities[3]);
            last_print = millis();
        }
    }
}

void encoder_timer_callback(rcl_timer_t*, int64_t) {
    read_encoders();
    for (int i = 0; i < 4; i++) {
        encoder_msg.data.data[i] = encoder_counts[i];
    }
    rcl_publish(&encoder_publisher, &encoder_msg, NULL);
}

// ════════════════════════════════════════════════════
// MICRO-ROS SETUP
// ════════════════════════════════════════════════════

bool setup_micro_ros() {
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "esp32_mecanum_node", "", &support) != RCL_RET_OK) return false;
    if (rclc_publisher_init_default(&joy_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy), "/joy") != RCL_RET_OK) return false;
    if (rclc_publisher_init_default(&encoder_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/encoder_ticks") != RCL_RET_OK) return false;
    if (rclc_subscription_init_default(&motor_cmd_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/motor_cmds") != RCL_RET_OK) return false;
    if (rclc_timer_init_default(&encoder_timer, &support,
            RCL_MS_TO_NS(1000 / ENCODER_PUB_HZ), encoder_timer_callback) != RCL_RET_OK) return false;
    
    if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false;
    if (rclc_executor_add_timer(&executor, &encoder_timer) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(&executor, &motor_cmd_subscriber, &motor_cmd_msg,
            motor_cmd_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
    
    // Initialize message buffers
    static float joy_axes_data[JOY_NUM_AXES] = {0};
    static int32_t joy_buttons_data[JOY_NUM_BUTTONS] = {0};
    joy_msg.axes.data = joy_axes_data;
    joy_msg.axes.size = JOY_NUM_AXES;
    joy_msg.axes.capacity = JOY_NUM_AXES;
    joy_msg.buttons.data = joy_buttons_data;
    joy_msg.buttons.size = JOY_NUM_BUTTONS;
    joy_msg.buttons.capacity = JOY_NUM_BUTTONS;
    joy_buffers_ready = true;
    
    static int32_t enc_data[4] = {0};
    encoder_msg.data.data = enc_data;
    encoder_msg.data.size = 4;
    encoder_msg.data.capacity = 4;
    
    static float motor_data[4] = {0};
    motor_cmd_msg.data.data = motor_data;
    motor_cmd_msg.data.size = 4;
    motor_cmd_msg.data.capacity = 4;
    
    Serial.println("✅ micro-ROS initialized");
    return true;
}

// ════════════════════════════════════════════════════
// micro-ROS TASK (Core 1)
// ════════════════════════════════════════════════════

void microros_task(void* pvParameters) {
    const TickType_t period    = pdMS_TO_TICKS(1000 / JOY_PUB_HZ);
    TickType_t       last_wake = xTaskGetTickCount();
    unsigned long last_joy_pub_ms = 0;

    while (true) {
        if (microros_ready && joy_buffers_ready) {
            // Spin executor to process timer (encoders) and subscriber (motor_cmds)
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

            unsigned long now_ms = millis();
            // Publish /joy only when data changed OR as a heartbeat every 150 ms
            if (joy_msg_changed() || (now_ms - last_joy_pub_ms >= 150)) {
                rcl_publish(&joy_publisher, &joy_msg, NULL);
                snapshot_prev_joy();
                last_joy_pub_ms = now_ms;
            }
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

// ════════════════════════════════════════════════════
// SETUP & LOOP
// ════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200); // Reverted to 115200 to match default micro-ros agent baud rate
    delay(1000);
    
    Serial.println("\n╔════════════════════════════════════╗");
    Serial.println("║   MECANUM ROBOT - FIXED VERSION    ║");
    Serial.println("╚════════════════════════════════════╝\n");
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    setup_motors();
    setup_encoders();
    setup_bluepad32();
    
    if (setup_micro_ros()) {
        microros_ready = true;
        digitalWrite(LED_PIN, HIGH);
    } else {
        Serial.println("⚠️ micro-ROS init failed (running in offline mode)");
    }
    
    xTaskCreatePinnedToCore(
        microros_task,
        "uros_pub",
        4096,
        NULL,
        2,
        &microros_task_handle,
        1
    );
    
    Serial.println("🚀 System ready!");
    
    // Test motors at startup (optional)
    delay(1000);
    Serial.println("Testing motors...");
    for(int i = 0; i < 4; i++) {
        set_motor_velocity(i, 0.5);
        delay(500);
        set_motor_velocity(i, 0);
        delay(200);
    }
    Serial.println("Motor test complete");
}

void loop() {
    unsigned long now = millis();
    
    process_controller();
    
    // Apply motor commands with 200ms watchdog failsafe
    if (motors_enabled) {
        if (now - last_motor_cmd_time > 200) {
            stop_all_motors();
            motors_enabled = false;
        } else {
            for (int i = 0; i < 4; i++) {
                set_motor_velocity(i, target_velocities[i]);
            }
        }
    }
    
    static unsigned long last_blink = 0;
    if (now - last_blink > 1000) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        last_blink = now;
    }
}
