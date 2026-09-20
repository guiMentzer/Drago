#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <U8g2lib.h>

// Micro-ROS
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/joint_state.h>
#include <micro_ros_utilities/type_utilities.h>

// Custom bitmap graphics
#include "imagens.h"

// ─────────────────────────────────────────────────────────────────────────────
// Macros & Constants
// ─────────────────────────────────────────────────────────────────────────────
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ return false; }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define SERVO_FREQ   50 // Standard analog servo update frequency (50 Hz)
#define NUM_JOINTS   7  // 6 Arm joints (0..5) + 1 Gripper servo (6)

// ─────────────────────────────────────────────────────────────────────────────
// Joint Calibration & Kinematic Limits
// Units:
//   - Joints 0 to 5 (Arm): Radians
//   - Joint 6 (Gripper): Meters displacement (0.0 = closed, 0.009 = open)
// The calibrated pulse counts absorb all original offsets and inversions.
// ─────────────────────────────────────────────────────────────────────────────
struct JointCalib {
    const char *name;      // Joint name matching ROS 2 URDF / ros2_control
    float min_pos;         // Minimum limit (rad or m)
    float max_pos;         // Maximum limit (rad or m)
    int16_t pulse_at_min;  // PCA9685 pulse count at min_pos
    int16_t pulse_at_max;  // PCA9685 pulse count at max_pos
};

static const JointCalib JOINTS_CALIB[NUM_JOINTS] = {
    // Joint 0: Waist_joint [-1.5708, 1.5708] rad -> [126, 633]
    { "Waist_joint",       -1.5708f,  1.5708f, 126, 633 },

    // Joint 1: Shoulder_joint [-1.3100, 2.5000] rad -> Inverted [573, 66]
    { "Shoulder_joint",    -1.3100f,  2.5000f, 573,  66 },

    // Joint 2: Elbow_joint [-4.0000, 0.1500] rad -> [81, 588]
    { "Elbow_joint",       -4.0000f,  0.1500f,  81, 588 },

    // Joint 3: Wrist_1_joint [-1.5708, 1.5708] rad -> Inverted [598, 152]
    { "Wrist_1_joint",     -1.5708f,  1.5708f, 598, 152 },

    // Joint 4: Wrist_2_joint [-1.5708, 1.3963] rad -> Inverted [578, 71]
    { "Wrist_2_joint",     -1.5708f,  1.3963f, 578,  71 },

    // Joint 5: Effector_joint [-1.5708, 1.5708] rad -> [120, 544]
    { "Effector_joint",    -1.5708f,  1.5708f, 120, 544 },

    // Joint 6: Right_Claw_joint [0.0 m (closed), 0.009 m (open)] -> [501, 372]
    { "Right_Claw_joint",   0.0000f,  0.0090f, 501, 372 }
};

// Current joint command positions (0..5: rad, 6: m)
float current_positions[NUM_JOINTS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.009f};

// ─────────────────────────────────────────────────────────────────────────────
// Hardware Peripherals
// ─────────────────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver Driver = Adafruit_PWMServoDriver();
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ─────────────────────────────────────────────────────────────────────────────
// Clean Linear Interpolation (Angle / Position to PWM Pulse)
// ─────────────────────────────────────────────────────────────────────────────
int16_t angleToPulse(float position, uint8_t joint) 
{
    if (joint >= NUM_JOINTS) return 0;

    const JointCalib &j = JOINTS_CALIB[joint];

    // Normalized progress (0.0 to 1.0), clamped safely between limits
    float t = (position - j.min_pos) / (j.max_pos - j.min_pos);
    t = constrain(t, 0.0f, 1.0f);

    // Continuous floating-point interpolation rounded to nearest PWM count
    return (int16_t)roundf(j.pulse_at_min + t * (j.pulse_at_max - j.pulse_at_min));
}

// ─────────────────────────────────────────────────────────────────────────────
// Micro-ROS Structures & Agent State
// ─────────────────────────────────────────────────────────────────────────────
enum AgentState {
    WAITING_AGENT,
    CONNECTED_AGENT,
    DISCONNECTED_AGENT
};

AgentState agentState = WAITING_AGENT;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t subscriber;
sensor_msgs__msg__JointState joint_state_msg;
rclc_executor_t executor;

// ─────────────────────────────────────────────────────────────────────────────
// Dynamic Memory Pre-allocation for sensor_msgs/msg/JointState
// ─────────────────────────────────────────────────────────────────────────────
bool initJointStateMemory() 
{
    micro_ros_utilities_memory_conf_t conf = micro_ros_utilities_memory_conf_default;
    conf.max_string_capacity              = 32; // Longest joint name string
    conf.max_basic_type_sequence_capacity = 10; // Supports up to 10 joints

    return micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        &joint_state_msg,
        conf
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Joint State Subscriber Callback
// ─────────────────────────────────────────────────────────────────────────────
void joint_state_callback(const void *msgin) 
{
    const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;
    if (!msg) return;

    // 1. Match by joint name (robust against any publisher joint re-ordering)
    for (size_t i = 0; i < msg->name.size; ++i) {
        if (i >= msg->position.size) break;

        const char *name = msg->name.data[i].data;
        double pos = msg->position.data[i];

        for (uint8_t j = 0; j < NUM_JOINTS; ++j) {
            if (strcmp(name, JOINTS_CALIB[j].name) == 0) {
                current_positions[j] = (float)pos;
                break;
            }
        }
    }

    // 2. Fallback: anonymous sequential order if names are omitted
    if (msg->name.size == 0 && msg->position.size >= 6) {
        size_t count = (msg->position.size < NUM_JOINTS) ? msg->position.size : NUM_JOINTS;
        for (size_t j = 0; j < count; ++j) {
            current_positions[j] = (float)msg->position.data[j];
        }
    }

    // 3. Command the PWM driver immediately for all 7 channels
    for (uint8_t j = 0; j < NUM_JOINTS; ++j) {
        Driver.setPWM(j, 0, angleToPulse(current_positions[j], j));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Micro-ROS Entity Lifecycle
// ─────────────────────────────────────────────────────────────────────────────
bool createEntities() 
{
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "drago_firmware_node", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states"
    ));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &subscriber,
        &joint_state_msg,
        &joint_state_callback,
        ON_NEW_DATA
    ));

    return true;
}

void destroyEntities() 
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}

// ─────────────────────────────────────────────────────────────────────────────
// OLED Display UI
// ─────────────────────────────────────────────────────────────────────────────
void drawJointAnglesWindow() 
{
    display.drawLine(50, 9, 50, 64);

    display.setFont(u8g2_font_04b_03_tr);
    display.setCursor(48, 8); display.print("0");
    display.setCursor(85, 8); display.print("MAX");
    display.setCursor(2, 8);  display.print("MIN");

    for (uint8_t i = 0; i < 6; i++) {
        // Compute normalized bar progress (0 to 100)
        float t = (current_positions[i] - JOINTS_CALIB[i].min_pos) / 
                  (JOINTS_CALIB[i].max_pos - JOINTS_CALIB[i].min_pos);
        int bar_width = (int)roundf(constrain(t, 0.0f, 1.0f) * 100.0f);

        display.drawFrame(1, 10 + i * 9, 100, 7);
        display.drawBox(1, 10 + i * 9, bar_width, 7);

        // Print joint angle in degrees
        display.setCursor(105, 16 + i * 9);
        display.print(current_positions[i] * (180.0f / PI), 1);
    }
}

void drawWaitingWindow() 
{
    display.setFont(u8g2_font_6x10_tr);
    display.drawStr(16, 25, "DRAGO ROBOT");
    display.setFont(u8g2_font_04b_03_tr);
    display.drawStr(12, 45, "WAITING MICRO-ROS AGENT...");
    
    // Animated progress indicator
    int dot_count = (millis() / 300) % 4;
    for (int i = 0; i < dot_count; i++) {
        display.drawDisc(55 + i * 8, 55, 2);
    }
}

void updateDisplay() 
{
    display.clearBuffer();
    if (agentState == CONNECTED_AGENT) {
        drawJointAnglesWindow();
    } else {
        drawWaitingWindow();
    }
    display.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() 
{
    // Micro-ROS Serial Transport at 115200 baud
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // I2C Peripherals: PCA9685 Driver & OLED Display
    Wire.begin();
    Driver.begin();
    Driver.setPWMFreq(SERVO_FREQ);

    display.begin();
    display.setI2CAddress(0x3C * 2);

    // Initial Splash Screen
    display.clearBuffer();
    display.drawXBMP(0, 0, 112, 64, epd_bitmap_drago);
    display.setFont(u8g2_font_04b_03_tr);
    display.drawStr(50, 80, "...DRAGO!");
    display.sendBuffer();

    // Allocate dynamic memory for sensor_msgs/msg/JointState
    initJointStateMemory();

    // Move servos to initial home positions
    for (uint8_t j = 0; j < NUM_JOINTS; ++j) {
        Driver.setPWM(j, 0, angleToPulse(current_positions[j], j));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() 
{
    static unsigned long last_ping_time = 0;
    static unsigned long last_display_time = 0;
    unsigned long now = millis();

    // 1. Refresh OLED display at ~15 Hz (non-blocking)
    if (now - last_display_time >= 66) {
        last_display_time = now;
        updateDisplay();
    }

    // 2. Micro-ROS Agent State Machine
    switch (agentState) {
        case WAITING_AGENT:
            // Check for ROS 2 agent presence every 500 ms
            if (now - last_ping_time >= 500) {
                last_ping_time = now;
                if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
                    if (createEntities()) {
                        agentState = CONNECTED_AGENT;
                    } else {
                        destroyEntities();
                    }
                }
            }
            break;

        case CONNECTED_AGENT:
            // Spin executor to process incoming JointState messages
            RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

            // Monitor agent connection health every 500 ms
            if (now - last_ping_time >= 500) {
                last_ping_time = now;
                if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
                    agentState = DISCONNECTED_AGENT;
                }
            }
            break;

        case DISCONNECTED_AGENT:
            destroyEntities();
            agentState = WAITING_AGENT;
            break;
    }
}
