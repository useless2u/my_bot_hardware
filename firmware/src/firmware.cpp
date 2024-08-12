// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <i2cdetect.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "config.h"
#include "syslog.h"
#include "led.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#include "mag.h"
//#define ENCODER_USE_INTERRUPTS
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "battery.h"
#include "range.h"
#include "lidar.h"
#include "wifis.h"
#include "ota.h"
#include "pwm.h"
#include "M5Module4EncoderMotor.h"

#ifdef WDT_TIMEOUT
#include <esp_task_wdt.h>
#endif
#ifdef MICRO_ROS_TRANSPORT_ARDUINO_WIFI
// remove wifi initialization code from wifi transport
static inline void set_microros_net_transports(IPAddress agent_ip, uint16_t agent_port)
{
    static struct micro_ros_agent_locator locator;
    locator.address = agent_ip;
    locator.port = agent_port;

    rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    );
}
#endif

#ifndef BAUDRATE
#define BAUDRATE 921600
#endif

#ifndef NODE_NAME
#define NODE_NAME "linorobot_base_node"
#endif
#ifndef TOPIC_PREFIX
#define TOPIC_PREFIX
#endif
#ifndef CONTROL_TIMER
#define CONTROL_TIMER 20 // 50Hz
#endif
#ifndef BATTERY_TIMER
#define BATTERY_TIMER 2000 // 2 sec
#endif
#ifndef RANGE_TIMER
#define RANGE_TIMER 100 // 10Hz
#endif

#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#endif
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#endif
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;
rcl_subscription_t twist_subscriber;
rcl_publisher_t battery_publisher;
rcl_publisher_t range_publisher;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__BatteryState battery_msg;
sensor_msgs__msg__Range range_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
float prev_voltage;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#ifdef USE_M5_4ENCODER_MOTOR_DRIVER
    Motor motor1_controller(MOTOR1_ADDR,MOTOR1_INV,MOTOR1_NUMBER);
    Motor motor2_controller(MOTOR2_ADDR,MOTOR2_INV,MOTOR2_NUMBER);
    Motor motor3_controller(MOTOR3_ADDR,MOTOR3_INV,MOTOR3_NUMBER);
    Motor motor4_controller(MOTOR4_ADDR,MOTOR4_INV,MOTOR4_NUMBER);
#else
    Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
    Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
    Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
    Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);
#endif

#ifdef USE_M5_4ENCODER_MOTOR_DRIVER
        Encoder motor1_encoder(MOTOR1_ADDR,MOTOR1_ENCODER_INV,COUNTS_PER_REV1,MOTOR1_NUMBER);
    Encoder motor2_encoder(MOTOR2_ADDR,MOTOR2_ENCODER_INV,COUNTS_PER_REV2,MOTOR2_NUMBER);
    Encoder motor3_encoder(MOTOR3_ADDR,MOTOR3_ENCODER_INV,COUNTS_PER_REV3,MOTOR3_NUMBER);
    Encoder motor4_encoder(MOTOR4_ADDR,MOTOR4_ENCODER_INV,COUNTS_PER_REV4,MOTOR4_NUMBER);       
#else
        Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
        Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
        Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
        Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);
#endif
	



PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE
);

Odometry odometry;
IMU imu;
MAG mag;

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        setLed(HIGH);
        delay(150);
        setLed(LOW);
        delay(150);
    }
    delay(1000);
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();
}

void rclErrorLoop()
{
    while(true)
    {
        flashLED(2); // flash 2 times
        runOta();
    }
}

void moveBase()
{
    // brake if there's no command received, or when it's only the first command sent
    if(((millis() - prev_cmd_time) >= 200))
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;
        setLed(HIGH);
    }
    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.angular.z
    );

    // get the current speed of each motor
    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();
    float current_rpm3 = motor3_encoder.getRPM();
    float current_rpm4 = motor4_encoder.getRPM();
    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
    motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));

    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rpm1,
        current_rpm2,
        current_rpm3,
        current_rpm4
    );

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt,
        current_vel.linear_x,
        current_vel.linear_y,
        current_vel.angular_z
    );
}

bool syncTime()
{
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true; // synchronized previously
    // get the current time from the agent
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    if (rmw_uros_epoch_synchronized()) {
#if (_POSIX_TIMERS > 0)
        // Get time in milliseconds or nanoseconds
        int64_t time_ns = rmw_uros_epoch_nanos();
    timespec tp;
    tp.tv_sec = time_ns / 1000000000;
    tp.tv_nsec = time_ns % 1000000000;
    clock_settime(CLOCK_REALTIME, &tp);
#else
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - millis();
#endif
    return true;
    }
    return false;
}

struct timespec getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}

void twistCallback(const void * msgin)
{
    setLed(!getLed());

    prev_cmd_time = millis();
}

void publishData()
{
    static unsigned skip_dip = 0;
    odom_msg = odometry.getData();
    imu_msg = imu.getData();
#ifdef USE_FAKE_IMU
    imu_msg.angular_velocity.z = odom_msg.twist.twist.angular.z;
#endif
    mag_msg = mag.getData();
#ifdef MAG_BIAS
    const float mag_bias[3] = MAG_BIAS;
    mag_msg.magnetic_field.x -= mag_bias[0];
    mag_msg.magnetic_field.y -= mag_bias[1];
    mag_msg.magnetic_field.z -= mag_bias[2];
#endif

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

#ifndef USE_FAKE_MAG
    mag_msg.header.stamp.sec = time_stamp.tv_sec;
    mag_msg.header.stamp.nanosec = time_stamp.tv_nsec;
#endif

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
#ifndef USE_FAKE_MAG
    RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
#endif
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
#if defined(BATTERY_PIN) || defined(USE_INA219) || defined(USE_INA226)
    battery_msg = getBattery();
    battery_msg.header.stamp.sec = time_stamp.tv_sec;
    battery_msg.header.stamp.nanosec = time_stamp.tv_nsec;
#ifdef BATTERY_DIP
    if (!skip_dip && battery_msg.voltage > 1.0  && battery_msg.voltage < prev_voltage * BATTERY_DIP) {
        RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
    syslog(LOG_WARNING, "%s voltage dip %.2f", __FUNCTION__, battery_msg.voltage);
        skip_dip = 5;
    }
    if (skip_dip) skip_dip--;
#endif
    battery_msg.voltage = prev_voltage = battery_msg.voltage * 0.01 + prev_voltage * 0.99;
    EXECUTE_EVERY_N_MS(BATTERY_TIMER, {
        getBatteryPercentage(&battery_msg);
        RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL)) });
#endif
#if defined(ECHO_PIN) || defined (VL53L0X)
    EXECUTE_EVERY_N_MS(RANGE_TIMER, {
        range_msg = getRange();
        range_msg.header.stamp.sec = time_stamp.tv_sec;
        range_msg.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&range_publisher, &range_msg, NULL)) });
#endif
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
       moveBase();
       publishData();
    }
}

bool createEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
    // create odometry publisher
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        TOPIC_PREFIX "odom/unfiltered"
    ));
    // create IMU publisher
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    // if we have magnetomter, use imu/data_raw for madgwick filter
#ifndef USE_FAKE_MAG
        TOPIC_PREFIX "imu/data_raw"
#else
        TOPIC_PREFIX "imu/data"
#endif
    ));
#ifndef USE_FAKE_MAG
    RCCHECK(rclc_publisher_init_default(
        &mag_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        TOPIC_PREFIX "imu/mag"
    ));
#endif
#if defined(BATTERY_PIN) || defined(USE_INA219) || defined(USE_INA226)
    // create battery pyblisher
    RCCHECK(rclc_publisher_init_default(
    &battery_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    TOPIC_PREFIX "battery"
    ));
#endif
#ifdef ECHO_PIN
    // create range pyblisher
    RCCHECK(rclc_publisher_init_default(
    &range_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    TOPIC_PREFIX "sonar"
    ));
#endif
    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        TOPIC_PREFIX "cmd_vel"
    ));
    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = CONTROL_TIMER;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // synchronize time with the agent
    syncTime();
    setLed(HIGH);

    return true;
}

bool destroyEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
#ifndef USE_FAKE_MAG
    RCSOFTCHECK(rcl_publisher_fini(&mag_publisher, &node));
#endif
#if defined(BATTERY_PIN) || defined(USE_INA219)
    RCSOFTCHECK(rcl_publisher_fini(&battery_publisher, &node));
#endif
#ifdef ECHO_PIN
    RCSOFTCHECK(rcl_publisher_fini(&range_publisher, &node));
#endif
    RCSOFTCHECK(rcl_subscription_fini(&twist_subscriber, &node));
    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node))
    RCSOFTCHECK(rclc_support_fini(&support));

    setLed(HIGH);

    return true;
}

void setup()
{
    Serial.begin(BAUDRATE);
    initLed();
#ifdef BOARD_INIT // board specific setup, must include Wire.begin
    BOARD_INIT
#else
    Wire.begin();
#endif

#ifdef WDT_TIMEOUT
    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
#endif
    initWifis();
    syslog(LOG_INFO, "%s test syslog ", __FUNCTION__ );
    Serial.println("serial started");
    initOta();
    i2cdetect();  // default range from 0x03 to 0x77
    initPwm();
    motor1_controller.begin();
    //motor1_controller.start(20);
    motor2_controller.begin();
    //motor2_controller.start(20);
    motor3_controller.begin();
    motor4_controller.begin();
    motor1_encoder.begin();
    motor2_encoder.begin();
    motor3_encoder.begin();
    motor4_encoder.begin();
    
    bool imu_ok = imu.init();
    if (!imu_ok) // take IMU failure as fatal
    {
        Serial.println("IMU init failed");
        syslog(LOG_INFO, "%s IMU init failed %lu", __FUNCTION__, millis());
        while (1)
        {
            flashLED(3); // flash 3 times
            runWifis();
            runOta();
        }
    }
    bool mag_ok = mag.init();
    if (!mag_ok) // take IMU failure as fatal
    {
        Serial.println("MAG init failed");
        syslog(LOG_INFO, "%s MAG init failed %lu", __FUNCTION__, millis());
        while (1)
        {
            flashLED(4); // flash 4 times
            runWifis();
            runOta();
        }
    }
    initBattery();
    initRange();
    initLidar(); // after wifi connected
    while (getLidarDeviceInfo==0) { Serial.println("waiting for Lidar device info\n");delay(200); }
    lidarStartScan();
    battery_msg = getBattery();
    prev_voltage = battery_msg.voltage;

#ifdef MICRO_ROS_TRANSPORT_ARDUINO_WIFI
    set_microros_net_transports(AGENT_IP, AGENT_PORT);
    Serial.println("transport is wifi");

#else
    Serial.println("transport is serial");

    set_microros_serial_transports(Serial);
#endif

#ifdef BOARD_INIT_LATE // board specific setup
    BOARD_INIT_LATE
#endif
    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());
}

void loop() {
    switch (state)
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            syslog(LOG_INFO, "%s agent available %lu", __FUNCTION__, millis());

            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT)
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:

            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED)
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            syslog(LOG_INFO, "%s agent disconnected %lu", __FUNCTION__, millis());
            fullStop();
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
    runWifis();
    runOta();
#ifdef WDT_TIMEOUT
    esp_task_wdt_reset();
#endif
#ifdef BOARD_LOOP // board specific loop
    BOARD_LOOP
#endif
}
