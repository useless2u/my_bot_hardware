// Copyright (c) 2021 Juan Miguel Jimeno
// Copyright (c) 2023 Thomas Chou
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
#include <stdio.h>
#include <i2cdetect.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "config.h"
#include "syslog.h"
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

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__BatteryState battery_msg;
sensor_msgs__msg__Range range_msg;

void setLed(int value)
{
#ifdef LED_PIN
    digitalWrite(LED_PIN, value);
#endif
}

int getLed(void)
{
#ifdef LED_PIN
    return digitalRead(LED_PIN);
#else
    return 0;
#endif
}

void initLed(void)
{
#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
#endif
}


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
unsigned total_motors = 2;

void setup()
{
    Serial.begin(BAUDRATE);
    initLed();
#ifdef BOARD_INIT // board specific setup
    BOARD_INIT;
#endif

    initWifis();
    initOta();
    i2cdetect();  // default range from 0x03 to 0x77
    initPwm();
    motor1_controller.begin();
    motor2_controller.begin();
    motor3_controller.begin();
    motor4_controller.begin();
    motor1_encoder.begin();
    //motor1_controller.setEncVal(1000);
    motor2_encoder.begin();
    motor3_encoder.begin();
    motor4_encoder.begin();
    //motor1_controller.start(200);
    //delay(1000);
    //motor2_controller.start(100);
    imu.init();
    mag.init();
    initBattery();
    initRange();

    if(Kinematics::LINO_BASE == Kinematics::DIFFERENTIAL_DRIVE)
    {
        total_motors = 4;
    }
    syslog(LOG_INFO, "%s pre rpm %lu", __FUNCTION__, millis());

    motor1_encoder.getRPM();
    motor2_encoder.getRPM();
    motor3_encoder.getRPM();
    motor4_encoder.getRPM();
    syslog(LOG_INFO, "%s post rpm %lu", __FUNCTION__, millis());

#ifdef BOARD_INIT_LATE // board specific setup
    BOARD_INIT_LATE;
#endif
    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());
}

void loop() {
    static unsigned tk = 0; // tick
    const unsigned run_time = 8; // run time of each motor
    const unsigned cycle = run_time * total_motors;
    unsigned current_motor = tk / run_time % total_motors;
    unsigned direction = tk / cycle % 2; // 0 forward, 1 reverse
    int pwm_max = (1 << PWM_BITS) - 1;
    pwm_max = PWM_MAX;
    static float max_rpm, stopping;
    //pwm_max = 200;
    syslog(LOG_INFO, "%s pwm_max %d", __FUNCTION__, pwm_max);
    setLed(direction ? LOW : HIGH);

    motor1_controller.spin((current_motor == 0) ? (direction ? -pwm_max : pwm_max) : 0);
    motor2_controller.spin((current_motor == 1) ? (direction ? -pwm_max : pwm_max) : 0);
    motor3_controller.spin((current_motor == 2) ? (direction ? -pwm_max : pwm_max) : 0);
    motor4_controller.spin((current_motor == 3) ? (direction ? -pwm_max : pwm_max) : 0);
    //syslog(LOG_INFO, "%s post %lu", __FUNCTION__, millis());
    //Serial.print("encoder motor 1:");
    //Serial.println(motor1_controller.getEncVal());
    delay(1000);
    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();
    float current_rpm3 = motor3_encoder.getRPM();
    float current_rpm4 = motor4_encoder.getRPM();
    if (current_motor == 0 && tk % run_time == run_time - 1) max_rpm = current_rpm1;
    if (current_motor == 1 && tk % run_time == 0) stopping = current_rpm1;
    if (current_motor == 1 && tk % run_time == run_time - 1) max_rpm = current_rpm2;
    if (total_motors == 2 && current_motor == 0 && tk % run_time == 0) stopping = current_rpm2;
    if (current_motor == 2 && tk % run_time == 0) stopping = current_rpm2;
    if (current_motor == 2 && tk % run_time == run_time - 1) max_rpm = current_rpm3;
    if (current_motor == 3 && tk % run_time == 0) stopping = current_rpm3;
    if (current_motor == 3 && tk % run_time == run_time - 1) max_rpm = current_rpm4;
    if (total_motors == 4 && current_motor == 0 && tk % run_time == 0) stopping = current_rpm4;
    if (tk && tk % run_time == 0) {
        Kinematics::velocities max_linear = kinematics.getVelocities(max_rpm, max_rpm, max_rpm, max_rpm);
	Kinematics::velocities max_angular = kinematics.getVelocities(-max_rpm, max_rpm,-max_rpm, max_rpm);
	Serial.printf("MOTOR%d SPEED %6.2f m/s %6.2f rad/s STOP %6.3f m\n", current_motor ? current_motor : total_motors,
	       max_linear.linear_x, max_angular.angular_z, max_linear.linear_x * stopping / max_rpm);
	syslog(LOG_INFO, "MOTOR%d SPEED %6.2f m/s %6.2f rad/s STOP %6.3f m\n", current_motor ? current_motor : total_motors,
	       max_linear.linear_x, max_angular.angular_z, max_linear.linear_x * stopping / max_rpm);
    }
    Serial.printf("MOTOR%d %s RPM %8.1f %8.1f %8.1f %8.1f\n",
	   current_motor + 1, direction ? "REV" : "FWD",
	   current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    syslog(LOG_INFO, "MOTOR%d %s RPM %8.1f %8.1f %8.1f %8.1f\n",
	   current_motor + 1, direction ? "REV" : "FWD",
	   current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    tk++;
    runWifis();
    runOta();
}
