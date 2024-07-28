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
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "battery.h"
#include "range.h"
#include "lidar.h"
#include "wifis.h"
#include "ota.h"
#include "pwm.h"

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

// #define DEBUG

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

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

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
unsigned total_motors = 4;

void setup()
{
    Serial.begin(BAUDRATE);
    initLed();
#ifdef BOARD_INIT // board specific setup
    BOARD_INIT;
#endif

    initPwm();
    motor1_controller.begin();
    motor2_controller.begin();
    motor3_controller.begin();
    motor4_controller.begin();
    initWifis();
    initOta();

    imu.init();
    mag.init();
    initBattery();
    initRange();

    if(Kinematics::LINO_BASE == Kinematics::DIFFERENTIAL_DRIVE)
    {
        total_motors = 2;
    }
    motor1_encoder.getRPM();
    motor2_encoder.getRPM();
    motor3_encoder.getRPM();
    motor4_encoder.getRPM();

#ifdef BOARD_INIT_LATE // board specific setup
    BOARD_INIT_LATE;
#endif
    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());
    delay(2000);
}

unsigned runs = 12;
const unsigned ticks = 20;
const float dt = ticks * 0.001;
const unsigned run_time = 1000; // 1s
const unsigned buf_size = run_time / ticks * 4;
Kinematics::velocities buf[buf_size];
float batt[buf_size];
float imu_max_acc_x, imu_min_acc_x;
unsigned idx = 0;
void record(unsigned n) {
    unsigned i;
    for (i = 0; i < n; i++, idx++) {
        float rpm1 = motor1_encoder.getRPM();
        float rpm2 = motor2_encoder.getRPM();
        float rpm3 = motor3_encoder.getRPM();
        float rpm4 = motor4_encoder.getRPM();
        imu_msg = imu.getData();
        battery_msg = getBattery();
        float imu_acc_x = imu_msg.linear_acceleration.x;
        if (imu_acc_x > imu_max_acc_x) imu_max_acc_x = imu_acc_x;
        if (imu_acc_x < imu_min_acc_x) imu_min_acc_x = imu_acc_x;

        if (idx < buf_size) {
            buf[idx] = kinematics.getVelocities(rpm1, rpm2, rpm3, rpm4);
            batt[idx] = battery_msg.voltage;
        }
        delay(ticks);
        runWifis();
        runOta();
    }
}

void dump_record(void) {
    float max_vel_x = 0, min_vel_x = 0, max_acc_x = 0, min_acc_x = 0;
    float max_vel_y = 0, min_vel_y = 0, max_acc_y = 0, min_acc_y = 0;
    float max_vel_z = 0, min_vel_z = 0, max_acc_z = 0, min_acc_z = 0;
    float dist = 0;
    float avg_batt = 0, min_batt = batt[0];
    for (idx = 0; idx < buf_size; idx++) {
        float vel_x = buf[idx].linear_x;
        float vel_y = buf[idx].linear_y;
        float vel_z = buf[idx].angular_z;
        if (vel_x > max_vel_x) max_vel_x = vel_x;
        if (vel_x < min_vel_x) min_vel_x = vel_x;
        if (vel_y > max_vel_y) max_vel_y = vel_y;
        if (vel_y < min_vel_y) min_vel_y = vel_y;
        if (vel_z > max_vel_z) max_vel_z = vel_z;
        if (vel_z < min_vel_z) min_vel_z = vel_z;
	if (min_batt > batt[idx]) min_batt = batt[idx];
	avg_batt += batt[idx];
#ifdef DEBUG
        Serial.printf("%04d VEL %6.2f %6.2f m/s  %6.2f rad/s BAT %5.2fV\n",
            idx, vel_x, vel_y, vel_z, batt[idx]);
        syslog(LOG_INFO, "%04d VEL %6.2f %6.2f m/s  %6.2f rad BAT %5.2fV/s",
            idx, vel_x, vel_y, vel_z, batt[idx]);
#endif
    }
    for (idx = 0; idx < buf_size; idx++) {
        unsigned prev = idx ? (idx -1) : 0;
        float acc_x = (buf[idx].linear_x - buf[prev].linear_x) / dt;
        float acc_y = (buf[idx].linear_y - buf[prev].linear_y) / dt;
        float acc_z = (buf[idx].angular_z - buf[prev].angular_z) / dt;
        if (acc_x > max_acc_x) max_acc_x = acc_x;
        if (acc_x < min_acc_x) min_acc_x = acc_x;
        if (acc_y > max_acc_y) max_acc_y = acc_y;
        if (acc_y < min_acc_y) min_acc_y = acc_y;
        if (acc_z > max_acc_z) max_acc_z = acc_z;
        if (acc_z < min_acc_z) min_acc_z = acc_z;
#ifdef DEBUG
        Serial.printf("%04d ACC %6.2f %6.2f m/s2 %6.2f rad/s2 BAT %5.2fV\n",
            idx, acc_x, acc_y, acc_z, batt[idx]);
        syslog(LOG_INFO, "%04d ACC %6.2f %6.2f m/s2 %6.2f rad/s2 BAT %5.2fV",
            idx, acc_x, acc_y, acc_z, batt[idx]);
#endif
    }
    if (runs & 1) {
        for (idx = buf_size / 4; idx < buf_size / 2; idx++)
            dist += buf[idx].linear_x * dt;
        for (idx = 0; idx < buf_size / 4; idx++)
            if (buf[idx].linear_x > max_vel_x * 0.9) break;
    } else {
        for (idx = buf_size / 4; idx < buf_size / 2; idx++)
	    dist += buf[idx].angular_z * dt;
        for (idx = 0; idx < buf_size / 4; idx++)
            if (buf[idx].angular_z > max_vel_z * 0.9) break;
    }
    avg_batt /= buf_size;

    Serial.printf("MAX VEL %6.2f %6.2f m/s  %6.2f rad/s\n",
	max_vel_x, max_vel_y, max_vel_z);
    Serial.printf("MIN VEL %6.2f %6.2f m/s  %6.2f rad/s\n",
	min_vel_x, min_vel_y, min_vel_z);
    Serial.printf("MAX ACC %6.2f %6.2f m/s2  %6.2f rad/s2\n",
	max_acc_x, max_acc_y, max_acc_z);
    Serial.printf("MIN ACC %6.2f %6.2f m/s2  %6.2f rad/s2\n",
	min_acc_x, min_acc_y, min_acc_z);
    Serial.printf("IMU ACC %6.2f %6.2f m/s2\n", imu_max_acc_x, imu_min_acc_x);
    Serial.printf("time to 0.9x max vel %6.2f sec\n", idx * dt);
    Serial.printf("distance to stop %6.2f %s\n", dist, (runs & 1) ? "m" : "rad");
    Serial.printf("BAT %5.2fV MIN %5.2fV\n", avg_batt, min_batt);

    syslog(LOG_INFO, "MAX VEL %6.2f %6.2f m/s  %6.2f rad/s",
	max_vel_x, max_vel_y, max_vel_z);
    syslog(LOG_INFO, "MIN VEL %6.2f %6.2f m/s  %6.2f rad/s",
	min_vel_x, min_vel_y, min_vel_z);
    syslog(LOG_INFO, "MAX ACC %6.2f %6.2f m/s2  %6.2f rad/s2",
	max_acc_x, max_acc_y, max_acc_z);
    syslog(LOG_INFO, "MIN ACC %6.2f %6.2f m/s2  %6.2f rad/s2",
	min_acc_x, min_acc_y, min_acc_z);
    syslog(LOG_INFO, "IMU ACC %6.2f %6.2f m/s2", imu_max_acc_x, imu_min_acc_x);
    syslog(LOG_INFO, "time to 0.9x max vel %6.2f sec", idx * dt);
    syslog(LOG_INFO, "distance to stop %6.2f %s\n", dist, (runs & 1) ? "m" : "rad");
    syslog(LOG_INFO, "BAT %5.2fV MIN %5.2fV\n", avg_batt, min_batt);
}

void loop() {
    float pwm_max = PWM_MAX;
    float pwm_min = -pwm_max;

    while (runs > 0) {
        runs--;
        idx = 0;
        imu_max_acc_x = 0;
        imu_min_acc_x = 0;
        // full speed forward / spin counterclockwise
        setLed(HIGH);
        motor1_controller.spin((runs & 1) ? pwm_max : pwm_min);
        motor2_controller.spin(pwm_max);
        motor3_controller.spin((runs & 1) ? pwm_max : pwm_min);
        motor4_controller.spin(pwm_max);
        record(run_time / ticks);
        // stop
        setLed(LOW);
        motor1_controller.spin(0);
        motor2_controller.spin(0);
        motor3_controller.spin(0);
        motor4_controller.spin(0);
        record(run_time / ticks);
        // full speed backward / spin clockwise
        setLed(HIGH);
        motor1_controller.spin((runs & 1) ? pwm_min : pwm_max);
        motor2_controller.spin(pwm_min);
        motor3_controller.spin((runs & 1) ? pwm_min : pwm_max);
        motor4_controller.spin(pwm_min);
        record(run_time / ticks);
        // stop
        setLed(LOW);
        motor1_controller.spin(0);
        motor2_controller.spin(0);
        motor3_controller.spin(0);
        motor4_controller.spin(0);
        record(run_time / ticks);
        // print result
        Serial.printf("MAX PWM %6.1f %6.1f\n", pwm_max, pwm_min);
        syslog(LOG_INFO, "MAX PWM %6.1f %6.1f", pwm_max, pwm_min);
        dump_record();
        if ((runs & 3) == 0) {
            pwm_max /= 2;
            pwm_min /= 2;
        }
    }

    // idle
    delay(100);
    runWifis();
    runOta();
}
