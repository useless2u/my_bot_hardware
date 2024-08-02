#include <Arduino.h>
#include <sensor_msgs/msg/battery_state.h>
#include "config.h"

#if defined USE_INA219 || defined USE_INA226
#ifdef USE_INA219
  #include <INA219_WE.h>
  #define INA219_ADDRESS 0x42
  INA219_WE ina = INA219_WE(INA219_ADDRESS);
#else
  #define INA226_WE_COMPATIBILITY_MODE_
  #include <INA226_WE.h>
  #define INA226_ADDRESS 0x40
  INA226_WE ina = INA226_WE(INA226_ADDRESS);
#endif

float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0;
bool ina_overflow = false;

void initBattery(){
  if(!ina.init()){
    Serial.println("ina not connected!");
  } else {
    Serial.println("ina2xx connected!");
  }
  #ifdef USE_INA219
  ina.setADCMode(BIT_MODE_9);
  ina.setPGain(PG_320);
  ina.setBusRange(BRNG_16);
  ina.setShuntSizeInOhms(0.01); // used in ina.
  #else
  ina.setResistorRange(0.002,41.0);
  #endif
}

void InaDataUpdate(){
  shuntVoltage_mV = ina.getShuntVoltage_mV();
  busVoltage_V = ina.getBusVoltage_V();
  current_mA = ina.getCurrent_mA();
  power_mW = ina.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  /*Serial.print("busVoltag_V:");
  Serial.println(busVoltage_V);
  */
  #ifdef USE_INA219
    ina_overflow = ina.getOverflow();
  #endif
}
#else
void initBattery() {
#ifdef BATTERY_PIN
    analogReadResolution(12);
#endif
}
#endif

#ifdef BATTERY_PIN
double readVoltage(int pin) {
  int reading = 0;
  int i;
  for (i = 0; i < 4; i++) // smoothing
    reading += analogRead(pin);
  reading /= i;
  return BATTERY_ADJUST(reading);
}
#endif

sensor_msgs__msg__BatteryState battery_msg_ = { .current = NAN, .charge = NAN,
    .capacity = NAN, .design_capacity = NAN, .percentage = NAN, .present =  true };
sensor_msgs__msg__BatteryState getBattery()
{
#ifdef BATTERY_PIN
    battery_msg_.voltage = readVoltage(BATTERY_PIN);
#endif
#if defined USE_INA219 || defined USE_INA226
    // read voltage
    InaDataUpdate();
    battery_msg_.voltage = loadVoltage_V;
    battery_msg_.current = -current_mA / 1000 * 2.00/1.72; // Amp, minu when discharge
#endif
    return battery_msg_;
}

/** https://github.com/rlogiacco/BatterySense/blob/master/Battery.h
 *
 * Symmetric sigmoidal approximation
 * https://www.desmos.com/calculator/7m9lu26vpy
 *
 * c - c / (1 + k*x/v)^3
 */
static inline float sigmoidal(float voltage, float minVoltage, float maxVoltage) {
    // slow
    // float result = 110 - (110 / (1 + pow(1.468 * (voltage - minVoltage)/(maxVoltage - minVoltage), 6)));

    // steep
    // float result = 102 - (102 / (1 + pow(1.621 * (voltage - minVoltage)/(maxVoltage - minVoltage), 8.1)));

    // normal
    float result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage)/(maxVoltage - minVoltage), 5.5)));
    return result >= 100 ? 100 : result;
}

/**
 * Asymmetric sigmoidal approximation
 * https://www.desmos.com/calculator/oyhpsu8jnw
 *
 * c - c / [1 + (k*x/v)^4.5]^3
 */
static inline float asigmoidal(float voltage, float minVoltage, float maxVoltage) {
    float result = 101 - (101 / pow(1 + pow(1.33 * (voltage - minVoltage)/(maxVoltage - minVoltage) ,4.5), 3));
    return result >= 100 ? 100 : result;
}

void getBatteryPercentage(sensor_msgs__msg__BatteryState *msg)
{
#if defined(BATTERY_MIN) && defined(BATTERY_MAX)
    msg->percentage = sigmoidal(msg->voltage, BATTERY_MIN, BATTERY_MAX) / 100;
#ifdef BATTERY_CAP
    msg->design_capacity = BATTERY_CAP;
    msg->capacity = BATTERY_CAP * msg->percentage;
#endif
#endif
}
