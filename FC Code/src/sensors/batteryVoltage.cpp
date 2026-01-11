#include "batteryVoltage.h"


void initVoltageSensor(){
  pinMode(BATTERY_VOLTAGE_PIN, INPUT_DISABLE);
}
float readBatteryVoltage(){
  float voltage = analogRead(BATTERY_VOLTAGE_PIN) / 1023.0f * REFERENCE_VOLTAGE;
  return voltage * (BATTERY_RESISTOR_1 + BATTERY_RESISTOR_2) / BATTERY_RESISTOR_2;
} 