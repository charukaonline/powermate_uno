#include <Arduino.h>

// Battery Monitoring
const int batteryVoltagePin = A0;  // Battery Voltage sensor output to A0
const int batteryCurrentPin = A1;  // Battery ACS712 sensor output to A1

// DC Power Monitoring
const int dcVoltagePin = A2;  // DC Voltage sensor output to A2
const int dcCurrentPin = A3;  // DC ACS712 sensor output to A3

void setup() {
    Serial.begin(9600);  // Serial for communication with ESP32
}

void loop() {
    // 🔋 **Read Battery Voltage (0-25V module)**
    int rawBatteryVoltage = analogRead(batteryVoltagePin);
    float batteryVoltage = (rawBatteryVoltage / 1023.0) * 25.0;  // Convert to actual voltage

    // 🔋 **Read Battery Current (ACS712)**
    int rawBatteryCurrent = analogRead(batteryCurrentPin);
    float batteryOffset = 512;  // Adjust if needed (midpoint of 0-1023)
    float batterySensitivity = 0.185; // For ACS712-5A (adjust for 20A or 30A models)
    float batteryCurrent = ((rawBatteryCurrent - batteryOffset) * 5.0) / 1023.0 / batterySensitivity;

    // ⚡ **Read DC Power Voltage (0-25V module)**
    int rawDCVoltage = analogRead(dcVoltagePin);
    float dcVoltage = (rawDCVoltage / 1023.0) * 25.0;  // Convert to actual voltage

    // ⚡ **Read DC Power Current (ACS712)**
    int rawDCCurrent = analogRead(dcCurrentPin);
    float dcOffset = 512;  // Adjust if needed (midpoint of 0-1023)
    float dcSensitivity = 0.185; // For ACS712-5A (adjust for 20A or 30A models)
    float dcCurrent = ((rawDCCurrent - dcOffset) * 5.0) / 1023.0 / dcSensitivity;

    // 🔄 **Send Battery Voltage & Current Data to ESP32**
    Serial.print(batteryVoltage);
    Serial.print(" V, ");
    Serial.print(batteryCurrent);
    Serial.println(" A");

    // ⚡ **Send DC Power Voltage & Current Data to ESP32**
    Serial.print(dcVoltage);
    Serial.print(" V, ");
    Serial.print(dcCurrent);
    Serial.println(" A");

    delay(10000);
}
