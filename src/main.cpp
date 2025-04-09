#include <Arduino.h>
#include <SoftwareSerial.h>

// DC measurement pins
const int dcVoltagePin = A0;  // DC Voltage sensor output
const int dcCurrentPin = A1;  // DC ACS712 output

// Battery measurement pins
const int batteryVoltagePin = A2;  // Battery Voltage sensor output
const int batteryCurrentPin = A3;  // Battery ACS712 output

// Define software serial pins for ESP32 communication
#define ESP_RX 10  // Connect to ESP32 TX
#define ESP_TX 11  // Connect to ESP32 RX
SoftwareSerial espSerial(ESP_RX, ESP_TX);

void setup() {
    Serial.begin(9600);    // Serial communication for debugging
    espSerial.begin(9600); // Serial communication with ESP32
}

void loop() {
    // Read DC Voltage Sensor (0-25V module)
    int rawDcVoltage = analogRead(dcVoltagePin);
    float dcVoltage = (rawDcVoltage / 1023.0) * 25.0;  // Convert to actual voltage

    // Read DC ACS712 Current Sensor
    int rawDcCurrent = analogRead(dcCurrentPin);
    float dcOffset = 512;  // Adjust this if needed (midpoint of 0-1023)
    float dcSensitivity = 0.100; // For ACS712-5A (adjust for 20A or 30A models)
    float dcCurrent = ((rawDcCurrent - dcOffset) * 5.0) / 1023.0 / dcSensitivity;  // Convert to Amps

    // Read Battery Voltage Sensor (0-25V module)
    int rawBatteryVoltage = analogRead(batteryVoltagePin);
    float batteryVoltage = (rawBatteryVoltage / 1023.0) * 25.0;  // Convert to actual voltage

    // Read Battery ACS712 Current Sensor
    int rawBatteryCurrent = analogRead(batteryCurrentPin);
    float batteryOffset = 512;  // Adjust this if needed (midpoint of 0-1023)
    float batterySensitivity = 0.100; // For ACS712-5A (adjust for 20A or 30A models)
    float batteryCurrent = ((rawBatteryCurrent - batteryOffset) * 5.0) / 1023.0 / batterySensitivity;  // Convert to Amps

    // Send to Serial Monitor (for debugging)
    Serial.print("DC: ");
    Serial.print(dcVoltage);
    Serial.print(" V, ");
    Serial.print(dcCurrent);
    Serial.print(" A | Battery: ");
    Serial.print(batteryVoltage);
    Serial.print(" V, ");
    Serial.print(batteryCurrent);
    Serial.println(" A");

    // Send to ESP32 in an easily parsable format (CSV style)
    espSerial.print(dcVoltage);
    espSerial.print(",");
    espSerial.print(dcCurrent);
    espSerial.print(",");
    espSerial.print(batteryVoltage);
    espSerial.print(",");
    espSerial.println(batteryCurrent);

    delay(500);
}