#include <Arduino.h>

const int voltagePin = A0;  // Voltage sensor output to A0
const int currentPin = A1;  // ACS712 output to A1

void setup() {
    Serial.begin(9600); // Serial1 for communication with ESP32
}

void loop() {
    // Read Voltage Sensor (0-25V module)
    int rawVoltage = analogRead(voltagePin);
    float voltage = (rawVoltage / 1023.0) * 25.0;  // Convert to actual voltage

    // Read ACS712 Current Sensor
    int rawCurrent = analogRead(currentPin);
    float offset = 512;  // Adjust this if needed (midpoint of 0-1023)
    float sensitivity = 0.185; // For ACS712-5A (adjust for 20A or 30A models)
    float current = ((rawCurrent - offset) * 5.0) / 1023.0 / sensitivity;  // Convert to Amps

    // Send voltage & current values to ESP32
    Serial.print(voltage);
    Serial.print(" V, ");
    Serial.print(current);
    Serial.println(" A");

    delay(500);
}
