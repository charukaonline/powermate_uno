#include <Arduino.h>

const int dcVoltagePin = A0;  // DC Voltage sensor output to A2
const int dcCurrentPin = A1;  // ACS712 output to A3

void setup() {
    Serial.begin(9600);  // Serial for communication with ESP32
}

void loop() {
    // Read DC Voltage Sensor (0-25V module)
    int rawDCVoltage = analogRead(dcVoltagePin);
    float dcVoltage = (rawDCVoltage / 1023.0) * 25.0;  // Convert to actual voltage

    // Read ACS712 Current Sensor for DC Power
    int rawDCCurrent = analogRead(dcCurrentPin);
    float offset = 512;  // Adjust if needed (midpoint of 0-1023)
    float sensitivity = 0.185; // For ACS712-5A (adjust for 20A or 30A models)
    float dcCurrent = ((rawDCCurrent - offset) * 5.0) / 1023.0 / sensitivity;  // Convert to Amps

    // Send DC voltage & current values to ESP32
    Serial.print(dcVoltage);
    Serial.print(" V, ");
    Serial.print(dcCurrent);
    Serial.println(" A");

    delay(500);
}
