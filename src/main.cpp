// ACS712 Current Sensor (5A, 20A, or 30A) with Arduino Uno

#include <Arduino.h>

const int currentSensorPin = A0; // ACS712 sensor output to Arduino A0
const float VCC = 5.0; // Arduino operating voltage (5V)
const int ADCresolution = 1023; // 10-bit ADC resolution

// Adjust these values based on your specific ACS712 model:
// 5A model: 0.185 V/A
// 20A model: 0.100 V/A
// 30A model: 0.066 V/A
const float ACS712_Sensitivity = 0.100; // Sensitivity in V/A

// Zero current calibration value (needs to be measured)
float offsetVoltage = 2.5; // This is theoretical, should be calibrated

void setup() {
    Serial.begin(9600);
    
    // Calibration routine to find the actual zero-current offset voltage
    long sum = 0;
    for(int i = 0; i < 500; i++) {
        sum += analogRead(currentSensorPin);
        delay(1);
    }
    offsetVoltage = (sum / 500.0) * VCC / ADCresolution;
    
    Serial.print("Calibrated offset voltage: ");
    Serial.print(offsetVoltage, 3);
    Serial.println(" V");
}

void loop() {
    int rawADC = analogRead(currentSensorPin); // Read ADC value
    float voltage = (rawADC * VCC) / ADCresolution; // Convert ADC value to voltage
    
    // Calculate actual current (in Amperes)
    float current = (voltage - offsetVoltage) / ACS712_Sensitivity;

    // Display results in Serial Monitor
    Serial.print("Raw ADC: ");
    Serial.print(rawADC);
    Serial.print(" | Voltage: ");
    Serial.print(voltage, 3);
    Serial.print(" V | Current: ");
    Serial.print(current, 3); // 3 decimal places
    Serial.println(" A");

    delay(1000); // Update every second
}