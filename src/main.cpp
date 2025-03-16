#include <Arduino.h>

const int batteryPin = A1; // Analog pin connected to voltage divider
const float R1 = 100000.0; // 100kΩ resistor
const float R2 = 47000.0;  // 47kΩ resistor
const float referenceVoltage = 5.0; // Arduino ADC reference voltage
const int ADCresolution = 1024; // Corrected ADC resolution

void setup() {
    Serial.begin(9600);
}

float readBatteryVoltage() {
    int numSamples = 10;
    long sumADC = 0;
    for (int i = 0; i < numSamples; i++) {
        sumADC += analogRead(batteryPin);
        delay(5); // Short delay to allow ADC to settle
    }
    float avgADC = sumADC / numSamples;
    float measuredVoltage = (avgADC * referenceVoltage) / ADCresolution;
    return measuredVoltage * ((R1 + R2) / R2);
}

void loop() {
    float batteryVoltage = readBatteryVoltage();

    // Estimate battery percentage using linear mapping
    float batteryPercentage = ((batteryVoltage - 9.0) / (12.6 - 9.0)) * 100.0;
    batteryPercentage = constrain(batteryPercentage, 0, 100); // Keep within 0-100%

    // Display results
    Serial.print("Battery Voltage: ");
    Serial.print(batteryVoltage);
    Serial.print(" V, Battery Percentage: ");
    Serial.print(batteryPercentage);
    Serial.println("%");

    delay(1000); // Update every second
}
