#include <Arduino.h>
#include <SoftwareSerial.h>

// DC measurement pins
const int dcVoltagePin = A0;  // DC Voltage sensor output
const int dcCurrentPin = A1;  // DC ACS712 output

// Battery measurement pins
const int batteryVoltagePin = A2;  // Battery Voltage sensor output
const int batteryCurrentPin = A3;  // Battery ACS712 output

// Define software serial pins for ESP32 communication
#define ESP_RX 0  // Connect to ESP32 TX
#define ESP_TX 1  // Connect to ESP32 RX
SoftwareSerial espSerial(ESP_RX, ESP_TX);

// Timing variables
unsigned long previousMillis = 0;
const unsigned long outputInterval = 1000;  // Output interval in milliseconds (1 second)

// Variables for storing real-time measurements
float currentDcVoltage = 0;
float currentDcCurrent = 0;
float currentBatteryVoltage = 0;
float currentBatteryCurrent = 0;

// Variables for averaging
const int numReadings = 10;
int readingCount = 0;
float totalDcVoltage = 0;
float totalDcCurrent = 0;
float totalBatteryVoltage = 0;
float totalBatteryCurrent = 0;

void setup() {
    Serial.begin(9600);    // Serial communication for debugging
    espSerial.begin(9600); // Serial communication with ESP32
}

void loop() {
    // Take readings on every loop iteration for real-time data
    
    // Read DC Voltage Sensor (0-25V module)
    int rawDcVoltage = analogRead(dcVoltagePin);
    currentDcVoltage = (rawDcVoltage / 1023.0) * 25.0;
    totalDcVoltage += currentDcVoltage;

    // Read DC ACS712 Current Sensor
    int rawDcCurrent = analogRead(dcCurrentPin);
    float dcOffset = 512;
    float dcSensitivity = 0.100;
    currentDcCurrent = ((rawDcCurrent - dcOffset) * 5.0) / 1023.0 / dcSensitivity;
    totalDcCurrent += currentDcCurrent;

    // Read Battery Voltage Sensor (0-25V module)
    int rawBatteryVoltage = analogRead(batteryVoltagePin);
    currentBatteryVoltage = (rawBatteryVoltage / 1023.0) * 25.0;
    totalBatteryVoltage += currentBatteryVoltage;

    // Read Battery ACS712 Current Sensor
    int rawBatteryCurrent = analogRead(batteryCurrentPin);
    float batteryOffset = 512;
    float batterySensitivity = 0.100;
    currentBatteryCurrent = ((rawBatteryCurrent - batteryOffset) * 5.0) / 1023.0 / batterySensitivity;
    totalBatteryCurrent += currentBatteryCurrent;

    readingCount++;

    // Output at regular intervals without blocking the measurements
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= outputInterval) {
        // Save the current time
        previousMillis = currentMillis;
        
        // Calculate averages
        float avgDcVoltage = totalDcVoltage / readingCount;
        float avgDcCurrent = totalDcCurrent / readingCount;
        float avgBatteryVoltage = totalBatteryVoltage / readingCount;
        float avgBatteryCurrent = totalBatteryCurrent / readingCount;
        
        // Send to Serial Monitor (for debugging)
        Serial.print("DC: ");
        Serial.print(avgDcVoltage);
        Serial.print(" V, ");
        Serial.print(avgDcCurrent);
        Serial.print(" A | Battery: ");
        Serial.print(avgBatteryVoltage);
        Serial.print(" V, ");
        Serial.print(avgBatteryCurrent);
        Serial.println(" A");

        // Send to ESP32 in an easily parsable format (CSV style)
        espSerial.print(avgDcVoltage);
        espSerial.print(",");
        espSerial.print(avgDcCurrent);
        espSerial.print(",");
        espSerial.print(avgBatteryVoltage);
        espSerial.print(",");
        espSerial.println(avgBatteryCurrent);
        
        // Reset for next averaging period
        totalDcVoltage = 0;
        totalDcCurrent = 0;
        totalBatteryVoltage = 0;
        totalBatteryCurrent = 0;
        readingCount = 0;
    }
    
    // You can add other real-time operations here
}