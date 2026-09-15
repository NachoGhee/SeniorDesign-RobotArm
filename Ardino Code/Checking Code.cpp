#include <Arduino.h>
#include <Wire.h>

// Define the analog pin connected to your voltage divider
const int VOLTAGE_PIN = 34; 
// Calibration factor based on your specific resistor values (e.g., 100k and 10k)
const float VOLTAGE_DIVIDER_RATIO = 11.0; 
const float MIN_SAFE_VOLTAGE = 22.0;

// Example I2C Addresses for the encoders (Update with actual MT6701 addresses)
const int ENCODER_ADDRESSES[6] = {0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B};

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Start the I2C bus for the encoders
  
  delay(1000);
  Serial.println("=== POSE SYSTEM DIAGNOSTICS START ===");

  bool powerOk = checkMainPower();
  bool encodersOk = checkEncoders();

  if (powerOk && encodersOk) {
    Serial.println("SYSTEM GO: All components responding and powered.");
    // Proceed to enable stepper drivers here
  } else {
    Serial.println("SYSTEM HALT: Diagnostic failure. Motors disabled.");
    // Trigger E-Stop state
    while(1); 
  }
}

void loop() {
  // Main control loop runs here after passing diagnostics
}

bool checkMainPower() {
  int rawADC = analogRead(VOLTAGE_PIN);
  // Convert 12-bit ADC reading (0-4095) to actual voltage
  float pinVoltage = (rawADC / 4095.0) * 3.3; 
  float actualVoltage = pinVoltage * VOLTAGE_DIVIDER_RATIO;
  
  Serial.print("Main Power Voltage: ");
  Serial.print(actualVoltage);
  Serial.println(" V");

  if (actualVoltage < MIN_SAFE_VOLTAGE) {
    Serial.println("ERROR: Voltage too low!");
    return false;
  }
  return true;
}

bool checkEncoders() {
  bool allConnected = true;
  
  for (int i = 0; i < 6; i++) {
    Wire.beginTransmission(ENCODER_ADDRESSES[i]);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Encoder ");
      Serial.print(i + 1);
      Serial.println(" ... CONNECTED");
    } else {
      Serial.print("Encoder ");
      Serial.print(i + 1);
      Serial.println(" ... MISSING/FAULT");
      allConnected = false;
    }
  }
  return allConnected;
}