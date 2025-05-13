/**
 * BhoomiMitra - Smart Agriculture System
 * ESP32 Irrigation Control Module
 * 
 * This code handles soil moisture sensing and automated irrigation control
 * for 9 different plant zones using capacitive soil moisture sensors and
 * water pumps controlled via relays.
 * 
 * Author: Soumyajyoti Banik
 * License: MIT
 */

// Define the number of sensors and pumps
#define SENSOR_COUNT 9

// Define pins for moisture sensors (analog inputs)
int moisturePins[SENSOR_COUNT] = {34, 35, 32, 33, 25, 26, 27, 14, 12};

// Define pins for relay control (digital outputs)
int relayPins[SENSOR_COUNT] = {23, 22, 21, 19, 18, 5, 4, 2, 15};

// Moisture thresholds and pump timing
const int DRY_THRESHOLD = 2500;      // Adjust based on sensor calibration (higher value = drier soil)
const int WATERING_TIME = 5000;      // Pump activation time in milliseconds
const int READING_INTERVAL = 10000;  // Time between sensor readings in milliseconds

// Variables to store sensor readings
int moistureValues[SENSOR_COUNT];
bool pumpStatus[SENSOR_COUNT];

// Calibration values (can be adjusted based on sensor readings)
const int AIR_VALUE = 3200;    // Value when sensor is in air (dry)
const int WATER_VALUE = 1000;  // Value when sensor is in water (wet)

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("BhoomiMitra - ESP32 Irrigation Control System");
  Serial.println("-----------------------------------------------");
  
  // Initialize relay pins as outputs and turn OFF all pumps
  // Note: Relays are typically active LOW (LOW = ON, HIGH = OFF)
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], HIGH); // Turn OFF all pumps initially
    pumpStatus[i] = false;
  }
  
  // Short delay for system stabilization
  delay(1000);
  
  Serial.println("System initialized. Starting moisture monitoring...");
  Serial.println();
}

void loop() {
  // Read all moisture sensors
  readMoistureSensors();
  
  // Display readings
  displayReadings();
  
  // Control irrigation based on moisture levels
  controlIrrigation();
  
  // Wait before next reading cycle
  delay(READING_INTERVAL);
}

// Function to read all moisture sensors
void readMoistureSensors() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    // Read the analog value from sensor
    moistureValues[i] = analogRead(moisturePins[i]);
    
    // Add a small delay between readings to avoid ADC conflicts
    delay(100);
  }
}

// Function to display sensor readings
void displayReadings() {
  Serial.println("Current Moisture Readings:");
  Serial.println("-------------------------");
  
  for (int i = 0; i < SENSOR_COUNT; i++) {
    // Calculate moisture percentage (0% = dry, 100% = wet)
    int moisturePercent = map(moistureValues[i], AIR_VALUE, WATER_VALUE, 0, 100);
    
    // Constrain to valid range
    moisturePercent = constrain(moisturePercent, 0, 100);
    
    Serial.print("Zone ");
    Serial.print(i + 1);
    Serial.print(": Raw: ");
    Serial.print(moistureValues[i]);
    Serial.print(" | Percent: ");
    Serial.print(moisturePercent);
    Serial.print("% | Pump: ");
    Serial.println(pumpStatus[i] ? "ON" : "OFF");
  }
  
  Serial.println();
}

// Function to control irrigation based on moisture readings
void controlIrrigation() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    // Check if soil is too dry (high sensor value = dry soil)
    if (moistureValues[i] > DRY_THRESHOLD) {
      // Soil is dry, activate pump
      Serial.print("Zone ");
      Serial.print(i + 1);
      Serial.println(" is dry. Activating irrigation...");
      
      // Turn ON pump (active LOW)
      digitalWrite(relayPins[i], LOW);
      pumpStatus[i] = true;
      
      // Keep pump ON for specified watering time
      delay(WATERING_TIME);
      
      // Turn OFF pump
      digitalWrite(relayPins[i], HIGH);
      pumpStatus[i] = false;
      
      Serial.print("Zone ");
      Serial.print(i + 1);
      Serial.println(" irrigation completed.");
    }
  }
}

// Additional functions for future expansion

// Function to calibrate sensors
void calibrateSensors() {
  // To be implemented
  // This would measure and store the air and water values for each sensor
}

// Function to manually control a specific pump
void manualPumpControl(int pumpIndex, bool turnOn) {
  if (pumpIndex >= 0 && pumpIndex < SENSOR_COUNT) {
    digitalWrite(relayPins[pumpIndex], turnOn ? LOW : HIGH);
    pumpStatus[pumpIndex] = turnOn;
    
    Serial.print("Manual control: Pump ");
    Serial.print(pumpIndex + 1);
    Serial.println(turnOn ? " turned ON" : " turned OFF");
  }
}
