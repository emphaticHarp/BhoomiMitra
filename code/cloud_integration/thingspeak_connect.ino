/**
 * BhoomiMitra - Smart Agriculture System
 * ThingSpeak Cloud Integration Module
 * 
 * This code handles the connection between the ESP32 and ThingSpeak IoT platform
 * for storing sensor data and visualizing irrigation system metrics.
 * 
 * Author: Soumyajyoti Banik
 * License: MIT
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include "ThingSpeak.h"

// Define the number of sensors and pumps
#define SENSOR_COUNT 9

// WiFi credentials
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// ThingSpeak credentials
#define CHANNEL_ID 0000000           // Replace with your ThingSpeak Channel ID
#define WRITE_API_KEY "YOUR_API_KEY" // Replace with your ThingSpeak Write API Key

// Define sensor and relay pins
int moisturePins[SENSOR_COUNT] = {34, 35, 32, 33, 25, 26, 27, 14, 12};
int relayPins[SENSOR_COUNT] = {23, 22, 21, 19, 18, 5, 4, 2, 15};

// Variables to store sensor readings
int moistureValues[SENSOR_COUNT];
bool pumpStatus[SENSOR_COUNT];

// Calibration values
const int AIR_VALUE = 3200;    // Value when sensor is in air (dry)
const int WATER_VALUE = 1000;  // Value when sensor is in water (wet)

// Timing variables
unsigned long previousMillis = 0;
const long interval = 20000;  // Data upload interval (20 seconds)

// ThingSpeak client
WiFiClient client;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("BhoomiMitra - ThingSpeak Cloud Integration");
  Serial.println("------------------------------------------");
  
  // Initialize relay pins as outputs and turn OFF all pumps
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], HIGH); // Turn OFF all pumps initially
    pumpStatus[i] = false;
  }
  
  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  
  // Initialize ThingSpeak
  ThingSpeak.begin(client);
  
  Serial.println("System initialized. Starting ThingSpeak monitoring...");
  Serial.println();
}

void loop() {
  // Check if WiFi is still connected
  if (WiFi.status() == WL_CONNECTED) {
    unsigned long currentMillis = millis();
    
    // Upload data at specified intervals
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      // Read all moisture sensors
      readMoistureSensors();
      
      // Upload data to ThingSpeak
      uploadToThingSpeak();
    }
  } else {
    // Try to reconnect to WiFi
    Serial.println("WiFi disconnected. Attempting to reconnect...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    // Wait for reconnection
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.print("Reconnected with IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println();
      Serial.println("Failed to reconnect. Will try again later.");
    }
  }
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

// Function to upload data to ThingSpeak
void uploadToThingSpeak() {
  Serial.println("Uploading data to ThingSpeak...");
  
  // Create a ThingSpeak multi-field update object
  ThingSpeak.setField(1, calculateAverageMoisture());
  
  // We can only send 8 fields to ThingSpeak, so we'll need to be selective
  // For this example, we'll send the first 7 individual moisture readings and the average
  for (int i = 0; i < min(7, SENSOR_COUNT); i++) {
    // Calculate moisture percentage (0% = dry, 100% = wet)
    int moisturePercent = map(moistureValues[i], AIR_VALUE, WATER_VALUE, 0, 100);
    moisturePercent = constrain(moisturePercent, 0, 100);
    
    // Set field (field 2-8 for individual sensors)
    ThingSpeak.setField(i + 2, moisturePercent);
  }
  
  // Write data to ThingSpeak
  int httpCode = ThingSpeak.writeFields(CHANNEL_ID, WRITE_API_KEY);
  
  if (httpCode == 200) {
    Serial.println("Data sent to ThingSpeak successfully");
  } else {
    Serial.print("Error sending data to ThingSpeak. HTTP error code: ");
    Serial.println(httpCode);
  }
  
  // ThingSpeak has a rate limit of 15 seconds between updates
  delay(15000);
}

// Function to calculate average moisture across all sensors
int calculateAverageMoisture() {
  long sum = 0;
  
  for (int i = 0; i < SENSOR_COUNT; i++) {
    // Calculate moisture percentage for each sensor
    int moisturePercent = map(moistureValues[i], AIR_VALUE, WATER_VALUE, 0, 100);
    moisturePercent = constrain(moisturePercent, 0, 100);
    
    sum += moisturePercent;
  }
  
  return sum / SENSOR_COUNT;
}

// Function to check if any zone needs irrigation
bool checkIrrigationNeeded() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    // Calculate moisture percentage
    int moisturePercent = map(moistureValues[i], AIR_VALUE, WATER_VALUE, 0, 100);
    moisturePercent = constrain(moisturePercent, 0, 100);
    
    // If moisture is below 30%, irrigation is needed
    if (moisturePercent < 30) {
      return true;
    }
  }
  
  return false;
}

// Function to count active pumps
int countActivePumps() {
  int count = 0;
  
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (pumpStatus[i]) {
      count++;
    }
  }
  
  return count;
}
