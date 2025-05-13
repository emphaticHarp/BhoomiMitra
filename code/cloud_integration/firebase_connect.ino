/**
 * BhoomiMitra - Smart Agriculture System
 * Firebase Cloud Integration Module
 * 
 * This code handles the connection between the ESP32 and Firebase Realtime Database
 * for storing sensor data and controlling the irrigation system remotely.
 * 
 * Author: Soumyajyoti Banik
 * License: MIT
 */

#include <WiFi.h>
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

// Define the number of sensors and pumps
#define SENSOR_COUNT 9

// WiFi credentials
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// Firebase credentials
#define API_KEY "YOUR_FIREBASE_API_KEY"
#define DATABASE_URL "YOUR_FIREBASE_DATABASE_URL"
#define USER_EMAIL "YOUR_FIREBASE_USER_EMAIL"
#define USER_PASSWORD "YOUR_FIREBASE_USER_PASSWORD"

// Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Sensor pins (same as in the main irrigation code)
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
const long interval = 60000;  // Data upload interval (1 minute)

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("BhoomiMitra - Firebase Cloud Integration");
  Serial.println("----------------------------------------");
  
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
  
  // Initialize Firebase
  initFirebase();
  
  Serial.println("System initialized. Starting cloud monitoring...");
  Serial.println();
}

void loop() {
  // Check if Firebase connection is ready
  if (Firebase.ready()) {
    unsigned long currentMillis = millis();
    
    // Upload data at specified intervals
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      // Read all moisture sensors
      readMoistureSensors();
      
      // Upload data to Firebase
      uploadSensorData();
      
      // Check for remote control commands
      checkRemoteCommands();
    }
  }
}

// Function to initialize Firebase
void initFirebase() {
  // Configure Firebase credentials
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  
  // Configure user sign-in
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  
  // Initialize the library with the Firebase authen and config
  Firebase.begin(&config, &auth);
  
  // Enable auto reconnect
  Firebase.reconnectWiFi(true);
  
  // Set database read timeout
  Firebase.setReadTimeout(fbdo, 1000 * 60);
  
  // Set database size limit
  Firebase.setwriteSizeLimit(fbdo, "tiny");
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

// Function to upload sensor data to Firebase
void uploadSensorData() {
  Serial.println("Uploading data to Firebase...");
  
  // Create a timestamp
  String timestamp = String(millis());
  
  // Create a parent path for this data set
  String parentPath = "/sensor_data/" + timestamp;
  
  // Use JSON to batch update
  FirebaseJson json;
  
  // Add timestamp
  json.set("timestamp", timestamp);
  
  // Add all moisture readings
  for (int i = 0; i < SENSOR_COUNT; i++) {
    // Calculate moisture percentage (0% = dry, 100% = wet)
    int moisturePercent = map(moistureValues[i], AIR_VALUE, WATER_VALUE, 0, 100);
    moisturePercent = constrain(moisturePercent, 0, 100);
    
    // Add to JSON object
    String sensorKey = "moisture_" + String(i + 1);
    json.set(sensorKey, moisturePercent);
    
    // Add pump status
    String pumpKey = "pump_" + String(i + 1);
    json.set(pumpKey, pumpStatus[i]);
  }
  
  // Set JSON data to Firebase
  if (Firebase.setJSON(fbdo, parentPath, json)) {
    Serial.println("Data uploaded successfully");
  } else {
    Serial.println("Failed to upload data");
    Serial.println("Reason: " + fbdo.errorReason());
  }
  
  // Also update the latest readings
  if (Firebase.setJSON(fbdo, "/latest_readings", json)) {
    Serial.println("Latest readings updated");
  } else {
    Serial.println("Failed to update latest readings");
    Serial.println("Reason: " + fbdo.errorReason());
  }
}

// Function to check for remote control commands
void checkRemoteCommands() {
  Serial.println("Checking for remote commands...");
  
  // Check for manual irrigation commands
  for (int i = 0; i < SENSOR_COUNT; i++) {
    String commandPath = "/commands/irrigate_zone_" + String(i + 1);
    
    if (Firebase.getBool(fbdo, commandPath)) {
      bool command = fbdo.boolData();
      
      // If command is true, irrigate this zone
      if (command) {
        Serial.print("Remote command: Irrigate Zone ");
        Serial.println(i + 1);
        
        // Turn ON pump
        digitalWrite(relayPins[i], LOW);
        pumpStatus[i] = true;
        
        // Update pump status in Firebase
        Firebase.setBool(fbdo, "/latest_readings/pump_" + String(i + 1), true);
        
        // Keep pump ON for specified time (5 seconds)
        delay(5000);
        
        // Turn OFF pump
        digitalWrite(relayPins[i], HIGH);
        pumpStatus[i] = false;
        
        // Update pump status in Firebase
        Firebase.setBool(fbdo, "/latest_readings/pump_" + String(i + 1), false);
        
        // Reset the command
        Firebase.setBool(fbdo, commandPath, false);
        
        Serial.print("Zone ");
        Serial.print(i + 1);
        Serial.println(" irrigation completed.");
      }
    }
  }
  
  // Check for system-wide commands
  if (Firebase.getBool(fbdo, "/commands/system_reset")) {
    bool resetCommand = fbdo.boolData();
    
    if (resetCommand) {
      Serial.println("Remote command: System Reset");
      
      // Turn OFF all pumps
      for (int i = 0; i < SENSOR_COUNT; i++) {
        digitalWrite(relayPins[i], HIGH);
        pumpStatus[i] = false;
        Firebase.setBool(fbdo, "/latest_readings/pump_" + String(i + 1), false);
      }
      
      // Reset the command
      Firebase.setBool(fbdo, "/commands/system_reset", false);
      
      Serial.println("System reset completed.");
    }
  }
}
