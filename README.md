# BhoomiMitra 🌾🇮🇳

A smart agriculture solution combining IoT, robotics, AI, and drone monitoring for high-efficiency, scalable, and sustainable farming practices.

## 🔍 Overview

BhoomiMitra is a comprehensive smart agriculture system that integrates IoT-based soil moisture sensing, autonomous irrigation, swarm robotics for seeding verification, and aerial monitoring using drones. The name "BhoomiMitra" translates to "Friend of the Earth" in Sanskrit, reflecting the project's goal of promoting sustainable and efficient agricultural practices.

## 🌟 Key Features

- **Automated Irrigation System**: Soil moisture-based precision irrigation
- **Swarm Robotics**: Autonomous seeding, verification, and correction robots
- **Aerial Monitoring**: Drone-based thermal imaging for crop health analysis
- **Real-Time Dashboard**: Cloud-based monitoring and control via Firebase/ThingSpeak
- **Sustainable Farming**: Optimized water usage and resource management

## 🧰 Hardware Components

| Component                              | Quantity | Description                                            |
| -------------------------------------- | -------- | ------------------------------------------------------ |
| ESP32 Microcontroller                  | 1        | Main controller for data aggregation and control       |
| Capacitive Soil Moisture Sensor        | 9        | One for each plant zone to detect soil moisture        |
| 5V Submersible Mini Water Pumps        | 9        | Each pump irrigates one plant via relays               |
| Relay Module (1/4/8 Channel)           | 1        | Controls water pumps based on sensor data              |
| Power Supply (5V/12V)                  | 1        | External power for pumps and ESP32                     |
| Swarm Robots                           | 3        | Seed Planter (Leader), Verifier, and Fixer             |
| L298N Motor Driver                     | 3        | Controls robot motors                                  |
| Ultrasonic Sensors                     | 6        | For obstacle detection (2 per robot)                   |
| Cameras (Raspberry Pi Cam/USB Webcams) | 3        | For robot vision (optional for detection/verification) |
| GPS Modules (optional)                 | 1–3      | For robot and field tracking                           |
| Drone with Thermal Camera              | 1        | Aerial surveillance, crop stress analysis              |
| ArUco Marker Sheets                    | Several  | For robot navigation and ID tracking                   |
| NodeMCU or Secondary ESP32 (optional)  | 1–2      | For drone or secondary robot control                   |
| Jumper Wires, Breadboards, PCBs        | -        | Wiring and prototyping                                 |
| Water Tank & Pipes                     | -        | Water storage and distribution setup                   |

## 🧠 System Architecture

### 1. Soil Moisture Monitoring & Irrigation Control

- Each capacitive sensor is connected to an analog pin of the ESP32
- The ESP32 processes moisture data and checks it against a threshold (e.g., 30%)
- If below threshold, ESP32 activates the relay, turning on the pump
- Pump irrigates the plant for a defined time/duration
- All sensor readings and actions are logged to Firebase/ThingSpeak for visualization

### 2. Swarm Robots Functionality

- **Leader Bot**: Plants the seeds at pre-defined GPS/mapped points
- **Verifier Bot**: Follows and scans seed placements using camera and CV
- **Fixer Bot**: Re-plants or corrects errors detected by Verifier
- Communication between robots using WiFi mesh or ESP-NOW protocol

### 3. Aerial Drone Surveillance

- Drone flies over field collecting thermal and visual data
- Data processed via AI to detect moisture-stressed zones and diseases
- Positions relayed to the central server for attention

## 🚀 Getting Started

### Prerequisites

- Arduino IDE with ESP32 board support
- Python 3.7+ for robot control scripts
- Firebase account (for cloud integration)
- Basic electronics tools and components

### Installation

1. Clone this repository:
   ```
   git clone https://github.com/yourusername/BhoomiMitra.git
   ```

2. Install required libraries for Arduino:
   - WiFi.h
   - FirebaseESP32.h
   - ESP32Servo.h

3. Install Python dependencies:
   ```
   pip install -r requirements.txt
   ```

4. Configure your Firebase credentials in the appropriate files

5. Upload the ESP32 code using Arduino IDE

6. Set up the hardware according to the schematics in the `hardware_schematics` folder

## 📁 Repository Structure

```
BhoomiMitra/
├── README.md
├── hardware_schematics/
│   └── circuit_diagrams.png
├── code/
│   ├── esp32_irrigation/
│   │   └── esp32_irrigation.ino
│   ├── robot_control/
│   │   ├── leader_bot.py
│   │   ├── verifier_bot.py
│   │   └── fixer_bot.py
│   └── cloud_integration/
│       ├── firebase_connect.ino
│       └── thingspeak_connect.ino
├── documentation/
│   └── BhoomiMitra_Technical_Report.pdf
├── media/
│   └── project_images/
└── LICENSE
```

## 📊 Data Flow

1. Soil moisture sensors collect data
2. ESP32 processes data and controls irrigation
3. Data is sent to cloud platform
4. Swarm robots receive commands based on data analysis
5. Drone provides aerial feedback for system optimization

## 🔧 Future Enhancements

- Machine learning for predictive irrigation
- Mobile app for remote monitoring and control
- Integration with weather APIs for smarter decision-making
- Expanded sensor array for comprehensive soil analysis
- Solar power integration for energy sustainability

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👤 Author

**Soumyajyoti Banik**

## 🙏 Acknowledgments

- Thanks to all contributors and testers
- Special thanks to the open-source community for providing libraries and tools
- Inspired by sustainable farming initiatives worldwide
