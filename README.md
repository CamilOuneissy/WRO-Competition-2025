# WRO Future Engineers Robot 2025

Self-driving car robot for WRO Future Engineers competition, featuring autonomous navigation with computer vision and sensor fusion.

## Team Information
- **Team Name:** Mindscape - 
- **School/Organization:** Mindscape Accademy
- **Country:** Lebanon
- **Season:** 2025

## Robot Overview

This robot is designed for the WRO Future Engineers Self-Driving Cars challenge. It autonomously navigates a track for 3 laps, handling both open challenge (no obstacles) and obstacle challenge (with red/green traffic signs and parking).

### Key Features
- **Autonomous Navigation:** Wall following with ultrasonic sensors
- **Computer Vision:** Traffic sign detection using OpenCV
- **4-Wheel Car Design:** Front-wheel drive with servo steering
- **Real-time Control:** Sensor fusion for stable navigation

## Hardware Components

### Main Controllers
- **Single Board Computer (SBC):** Raspberry Pi 5 for vision processing
- **Motor Controller:** PWM-based DC motor and servo control
- **GPIO Interface:** Pin management with safety features

### Sensors
- **Camera:** USB webcam for traffic sign detection and cornering lines detection
- **Ultrasonic Sensors:** 2x HC-SR04 for distance measurement

### Actuators
- **Drive Motor:** DC motor with speed control
- **Steering:** Servo motor for front wheel steering
- **Power System:** 14.8V Lithium Battery pack with StepDown converter to 5V

## Software Architecture

### Core Modules
1. **Robot Controller** (`navigation/robot_controller.py`) - Main state machine
2. **Motor Controller** (`hardware/motor_controller.py`) - Vehicle movement
3. **Color Detector** (`vision/color_detector.py`) - Traffic sign recognition
4. **Sensor Management** (`hardware/ultrasonic_sensor.py`) - Distance sensing
5. **Configuration** (`config/robot_config.py`) - Settings management

### Competition Challenges

#### Open Challenge
- Navigate 3 laps without obstacles
- Wall following using ultrasonic sensors
- Variable track width handling

#### Obstacle Challenge  
- Navigate 3 laps with traffic signs
- **Red signs:** Pass on the right side
- **Green signs:** Pass on the left side
- **Parking:** Parallel parking after 3 laps

## Installation & Setup

### Dependencies
```bash
pip install opencv-python pillow numpy
```

### Configuration
1. Edit `robot_config.json` for your hardware setup
2. Calibrate colors using the GUI: `python3 launch_gui.py`
3. Test systems: `python3 main.py --test`

### Running the Robot

#### Interactive Mode
```bash
python3 main.py --mode interactive
```

#### Autonomous Competition
```bash
# Open Challenge
python3 main.py --mode auto --challenge open --direction cw

# Obstacle Challenge  
python3 main.py --mode auto --challenge obstacle --direction ccw
```

## Color Calibration

Traffic sign detection requires proper HSV color calibration:

1. **Launch GUI:** `python3 launch_gui.py`
2. **Navigate to Color Calibration tab**
3. **Start calibration camera**
4. **Click on traffic signs** to sample colors
5. **Adjust HSV sliders** for optimal detection
6. **Save calibration** to `color_calibration.json`

### Pre-configured Color Ranges
- **Red Signs:** H: 0-10, S: 100-255, V: 100-255
- **Green Signs:** H: 35-85, S: 100-255, V: 100-255
- **Blue Boundaries:** H: 100-130, S: 100-255, V: 100-255

## Project Structure

```
├── main.py                 # Main entry point
├── launch_gui.py           # GUI launcher
├── robot_config.json       # Hardware configuration
├── config/                 # Configuration management
├── hardware/               # Motor and sensor control
├── navigation/             # Robot control logic
├── vision/                 # Computer vision processing
└── color_calibration.json  # Color detection settings
```

## Algorithm Description

### Navigation Strategy
1. **Sensor Fusion:** Combine ultrasonic sensor data for wall following
2. **Wall Following:** Proportional control to maintain optimal distance
3. **Traffic Sign Detection:** HSV color space filtering with contour analysis
4. **Obstacle Avoidance:** Dynamic steering adjustment based on sign color
5. **Lap Counting:** Time-based estimation with section tracking

### Control Flow
```
Initialize → Camera + Sensors → Challenge Selection →
Wall Following → Traffic Sign Detection → Steering Control →
Lap Completion Check → Parking (Obstacle Challenge) → Stop
```

## Safety Features

- **Emergency Stop:** Immediate motor shutdown capability
- **Sensor Validation:** Invalid reading filtering and error handling  
- **GPIO Cleanup:** Proper resource management on shutdown
- **Mock Mode:** Development without hardware dependencies

## Competition Compliance

This robot meets WRO Future Engineers 2025 requirements:
- ✅ 4-wheeled vehicle with car-like steering
- ✅ Single drive motor + single steering actuator
- ✅ Autonomous operation (no remote control)
- ✅ Dimensions: ≤ 300×200×300mm, ≤ 1.5kg max
- ✅ Engineering documentation and GitHub repository

## Usage Notes

### Before Competition
1. Test all systems: `python3 main.py --test`
2. Verify color calibration under competition lighting
3. Check vehicle dimensions and weight limits
4. Ensure GitHub repository has 3+ commits over 2 months

### During Competition
1. Place vehicle in starting zone (powered off)
2. Power on with two-switch system
3. Press start button when signaled by judge
4. Robot operates autonomously.

## Troubleshooting

**Camera Issues:** Check camera index in config, test with GUI
**Motor Problems:** Verify GPIO connections and power supply  
**Color Detection:** Recalibrate colors for competition lighting
**Sensor Errors:** Check wiring and distance sensor positioning

---

*Designed for WRO Future Engineers 2025 - Self-Driving Cars Challenge*
