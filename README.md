# 🌍 Earth Rover Design

**Earth Rover Design** is a ground rover project focused on remote exploration, environmental monitoring, and teleoperation. The system integrates multiple USB cameras, environmental and distance sensors, DC motors with closed-loop control, and a distributed **client–server architecture** based on **Raspberry Pi and Arduino**.

The project was designed with modularity in mind, enabling future extensions such as autonomous navigation, SLAM, and advanced computer vision.

---

## Rover Overview

![1768196071728](https://github.com/user-attachments/assets/a0d199e3-364a-44f3-bf2a-b0ff918f46d1)

---

## System Architecture

The rover follows a distributed architecture:

* **Server (Onboard Rover)**

  * Raspberry Pi 4 (2 GB RAM)
  * Arduino Nano
  * Sensors, actuators, and cameras

* **Client (Laptop)**

  * Graphical User Interface (GUI)
  * Real-time video streaming
  * Remote motion and sensor control

### Data Flow

1. The **Raspberry Pi** acts as the main server:

   * Captures video streams from USB cameras
   * Handles network communication with the client
   * Sends control commands to the Arduino
2. The **Arduino Nano**:

   * Controls DC motors and servos
   * Reads sensor data (ToF, IMU, environmental sensors)
3. The **Client GUI**:

   * Displays live camera feeds
   * Sends movement and control commands

---

## Vision System

* **Front USB Camera**

  * Resolution: 1280×720
* **Top USB Camera**

  * Resolution: 720p
  * Field of View: 120°

Both cameras use the **UVC standard**, allowing seamless integration with Linux-based systems and OpenCV pipelines.

---

## Actuation System

* **4× DC Gear Motors**

  * Model: 25GA-370
  * Voltage: 12 V
  * Speed: 200 RPM
* **2× DRV8871 DC Motor Drivers**

  * H-Bridge configuration
  * PWM control
  * Up to 3.6 A
* **Servo Motors**

  * High-torque metal gear servo (180°)
  * SG90 micro servos for auxiliary mechanisms

---

## Sensors

* **Distance Measurement**

  * 2× VL53L0X Time-of-Flight (I2C)
* **Inertial Measurement Unit**

  * MPU-6050 (3-axis accelerometer + gyroscope)
* **Environmental Sensors**

  * DHT11 (temperature and humidity)
  * Capacitive soil moisture sensors (×3)

---

## Power System

* **12 V 5200 mAh Li-ion Battery**
* **LM2596 DC–DC Buck Converters**

  * Voltage regulation for:

    * Raspberry Pi
    * Arduino
    * Sensors
    * Servos

---

## Mechanical Design

* 3D-printed structure (**PLA**)
* **M2 and M3** screws
* 6803-2RS sealed ball bearings
* Aluminum RC chassis and suspension upgrade kit (WPL compatible)

---

## Technologies & Tools

* Raspberry Pi OS / Linux
* Arduino
* Serial communication
* USB Video (UVC)
* 3D printing
* Modular hardware design

---

## Future Improvements

* Autonomous navigation
* SLAM (visual or LiDAR-based)
* ROS integration
* Advanced computer vision with OpenCV
* Reinforcement learning for motion control
* Extended telemetry and diagnostics

---

## Suggested Repository Structure

```bash
earth-rover-design/
├── firmware/
│   └── arduino/
├── server/
│   └── raspberry_pi/
├── client/
│   └── gui/
├── cad/
│   └── stl/
├── media/
│   └── images_videos/
└── README.md
```

---

## Project Status

- Functional prototype
- Ongoing development and improvements

---

## License

This project is released under the MIT License.

---

**Regards,**
*Earth Rover Design*
