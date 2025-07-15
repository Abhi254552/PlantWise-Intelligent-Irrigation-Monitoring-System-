# PlantWise: Intelligent Hydration & Environmental Management System

A robust, end-to-end embedded solution for smart plant irrigation and environmental monitoring. PlantWise combines STM32 microcontroller firmware, a Raspberry Pi MQTT broker, and a feature-rich Android application to deliver real-time data, automated watering, and user-driven control for optimal plant care.

## Table of Contents

- [Project Overview](#project-overview)
- [Why PlantWise?](#why-plantwise)
- [System Architecture](#system-architecture)
- [Key Features](#key-features)
- [ESP-01 Firmware Issue & Solution](#esp-01-firmware-issue--solution)
- [How It Works](#how-it-works)
- [Setup & Deployment](#setup--deployment)
- [Raspberry Pi Broker Commands](#raspberry-pi-broker-commands)
- [Component Summary](#component-summary)
- [Demo Video](#demo-video)

## Project Overview

PlantWise is a smart watering and environmental monitoring system designed to automate plant irrigation and improve crop health. It continuously tracks key environmental parameters—soil moisture, ambient light, air temperature, and humidity—and automatically waters plants when needed. A dedicated Android application provides real-time remote monitoring and control, enabling users to view sensor readings and switch between automatic and manual irrigation modes. By integrating sensors, actuators, and network connectivity, PlantWise optimizes water usage and ensures plant wellbeing without constant human intervention.

## Why PlantWise?

- **Water Conservation:** Automates irrigation based on real-time soil moisture, reducing water waste.
- **Plant Health:** Maintains optimal growing conditions by monitoring and responding to environmental changes.
- **Remote Management:** Allows users to monitor and control irrigation from anywhere via the Android app.
- **Reliability:** Ensures continued operation and data logging even during network outages.
- **Educational Value:** Demonstrates advanced embedded system design, sensor integration, actuator control, and IoT communication.

## System Architecture

**1. STM32F407 Microcontroller (Embedded Node)**
- Reads soil moisture, light, temperature, and humidity sensors.
- Controls water pump (via relay) and servo for irrigation.
- Communicates with Raspberry Pi over Wi-Fi (ESP-01) using MQTT.
- **Important:** See [ESP-01 Firmware Issue & Solution](#esp-01-firmware-issue--solution) for critical setup notes.

**2. Raspberry Pi (MQTT Broker & Automation Engine)**
- Runs Mosquitto MQTT broker for all inter-device messaging.
- Logs sensor data and system events.
- Hosts Python scripts for automation and camera integration.

**3. Android Application**
- Subscribes to live sensor data and system notifications.
- Provides dashboard for real-time monitoring.
- Allows mode switching (Auto/Manual), manual pump control, and crop selection.
- Receives plant images and alerts.

**4. Communication**
- All messaging uses MQTT with topic-based publish/subscribe.
- Critical commands use QoS 1 for reliability.
- Security via broker configuration and topic ACLs.

## Key Features

- **Automatic & Manual Irrigation:** Switch between modes; pump activates automatically when soil moisture drops below crop-specific thresholds.
- **Real-Time Monitoring:** Live dashboard for soil moisture, light, temperature, and humidity.
- **Remote Control:** Start/stop watering, change modes, and update thresholds from the app.
- **Crop-Specific Thresholds:** Select plant type to auto-load optimal moisture settings.
- **Event & Alert System:** Notifies users of watering events, low-light, or out-of-range conditions.
- **Image Capture:** Raspberry Pi camera captures and sends plant images on watering events.
- **Robustness:** STM32 continues auto-watering with last known threshold if connectivity is lost; data syncs on reconnection.
- **Data Logging:** Raspberry Pi logs all sensor readings and events for history and analysis.

## ESP-01 Firmware Issue & Solution

**Firmware Issue:**
When using the ESP-01 Wi-Fi module for MQTT communication, we encountered a major roadblock: the default firmware (usually v1.5 or older) does **not** support MQTT AT commands. Only firmware versions v2.0+ provide the necessary MQTT AT command support. This issue can prevent the STM32 from communicating with the MQTT broker, halting the entire system.

**How to Fix:**
- **Firmware Update Required:** You must update your ESP-01 to a firmware version that supports MQTT AT commands (v2.0+).
- **Finding the Right Firmware:** The challenge is not the update process itself (many tutorials and videos are available), but finding the correct firmware for your ESP-01’s flash size (1MB or 4MB). Using the wrong firmware can brick your device.
- **Firmware Repository:** After extensive searching, we found a reliable source for ESP8266/ESP-01 firmwares:
  - [ESP8266 Open Source Toolchain & Firmware Repository (GitHub)](https://github.com/cjacker/opensource-toolchain-esp8266)
  - Download the firmware matching your device’s flash size.
- **Update Process:** 
  - Watch YouTube tutorials for step-by-step instructions on updating ESP-01 firmware.
  - The update can be performed using a Python script (esptool.py).
  - Pay special attention to which GPIO pins to connect/disconnect during flashing—refer to video guides for your specific setup.
- **Summary:** Most ESP-01 modules ship with v1.5 or older firmware. You must upgrade to v2.0+ for MQTT AT command support. The update is straightforward if you have the correct firmware and follow a reliable tutorial.

## How It Works

1. **Sensor Acquisition:** STM32 polls soil moisture, light, and DHT11 sensors every 500 ms for UI updates and every hour for irrigation decisions.
2. **Irrigation Logic:** In Auto mode, the system compares soil moisture to the selected crop’s threshold and activates the pump and servo as needed. In Manual mode, users can start/stop watering via the app.
3. **Data Publishing:** Sensor readings and system events are published as JSON over MQTT to the Raspberry Pi, which relays them to the Android app.
4. **User Interaction:** The Android app displays live data, allows mode switching, and provides manual controls. Users can select crops, set thresholds, and receive alerts.
5. **Image Notification:** When watering starts, the Pi triggers the camera, encodes the image, and publishes it to the app.
6. **Reliability:** If the network is lost, STM32 continues with the last configuration; on reconnection, all data and alerts are synchronized.

## Setup & Deployment

### Prerequisites

- **STM32F407 Discovery Board** (or compatible)
- **ESP-01 Wi-Fi Module** (UART interface, with updated firmware—see above)
- **Raspberry Pi 3** (or newer) with Mosquitto MQTT broker
- **Android Smartphone** (Android 9+)
- **Sensor Modules:** Soil moisture, LDR (light), DHT11 (temperature/humidity)
- **Actuators:** 5V relay, DC water pump, servo motor

### Steps

1. **STM32 Firmware**
   - Build and flash the firmware using the provided source files (`main.c`, `sensor.c`, `mqtt.c`, etc.).
   - Configure UART for ESP-01 and connect sensors/actuators as per documentation.

2. **Raspberry Pi Setup**
   - Install Mosquitto MQTT broker.
   - Replace the default `mosquitto.conf` with the provided configuration for LAN access and topic security.
   - Run the `mqtt_cam.py` script for camera integration.
   - Use `Pi_terminal_commands.txt` for broker and camera commands.

3. **Android App**
   - Install the PlantWise app (source not included here).
   - Connect to the same Wi-Fi network as the broker.
   - Configure MQTT broker address in the app settings if required.

4. **System Integration**
   - Power up STM32 and Raspberry Pi.
   - Launch the Android app and verify live data and controls.
   - Test automatic and manual irrigation, crop selection, and image notifications.

## Raspberry Pi Broker Commands

- **Start Mosquitto with custom config:**
  ```
  sudo systemctl stop mosquitto
  mosquitto -c /etc/mosquitto/mosquitto.conf -v
  ```
- **Run camera bridge:**
  ```
  cd /home/pi/plantwise_cam
  ./mqtt_cam.py
  ```
- **Update broker config:**
  ```
  sudo cp /home/pi/mosquitto.conf /etc/mosquitto/mosquitto.conf
  sudo systemctl restart mosquitto
  ```
- **Enable/disable Mosquitto on boot:**
  ```
  sudo systemctl enable mosquitto
  sudo systemctl disable mosquitto
  ```

## Component Summary

| Component         | Description                                                      |
|-------------------|------------------------------------------------------------------|
| STM32F407         | Embedded node for sensor reading and actuator control            |
| ESP-01            | Wi-Fi module for MQTT communication (requires v2.0+ firmware)    |
| Raspberry Pi 3    | MQTT broker, automation engine, and camera integration           |
| Android App       | Real-time dashboard, control interface, and alert system         |
| Sensors           | Soil moisture, LDR (light), DHT11 (temperature/humidity)         |
| Actuators         | 5V relay (pump), servo motor (nozzle sweep)                      |
| Camera            | Captures plant images on watering events (optional)              |

## Demo Video

A full demonstration of the PlantWise system—including STM32, ESP-01, Raspberry Pi, and Android app integration—is available here:

**[Project Demo Video](https://drive.google.com/file/d/1XK0QXm43m6MEAoeMoAGipwbf2nM6J-KE/view?usp=sharing)**
