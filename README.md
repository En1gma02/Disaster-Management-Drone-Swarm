# Aerial Disaster Management System

An integrated autonomous drone architecture developed for the **National Innovation Challenge for Drone Application and Research (NIDAR)**. This system is designed specifically for Mission 1: Disaster Management, providing rapid aerial reconnaissance and precision payload delivery to stranded survivors in flooded coastal regions.

## Mission Strategy: Two-Drone Segregation

To maximize autonomous efficiency and payload capacity, the system employs a two-tier aerial architecture operating from a unified Ground Control Station (GCS).

### 1. The Scout (Hybrid VTOL)
Designed for rapid, high-endurance surveillance over the 30-hectare Area of Interest.
- **Flight Controller:** Crossflight FC
- **Capabilities:** Vertical takeoff/landing, efficient fixed-wing forward flight.
- **Role:** Executes an optimized lawnmower path, transmits real-time video via an OpenHD 5.8 GHz link, runs the custom YOLOv8 human detection model, and logs precise, latency-compensated GPS coordinates of survivors.

### 2. The Mule (Heavy-Lift Hexacopter)
Selected for its 6-motor propulsion redundancy and superior hover stability.
- **Flight Controller:** Pixhawk V5+
- **Capabilities:** 5kg+ lift capacity, redundant fail-safes.
- **Role:** Deploys to coordinates acquired by the Scout, utilizes visual servoing for micro-adjustments, and actuates a 10-slot "Revolver dropper" servo mechanism to precisely deliver 200g survival kits within a 1.5m radius.

## System Architecture

The codebase is built on a modular, object-oriented framework utilizing `PyMAVLink` for robust communication with both the Crossflight and Pixhawk flight controllers.

### Intelligent Path Planning
The Scout utilizes a Minimum Bounding Rectangle (MBR) algorithm to automatically rotate the mission grid and generate parallel flight lines based on the camera sensor footprint. This ensures >90% coverage while eliminating unnecessary overshoot beyond the geofenced area, significantly reducing battery consumption.

### Latency-Compensated Geolocation
Standard pixel-to-GPS transformation suffers from errors due to inference delays. Our custom `Geolocator` engine utilizes a backtrack algorithm that interpolates vehicle telemetry logs (attitude and altitude) to estimate the drone's precise pose at the exact moment of image capture, ensuring pinpoint target accuracy.

### Unified Ground Control & Swarm Orchestration
To comply with NIDAR regulations and avoid multi-controller penalties, the system features a unified orchestration layer. A custom UDP hub synchronizes telemetry and mission commands across both the VTOL and Hexacopter platforms simultaneously.

## Repository Structure

*   **src/**: Core source code.
    *   **core/**: Object-oriented implementations for PyMAVLink hardware interfacing, MBR path planning, and latency-compensated geolocation.
    *   **missions/**: High-level autonomous execution scripts (`scout_vt_mission.py`, `mule_hex_mission.py`).
    *   **experimental/**: Unified GCS synchronization and advanced visual alignment routines.
*   **models/**: Custom YOLOv8 weights trained specifically for aerial human detection.
*   **config/**: Mission parameters and KML geofence definitions.
*   **docs/**: Technical reference material and the complete Solution Design Documentation.
*   **utilities/**: Diagnostic tools for hardware verification (servos, buzzers) and SITL orchestration.

## Setup and Deployment

### Dependencies
The system requires Python 3.8+ and the following core libraries:
*   `pymavlink`: For robust flight controller communication.
*   `opencv-python` & `ultralytics`: For the computer vision pipeline.
*   `shapely` & `geopy`: For geographic computations and geofence parsing.

### Execution
Mission scripts are designed to accept standard MAVLink connection strings (e.g., `udp:127.0.0.1:14560` for the unified GCS or `serial:///dev/ttyUSB0:57600` for physical telemetry radios), allowing identical code to execute in both SITL simulation and live deployment.

## License
This project is proprietary and was developed for the NIDAR competition.
