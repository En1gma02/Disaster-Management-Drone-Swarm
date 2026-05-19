# Comprehensive Design & Solution Documentation
**Competition:** NIDAR (National Innovation Challenge for Drone Application and Research)
**Mission Category:** Mission 1 - Disaster Management

---

## 1. Executive Summary & Mission Overview
The NIDAR Disaster Management mission simulates a flooded coastal town where severe weather has stranded residents on rooftops without food, water, or medicine. Since ground-based NDRF teams cannot access the area, an immediate aerial response is required. 

**Core Mission Objectives (per Rulebook):**
* **Surveillance:** Rapid scanning of a 30-hectare area to locate survivors.
* **Data Acquisition:** Transmit real-time video, geotag survivor coordinates, and broadcast instructions via an onboard loudspeaker.
* **Payload Delivery:** Execute precision air-drops of survival kits to identified locations.
* **Payload Specifications:** 200g per kit, with dimensions of $5 	imes 10 	imes 20$ cm.

To achieve maximum efficiency and target the maximum autonomous points allocation (600 points vs. 360 points for manual operation), We adopted a **Two-Drone Segregation Strategy**. This strategy separates the high-speed scanning requirements from the heavy-lift payload delivery requirements.

---

## 2. System Architecture & Approach

Our solution utilizes two distinct, specialized drones operating from a single, unified Ground Control Station (GCS) to comply with competition regulations and avoid manual interface penalties.

1.  **Drone 1: The Scout (Hybrid VTOL)** - Built for endurance, speed, and widespread aerial surveillance.
2.  **Drone 2: The Mule (Heavy-Lift Hexacopter)** - Built for stability, precision drops, and heavy payload capacity.

### Mission Flow (Autonomous Operation)
1.  **Initiation:** The mission is started from the unified Ground Control Station.
2.  **Surveillance:** The Scout (VTOL) begins a boustrophedon (lawnmower) survey path over the 30-hectare Area of Interest.
3.  **Data Transmission:** The Scout transmits live video and geotagged survivor coordinates back to the GCS.
4.  **Deployment:** The Mule (Hexacopter) deploys directly to the received coordinates.
5.  **Iteration:** The mission loops until all identified survivors are served.
6.  **Return:** Both drones automatically Return-to-Home (RTH) upon mission completion.

---

## 3. Drone 1: The Scout (Hybrid VTOL)

The Scout is a Hybrid Vertical Take-Off and Landing (VTOL) aircraft. It combines the hovering capability necessary for constrained takeoff/landing zones (6x6 ft launchpads) with fixed-wing aerodynamic efficiency for rapidly scanning the 30-hectare region.

### 3.1 Design & Aerodynamics
* **Airframe Material:** Carbon Fiber spars are used for structural rigidity, coupled with EPO/Composite foam for lightweight lifting surfaces.
* **Aerodynamics:** Features high-aspect-ratio wings to ensure low-drag cruising. Vertical lifting motors are disabled during forward flight to conserve energy.
* **Propulsion System:** 4x Vertical Motors for hover operations and 1x Rear Pusher Motor for cruise efficiency.
* **Safety & Fail-safes:** The drone is programmed to automatically transition back to multicopter (hover) mode if airspeed drops critically, acting as a stall prevention mechanism.

### 3.2 Subsystem Breakdown
| Sr. No | Component | Description |
| :--- | :--- | :--- |
| 1 | Flight Controller | Crossflight FC |
| 2 | Power Distribution | Standard Power Distribution Board (PDB) |
| 3 | Battery | Platinumpower 6200 mAh |
| 4 | Companion Computer | Raspberry Pi (handles computer vision / video processing) |
| 5 | Camera Module | R-Pi Cam |
| 6 | Comm Link | High-gain Wi-Fi adapter (5.8 GHz) + ELRS System |
| 7 | Positioning | GPS Module |
| 8 | Propulsion | 4x Vertical Motors, 1x Pusher Motor |
| 9 | Motor Control | 4-in-1 Electronic Speed Controller (Vertical), 1x ESC (Pusher) |
| 10 | Control Surfaces | 4x Servos |

---

## 4. Drone 2: The Mule (Hexacopter)

The Mule is a heavy-lift Hexacopter selected specifically for its stability and payload-carrying capabilities, maximizing payload drops per sortie to minimize round-trip times.

### 4.1 Airframe Justification & Capabilities
* **Airframe Type:** 6-Rotor Hexacopter.
* **Redundancy (Why not Quadcopter?):** Quadcopters have zero redundancy; a single motor failure results in an immediate crash. The hexacopter provides propulsion redundancy, allowing it to safely land even if one motor fails—a crucial safety feature for disaster zones.
* **Payload Capacity:** Designed to lift over 5kg (2kg Payload + dropping mechanism weight). It can efficiently carry up to 10 survival kits in a single sortie.
* **Precision Stability:** 6 points of thrust provide superior micro-adjustments for high-accuracy drops in "Zone A" (1.5m radius from the survivor).
* **Terrain Adaptability:** Wide landing gear stance enables landing on uneven, debris-covered terrain.

### 4.2 Payload Dropping Mechanism
* **Capacity:** 10 Survival Kits (200g each, $5 	imes 10 	imes 20$ cm).
* **Design:** Servo-actuated "Revolver dropper" / linear rack mechanism.
* **Drop Accuracy:** The mechanism is precisely calibrated to release payloads from a 20-foot altitude to hit the Rulebook's required Zone A accuracy radius for maximum points.

### 4.3 Subsystem Breakdown
| Sr. No | Component | Description |
| :--- | :--- | :--- |
| 1 | Flight Controller | Pixhawk V5+ |
| 2 | Power System | PDB + Power Module |
| 3 | Battery | 6S 16800 mAh High-Capacity Battery |
| 4 | Comm Link | Telemetry Module + ELRS |
| 5 | Positioning | High-precision GPS Module |
| 6 | Propulsion | 6x Motors |
| 7 | Motor Control | 6x Electronic Speed Controllers |

---

## 5. Software, Intelligence & Autonomy

To maximize our score under the NIDAR Autonomous scoring multiplier (x) vs Manual (0.6x), highly intelligent path planning and computer vision algorithms are integrated.

### 5.1 Intelligent Path Planning Algorithm
* **Grid Optimization:** Automatically rotates the map based on the Minimum Bounding Rectangle (MBR) and generates parallel flight lines based on the camera sensor footprint to ensure >90% coverage of the 30-hectare area.
* **Precision Clipping:** Eliminates unnecessary overshoot beyond the geofenced area, thereby reducing wasted battery consumption.
* **Efficiency Tuning:** Minimizes the number of turns required during the lawnmower survey.
* **Smart Stitching:** Utilizes a "Nearest Neighbor" strategy to connect disconnected flight segments efficiently.

### 5.2 Survivor Detection
* **Model:** Custom YOLO (You Only Look Once) Object Detection Model.
* **Function:** Trained specifically on aerial datasets to detect human volunteers (survivors) acting as targets in the field, generating real-time bounding boxes and outputting precise coordinate data.

---

## 6. Communication & Ground Control

### 6.1 In-House Developed Video Transmission Unit (OpenHD)
To maintain constant situational awareness and stream live video for survivor verification, an in-house digital video transmission unit was built using the open-source OpenHD protocol.
* **Why OpenHD?**
    * Long-range transmission (several kilometers).
    * Ultra-low latency (~50-120 ms).
    * HD video quality (720p / 1080p).
    * Cost-effective compared to commercial FPV systems.
* **Architecture:**
    * **Air Unit:** Raspberry Pi (SBC) + Camera Module + 5.8 GHz Wi-Fi adapter.
    * **Ground Unit:** Raspberry Pi / Laptop + Wi-Fi adapter + High-gain directional/omnidirectional antennas.

### 6.2 Ground Control Station (GCS)
* **Interface:** A unified multi-drone UI connected via UDP (127.0.0.1:14560).
* **Features:** Displays real-time battery voltage, satellite count, heading, speed, altitude, a live video feed window, and mission planner/waypoint mapping.
* **Rulebook Compliance:** Using a single GCS for both drones ensures compliance with the NIDAR rulebook, avoiding the severe -50 point penalty for using multiple remote controls or disjointed interfaces.

---

## 7. Rulebook Compliance & Fail-Safes

Our system architecture meticulously complies with NIDAR regulations:
* **Weight & Dimensions:** Drones are designed under the 25kg MTOW limit and fit within the mandatory 6x6 ft launchpad area.
* **Fail-Safes Built-In:**
    * **Loss of Link:** Configured to automatically Return-To-Home (RTH) upon loss of Command/Control or Data Link.
    * **Low Battery:** Dual-stage fail-safe (RTH on low warning; descend in place on critical low).
    * **Geofence:** Hardcoded boundary parameters prevent the drones from flying outside the 30-hectare Area of Interest or above 400 feet AGL.
* **Frequency:** Exclusively utilizes de-licensed frequency bands (5.8 GHz for video, standard 2.4 GHz for ELRS/Telemetry) to avoid unauthorized use of licensed spectrums.
