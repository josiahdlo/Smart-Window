# Smart-Window
Designed and built an autonomous smart window integrating motorized actuation, locking, and PDLC tint control. Developed custom ESP32 control circuitry and Python firmware enabling Wi-Fi communication, distributed sensor data sharing, and closed-loop control. Modeled housings in SolidWorks and 3D printed enclosures for all electronics.

# Smart Window — Autonomous IoT Window System

An autonomous smart window system integrating motorized actuation, electronic locking, and PDLC (polymer-dispersed liquid crystal) tint control. Developed as part of ME100 at UC Berkeley.

## Overview

This project demonstrates end-to-end integration of mechanical design, electronics, and embedded software in a single functional prototype. The system autonomously responds to environmental inputs—adjusting window position, lock state, and tint level—without user intervention.

## Features

- **Motorized Actuation** — Automated window opening/closing based on environmental conditions
- **Electronic Locking** — Secure locking mechanism with remote control capability
- **PDLC Tint Control** — Dynamic privacy glass that adjusts transparency
- **Environmental Sensing** — Temperature, smoke, and noise detection for automated responses
- **Distributed Control** — Three networked ESP32 microcontrollers communicating over Wi-Fi

## System Architecture

```
┌─────────────────┐     Wi-Fi      ┌─────────────────┐     Wi-Fi      ┌─────────────────┐
│   ESP32 #1      │◄──────────────►│   ESP32 #2      │◄──────────────►│   ESP32 #3      │
│   Motor Control │                │   Sensor Hub    │                │   Tint/Lock     │
└─────────────────┘                └─────────────────┘                └─────────────────┘
        │                                  │                                  │
        ▼                                  ▼                                  ▼
   Window Motor                    Temp/Smoke/Noise                   PDLC + Lock
                                      Sensors                         Actuators
```

## Hardware

- **Microcontrollers:** 3x ESP32 development boards
- **Sensors:** Temperature sensor, smoke detector, noise/sound sensor
- **Actuators:** DC motor (window), solenoid lock, PDLC film driver
- **Enclosures:** Custom 3D-printed housings designed in SolidWorks

## Software

All firmware is written in Python (MicroPython) and includes:

- `main.py` — Primary control loop and system initialization
- `wifi_comm.py` — Wi-Fi communication and data sharing between microcontrollers
- `sensors.py` — Sensor reading and data processing
- `actuators.py` — Motor, lock, and PDLC control functions
- `control_logic.py` — Closed-loop decision logic for autonomous operation

## Control Logic

The system uses closed-loop control to determine actuator states based on sensor inputs:

| Condition | Window | Lock | Tint |
|-----------|--------|------|------|
| High temperature | Open | Unlock | Clear |
| Smoke detected | Closed | Unlock | Clear |
| High noise level | Closed | — | — |
| Night / Privacy mode | — | Lock | Opaque |

## Mechanical Design

Enclosures for all electronic components were:
- Modeled in **SolidWorks** with attention to fit, mounting, and wiring clearance
- Fabricated via **3D printing** (FDM)
- Designed for easy assembly and access to components

## Repository Structure

```
smart-window/
├── firmware/
│   ├── main.py
│   ├── wifi_comm.py
│   ├── sensors.py
│   ├── actuators.py
│   └── control_logic.py
├── cad/
│   └── enclosures/
├── docs/
│   └── wiring_diagram.pdf
└── README.md
```

## Skills Demonstrated

- SolidWorks — CAD modeling for enclosures
- Python/MicroPython — Embedded firmware development
- ESP32 — Microcontroller programming and Wi-Fi networking
- Circuit Design — Power distribution and signal routing
- 3D Printing — Rapid prototyping and fabrication
- IoT Systems — Distributed sensing and control

## Authors

**Josiah Lo**  
Mechanical Engineering, UC Berkeley  
[LinkedIn](https://linkedin.com/in/josiahdavidlo) · josiahlo@berkeley.edu

**Gary Uppal**  
UC Berkeley

**Kyle Bazzone**  
UC Berkeley

*Developed as part of ME100 at UC Berkeley*

## License

This project is for educational purposes. Feel free to reference for learning.
