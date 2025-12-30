# Smart Window — Autonomous IoT Window System

An autonomous smart window system integrating motorized actuation, electronic locking, and PDLC (polymer-dispersed liquid crystal) tint control. Developed as part of ME100 at UC Berkeley.

## Overview

This project demonstrates end-to-end integration of mechanical design, electronics, and embedded software in a single functional prototype. The system autonomously responds to environmental inputs—adjusting window position, lock state, and tint level—without user intervention.

## Features

- **Motorized Window Actuation** — Stepper motor-driven window with limit switches for open/closed positions
- **Servo Lock Mechanism** — Secure locking with manual override and safety interlocks
- **PDLC Tint Control** — Relay-controlled privacy glass that toggles between clear and opaque
- **Environmental Sensing** — Indoor temperature (DHT11), outdoor temperature, smoke detection (PMS5003 PM2.5), and noise monitoring
- **Distributed Control** — Three networked ESP32 microcontrollers communicating over ESP-NOW protocol
- **Remote Control** — Joystick input with OLED display for status and temperature setpoint adjustment

## System Architecture

```
┌────────────────────────┐                    ┌────────────────────────┐
│     REMOTE ESP32       │                    │    OUTDOOR ESP32       │
│  (Adafruit Feather V2) │                    │  (Environmental Node)  │
│                        │                    │                        │
│  • Joystick X/Y input  │                    │  • DHT11 temperature   │
│  • Button (mode/lock)  │                    │  • Noise sensor (DO)   │
│  • 128x64 OLED display │                    │                        │
│  • Status LEDs         │                    │                        │
│  • Temp setpoint adj.  │                    │                        │
└───────────┬────────────┘                    └───────────┬────────────┘
            │ ESP-NOW                                     │ ESP-NOW
            │ (joystick, button,                          │ (outdoor temp,
            │  temp setpoint)                             │  noise state)
            ▼                                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         BASE ESP32                                   │
│                    (Window Controller)                               │
│                                                                      │
│  Sensors:                        Actuators:                          │
│  • DHT11 (indoor temp)           • Stepper motor (window position)   │
│  • PMS5003 (PM2.5 smoke)         • Servo (lock mechanism)            │
│  • Limit switches (open/closed)  • Relay (PDLC tint control)         │
│                                  • Piezo buzzer (alerts)             │
│                                                                      │
│  Control Logic:                                                      │
│  • AUTO mode: temp-based open/close with noise override              │
│  • MANUAL mode: joystick control of window position                  │
│  • Smoke override: forces window open on high PM2.5                  │
│  • Safety interlocks: prevents movement while locked                 │
└─────────────────────────────────────────────────────────────────────┘
```

## Control Logic

### AUTO Mode Temperature Control
| Indoor Temp | Outdoor Temp | Action |
|-------------|--------------|--------|
| Too hot (> setpoint + 2°F) | Cooler than indoor | Open window |
| Too cold (< setpoint - 2°F) | Warmer than indoor | Open window |
| Too hot | Hotter than indoor | Close window |
| Too cold | Colder than indoor | Close window |
| In comfort range | — | Maintain position |

### Override Conditions
| Condition | Action | Priority |
|-----------|--------|----------|
| Smoke detected (PM2.5 > 80 µg/m³) | Force open, unlock, alarm | Highest |
| Outdoor noise detected | Close window | High |
| Manual joystick input | Direct control | Overrides AUTO |

### Lock Behavior
- Auto-locks when window reaches closed position (with grace period)
- Unlocks automatically before any open movement
- Manual lock/unlock via joystick button
- Safety interlock prevents movement while locked

## Hardware

### Base ESP32 (Window Controller)
- **Microcontroller:** ESP32
- **Motor:** Stepper motor with A4988/DRV8825 driver
- **Lock:** Servo motor (0° unlock, 90° lock)
- **Sensors:** DHT11 (temperature), PMS5003 (PM2.5 particulate)
- **Switches:** 2x limit switches (open/closed positions)
- **Output:** Relay (PDLC control), piezo buzzer (alerts)

### Remote ESP32 (User Interface)
- **Microcontroller:** Adafruit ESP32 Feather V2
- **Input:** 2-axis analog joystick with button
- **Display:** 1.3" 128x64 OLED (SSD1306, I2C)
- **Indicators:** Status LEDs (auto mode, lock state)

### Outdoor ESP32 (Environmental Sensor)
- **Microcontroller:** ESP32
- **Sensors:** DHT11 (outdoor temperature), digital noise sensor

### Enclosures
- Custom housings designed in **SolidWorks**
- Fabricated via **3D printing** (FDM)
- Designed for fit, mounting, and wiring clearance

## Communication Protocol

All three ESP32s communicate using **ESP-NOW** (low-latency, connectionless protocol).

### Remote → Base Packet (7 bytes)
```
struct { uint16_t joy_x, joy_y; uint8_t button; int16_t temp_setpoint_x10; }
```

### Outdoor → Base Packet (3 bytes)
```
struct { int16_t temp_f_x10; uint8_t noise_state; }
```

### Base → Remote Packet (7 bytes)
```
struct { uint8_t status_flags; int16_t indoor_temp_x10; int16_t outdoor_temp_x10; uint8_t smoke; uint8_t noise; }
```

## Repository Structure

```
smart-window/
├── firmware/
│   ├── base_esp32.py          # Main controller - window, lock, sensors, control logic
│   ├── remote_esp32.py        # User interface - joystick, OLED, status LEDs
│   └── outdoor_esp32.py       # Environmental node - outdoor temp, noise detection
├── cad/
│   └── enclosures/            # SolidWorks files and STLs
├── docs/
│   └── wiring_diagram.pdf
└── README.md
```

## Key Software Features

- **Watchdog timer** (8s) on all microcontrollers for reliability
- **Retry logic** for ESP-NOW communication failures
- **Filtered sensor readings** (noise detection uses consecutive-reading filter)
- **Debounced inputs** (button hold detection, joystick deadzone)
- **Grace periods** (servo lock timing to prevent false triggers)
- **I2C error recovery** (OLED auto-reinitialize on failure)

## Skills Demonstrated

- **SolidWorks** — CAD modeling for enclosures
- **Python/MicroPython** — Embedded firmware development
- **ESP32** — Microcontroller programming, ESP-NOW networking
- **Circuit Design** — Power distribution, signal routing, motor drivers
- **3D Printing** — Rapid prototyping and fabrication
- **Control Systems** — Closed-loop temperature control with hysteresis
- **Sensor Integration** — DHT11, PMS5003, analog/digital inputs

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
