# Flipper Car ROS 2 Workspace

A dual-mode, auto-inverting RC flipper car controlled via ESP32, dual N-channel MOSFETs, a mechanical servo polarity inverter, and ROS 2.

---

## 🛠 Features
- **Differential Tank Steering:** Independent Left/Right PWM channels via low-side N-channel MOSFETs.
- **Hardware Polarity Inverter:** Single SG90 servo mechanically swaps motor terminal contacts for bidirectional drive without a full H-bridge.
- **Auto-Flip Inversion:** SW-520D tilt sensor triggers automated reverse compensation and control-remapping when the chassis is flipped upside down.
- **Micro-ROS / ROS 2 Integration:** Teleoperation and status telemetry over WiFi/Serial.

---

## ⚡ Hardware Pinout (ESP32)

| Component | ESP32 Pin | Function |
|---|---|---|
| **Right Motors MOSFET Gate** | `IO18` | PWM Speed Control (Right Side) |
| **Left Motors MOSFET Gate** | `IO33` | PWM Speed Control (Left Side) |
| **SG90 Reversing Servo** | `IO16` | Mechanical Polarity Switch (0° / 180°) |
| **SW-520D Tilt Sensor** | `IO22` | Chassis Orientation Detection (`INPUT_PULLUP`) |

---

## 🚀 Getting Started

### Prerequisites
- Ubuntu 22.04 / 24.04
- ROS 2 (Humble / Iron / Jazzy)
- `colcon` build tools

### Build & Run
```bash
# Clone and enter workspace
cd ~/ROS_projects/flipper_car_ws

# Build the packages
colcon build --symlink-install

# Source the overlay
source install/setup.bash

# Run using the startup script
./run.sh
```

## 🔌 Hardware Architecture & Circuit Schematic


![Circuit Schematic](hardware/schematic.png)

> **📌 Schematic Notes & Architecture Disclaimers:**
> - **Mechanical Polarity Inverter Abstraction:** In the schematic above, the DC motors are depicted directly wired to the low-side MOSFET channels for visual clarity. Physical bidirectional control (Forward/Reverse) is achieved via a dedicated SG90 servo mechanical wiper assembly that physically swaps motor terminal contacts by 180°.
> - **ESP32 Power Infrastructure:** Core power regulation, decoupling capacitors, and the EN boot/reset delay network for the ESP32 module are omitted from the diagram for schematic readability (an external/onboard 3.3V LDO regulator powers the microcontroller from the main 5V buck rail).