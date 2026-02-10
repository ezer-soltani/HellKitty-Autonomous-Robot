# Autonomous Mobile Robot with STM32 & FreeRTOS

<p align="center">
  <img width="400" alt="HellKitty Robot" src="https://github.com/user-attachments/assets/7b001b46-020f-4a15-b75d-87ee0aa59870" />
</p>

## Project Overview
**HellKitty** is an autonomous combat robot designed for a real-time "Cat and Mouse" game on a tabletop arena. The system integrates advanced perception, precise motor control, and a robust multitasking architecture to navigate and react to its environment autonomously.

### System Architecture
<img width="800" alt="Architectural Schema" src="docs/schema_arch.png" />

## Key Features
- **Real-time Navigation:** Multitasking architecture for simultaneous sensor processing, control, and strategy.
- **Data Fusion:** Combined input from **YDLIDAR X2** and **4x VL53L0X TOF** sensors for robust obstacle detection and safety.
- **Proactive Safety:** Integrated anti-fall algorithms (sequence pivot) and high-G collision detection via IMU.
- **High Performance:** Powered by an **STM32G431** (Cortex-M4 @ 170MHz) with hardware FPU.

## Technology Stack
- **Microcontroller:** STM32G431CBU6
- **OS:** FreeRTOS
- **Language:** C (HAL/LL Drivers)
- **CAD:** KiCad (4-layer PCB)
- **Sensors:** Lidar (UART/DMA), TOF (I2C Polling), IMU (ADXL343)
- **Connectivity:** Bluetooth (HC-05) for telemetry and control.

## Hardware Specifications
The hardware is designed for reliability and high performance in a compact form factor.

<p align="center">
  <img width="600" alt="PCB 3D View" src="https://github.com/user-attachments/assets/481b72ed-1033-43fa-afc0-7f35f21c6596" />
</p>

- **Custom 4-layer PCB:** Optimized for high-speed signal integrity.
- **Microcontroller:** STM32G431CBU6 (Cortex-M4 @ 170MHz) with dedicated motor control timers.
- **Power Management:** Onboard Buck and LDO regulators for stable 3.3V and 5V rails.
- **Actuators:** 2x DC Motors with high-resolution quadratic encoders.

### Electronics Design
| Module | Schematic |
| :--- | :--- |
| **Control** | <img width="400" src="https://github.com/user-attachments/assets/20eed7e6-8ec8-43f0-a155-ee942acf7542" /> |
| **Power** | <img width="400" src="https://github.com/user-attachments/assets/fa8be56d-729b-44cd-a620-70aa3347fee2" /> |
| **Motors** | <img width="400" src="https://github.com/user-attachments/assets/a448f5a5-3051-4683-b620-6901f1f0ada8" /> |
| **Sensors** | <img width="400" src="https://github.com/user-attachments/assets/a634e98e-761b-432a-a299-bae5318e2d9d" /> |

## Firmware Architecture
The firmware leverages **FreeRTOS** for deterministic real-time scheduling:

| Task | Priority | Frequency | Role |
| :--- | :---: | :---: | :--- |
| **vSafetyTask** | MAX | 100Hz | High-priority TOF polling & fall prevention (Stop -> Pivot -> Advance). |
| **vControlTask** | Med | 100Hz | Speed PID loops, Odometry calculation, and Strategy FSM. |
| **vLidarTask** | Med | 10Hz | DMA-based Lidar parsing, object segmentation, and tracking. |
| **vImuTask** | Low | 20Hz | Shock detection (>3.5G) for role switching (Cat/Mouse). |

**Inter-task Communication:**
- **Semaphores & Mutexes:** Protect shared peripherals (I2C1, UART) from concurrent access.
- **DMA (Direct Memory Access):** Efficient background reception of Lidar data streams.

## Key Algorithms
- **PID Control:** Optimized `Kp=5.0, Ki=20.0` for motor velocity regulation.
- **Clustering:** Carthesian conversion and fusion of Lidar points (`MERGE_THRESHOLD = 150mm`).
- **Strategy FSM:** Dynamic role switching based on sensor inputs and collision events.

## Telemetry & Debug
Real-time monitoring is achieved via the HC-05 Bluetooth module, providing insights into sensor data, state transitions, and system health.

<p align="center">
  <img width="300" alt="Bluetooth Terminal Logs" src="https://github.com/user-attachments/assets/a8628635-f0d2-4451-8770-b7ae8a8bb45b" />
</p>

## Installation & Setup
1. Clone this repository.
2. Open the `firmware/` directory as an existing project in **STM32CubeIDE**.
3. Build the project and flash it to the STM32G431 target.
4. Use the `scripts/visualizer.py` for real-time Lidar data visualization (requires Python).

## Directory Structure
- `firmware/`: STM32CubeIDE project and source code.
- `hardware/`: KiCad schematics, PCB layout, and libraries.
- `docs/`: Technical datasheets, architecture diagrams, and project reports.
- `scripts/`: Telemetry and visualization tools.
