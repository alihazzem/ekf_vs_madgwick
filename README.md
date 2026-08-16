# Bachelor Thesis: Real-Time Wearable Robotic Teleoperation & Sensor Fusion Analysis

**An academic and advanced engineering project focusing on high-frequency IMU sensor fusion, comparing the Extended Kalman Filter (EKF) against the Madgwick algorithm in highly dynamic robotic teleoperation environments.**

This repository contains the complete firmware and architectural framework developed for my Bachelor Thesis. While the core thesis has been completed, this project is actively maintained and continuously upgraded with industry-standard embedded engineering practices.

---

## Academic Objective: EKF vs. Madgwick
The primary research objective of this thesis was to design a wearable, multi-sensor motion tracking suit capable of driving robotic limbs with zero perceptible latency, and to use this platform to empirically evaluate sensor fusion algorithms.

*   **The Challenge:** Fast, erratic human movements (like wrist snaps) easily confuse standard IMU filters, causing the accelerometer gravity-vector to distort and the gyroscope to drift.
*   **The Comparison:** We implemented both a standard **Madgwick** filter and a custom **Extended Kalman Filter (EKF)**.
*   **The Result:** The EKF, utilizing the hardware Floating-Point Unit (FPU) of the STM32 for complex matrix operations, proved vastly superior in isolating linear acceleration from gravity during fast transients, resulting in absolute drift-free 3D Quaternions at 200 Hz.

---

## System Architecture
The system is a highly distributed, multi-core architecture designed to isolate heavy mathematics from unpredictable wireless network stacks.

### 1. Mathematical Core: STM32F411 (Black Pill)
Running **FreeRTOS**, the STM32 acts as the central nervous system. It strictly enforces a 5-millisecond (200 Hz) control loop deadline.
*   **Dual-Bus I2C:** Ingests raw 6-DOF data from 4x MPU6050/GY-91 IMUs split across two independent I2C buses (400kHz) to prevent address collisions and minimize bus time.
*   **Kinematics Engine:** Converts EKF Quaternions into physical Pitch/Roll angles, applies Exponential Moving Average (EMA) smoothing, and commands hardware PWM timers (`TIM3`, `TIM4`) to drive the physical robot servos.
*   **Gait Analysis:** Runs a continuous state machine to analyze human leg movement, detecting Heel-Strike, Mid-Swing, and Toe-Off phases in real-time.

### 2. Communications Hub: ESP32 (Dual-Core)
The ESP32 acts as a wireless bridge, utilizing its dual cores to handle two completely separate 2.4GHz Bluetooth (BLE) links simultaneously without thread-blocking.
*   **Core 1 (BLE Client - The Gripper):** Connects to a commercial Myo Armband to ingest raw EMG muscle signals. It calculates dynamic RMS energy thresholds to detect muscle flexion, instantly sending binary "FLEX/RELAX" commands to the STM32 over a 921600-baud UART stream.
*   **Core 0 (BLE Server - The Telemetry):** Dedicated to reading leg kinematic data from the STM32 and broadcasting it to the receiving robot.

---

## Advanced Firmware Engineering (Post-Thesis Upgrades)
Since the completion of the thesis, the codebase has been heavily refactored to showcase advanced embedded C techniques and hardware-level optimizations:

*   **Zero-Latency DMA Telemetry:** The STM32 uses Direct Memory Access (DMA) to stream 16-byte binary telemetry packets out of the UART port. This offloads the 1.3ms transmission time entirely from the CPU, reserving 100% of the clock cycles for EKF mathematics.
*   **Aggressive BLE Overrides:** The ESP32 Bluetooth stack has been modified via ESP-IDF APIs to force an aggressive **7.5ms connection interval**. This bypasses standard BLE buffering algorithms, allowing raw 200Hz data transfer to the receiving robot.
*   **Non-Volatile Flash Memory Boot:** Gyroscope environmental noise offsets are calculated once and burned directly into Sector 7 of the STM32's internal Flash Memory. This enables instant-booting of the robot without requiring the user to hold the sensors perfectly still on every power-up.

---

## Autonomous Failsafes
To ensure the physical safety of the robot and the operator, the firmware includes self-healing failsafes:

*   **Dead-Man's Switch (Gripper):** If the ESP32 loses connection with the Myo armband (or if the battery dies), the STM32 detects the UART silence (>500ms) and automatically forces the heavy DC motor gripper into a safe, fully-open position, while allowing the main arm servos to continue tracking normally.
*   **Auto-Tare Calibration:** The system continuously monitors the leg IMUs in the background. If the user stands perfectly still (variance < 0.5 deg) for 5 continuous seconds, the STM32 automatically captures the new orientation matrix and tares the physical "0" reference without requiring a reboot.
*   **Hardware Override Boot:** If the saved Flash Memory sensors drift due to extreme temperature changes, the user can hold the physical `PA0` button during boot to force a memory wipe and a fresh 4-second environmental recalibration.
*   **Fault-Code Injection:** If the physical wire between the STM32 and ESP32 is severed, the ESP32 generates a synthetic BLE packet with a dedicated `status = 1` error byte, instantly halting the receiving robot.

---

*This project represents hundreds of hours of research, mathematical modeling, and low-level C programming. It serves as an active testbed for exploring the limits of wearable robotics and edge-compute sensor fusion.*
