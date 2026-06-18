# Self-Balancing Robot with Light Following

**UET Lahore – Electrical Engineering Department**  
**Control Systems Lab – CEP Labs 12, 13, 14, 15**

---

## Overview

This repository contains the complete implementation of a **self-balancing two-wheel robot** (inverted pendulum) with **light-following capability**, developed as a Complex Engineering Problem (CEP) across four control systems lab experiments.

| Lab | Topic | Status |
|-----|-------|--------|
| Lab 12 | PID Balance Controller | ✅ |
| Lab 13 | MIMO Light-Following Control | ✅ |
| Lab 14 | Model Validation & Parameter Identification | ✅ |
| Lab 15 | Lead–Lag Controller Design | ✅ |

---

## Hardware

| Component | Part Used |
|-----------|-----------|
| Microcontroller | ESP32 DevKit |
| IMU | MPU6050 (I2C) |
| Motor Driver | L298N / TB6612FNG |
| Motors | 2× DC gear motors |
| Light Sensors | 2× LDR + voltage divider |
| Power Supply | 7.4 V Li-Po battery |
| Chassis | 2WD robot platform |

### Wiring Summary

```
ESP32 GPIO 18  →  Motor Left  PWM
ESP32 GPIO 19  →  Motor Left  DIR
ESP32 GPIO 22  →  Motor Right PWM
ESP32 GPIO 23  →  Motor Right DIR
ESP32 GPIO 21  →  MPU6050 SDA
ESP32 GPIO 22  →  MPU6050 SCL   (use separate I2C bus or remap if conflict)
ESP32 GPIO 34  →  LDR Left  (ADC)
ESP32 GPIO 35  →  LDR Right (ADC)
```

---

## Software Architecture

### Lab 12 – PID Balance Controller

The robot is modelled as an **inverted pendulum**:

```
θ'' = a·θ + b·u
```

A PID controller stabilizes the tilt angle:

```
u = Kp·e + Ki·∫e dt + Kd·de/dt
```

Tilt angle is estimated using a **complementary filter**:

```
angle = α·(angle + gyro·dt) + (1−α)·acc_angle      α = 0.98
```

### Lab 13 – MIMO Light Following

Two control loops run simultaneously at different frequencies:

| Loop | Variable | Frequency | Role |
|------|----------|-----------|------|
| Inner (fast) | Tilt angle | 200 Hz | Balance stabilization |
| Outer (slow) | Light error | 20 Hz | Directional navigation |

Motor commands combine both loops:

```
u_left  = u_balance + u_direction
u_right = u_balance − u_direction
```

### Lab 14 – Model Validation

The closed-loop system is modelled as a 3rd-order ODE:

```
θ'' + (Kd/J)·θ' + ((Kp − mgh)/J)·θ + (Ki/J)·∫θ = 0
```

Unknown parameters **m**, **h**, **J** are identified by matching the simulated impulse response to experimental data (RMSE ≤ 1°).

### Lab 15 – Lead–Lag Controller

A lead–lag compensator replaces PID for improved frequency-domain performance:

```
C(s) = K · [(s + z₁)/(s + p₁)] · [(s + z₂)/(s + p₂)]
```

Discretized using the **Tustin (bilinear) method** at 200 Hz and implemented as a difference equation:

```c
u[k] = a1*u[k-1] + a2*u[k-2]
     + b0*e[k]   + b1*e[k-1] + b2*e[k-2];
```

---

## Repository Structure

```
self-balancing-robot/
├── src/
│   ├── main.cpp              # Main control loop (ESP32 / PlatformIO)
│   ├── config.h              # All tunable parameters (PID gains, pins, timing)
│   ├── pid.h                 # Generic PID controller class
│   ├── angle_estimator.h     # Complementary filter
│   ├── motor.h               # Motor driver interface (ESP32 LEDC PWM)
│   └── data_logger.h         # Ring-buffer logger for impulse response (Lab 14)
├── matlab/
│   ├── lab14_model_validation.m   # Parameter identification & model validation
│   └── lab15_leadlag_design.m     # Lead–lag design, Bode, simulation, discretization
├── data/
│   └── impulse_response.csv  # (paste Serial Monitor output here for Lab 14)
├── platformio.ini            # PlatformIO build configuration
├── .gitignore
└── README.md
```

---

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) (VSCode extension recommended)
- MATLAB R2021a or later (Control System Toolbox required)
- [I2Cdevlib MPU6050](https://github.com/jrowberg/i2cdevlib) library (auto-installed by PlatformIO)

### Build & Flash

```bash
# Clone the repo
git clone https://github.com/<your-username>/self-balancing-robot.git
cd self-balancing-robot

# Build and upload via PlatformIO
pio run --target upload

# Open serial monitor (115200 baud)
pio device monitor
```

### Tuning the PID Gains

All gains are in `src/config.h`. Suggested tuning sequence:

1. Set `KI_BAL = 0`, `KD_BAL = 0`. Increase `KP_BAL` until the robot reacts to tilt.
2. Increase `KD_BAL` to reduce oscillations.
3. Add small `KI_BAL` to eliminate steady-state lean.

---

## Lab 14 – Data Logging Workflow

1. Flash the code and balance the robot.
2. Give the robot a small push (~3–5°).
3. The logger auto-triggers on `DISTURBANCE_THRESHOLD_DEG` and fills a 2000-sample buffer.
4. CSV data is printed to Serial Monitor between `--- DATA START ---` and `--- DATA END ---` markers.
5. Copy the CSV block into `data/impulse_response.csv`.
6. Run `matlab/lab14_model_validation.m` — adjust `m`, `h`, `J` until RMSE ≤ 1°.

---

## Lab 15 – Lead–Lag Implementation

1. Run `matlab/lab14_model_validation.m` to confirm model parameters.
2. Run `matlab/lab15_leadlag_design.m` — it prints the discretized difference equation coefficients.
3. Paste those coefficients into your embedded implementation.
4. Compare step response metrics (overshoot, settling time) against PID.

---

## Results Summary

| Metric | PID (Lab 12) | Lead–Lag (Lab 15) |
|--------|:---:|:---:|
| Overshoot | — % | — % |
| Settling Time | — s | — s |
| Phase Margin | — ° | — ° |

*(Fill in after experimental testing)*

---

## Authors

**Ali Ahsan (Malik)**  
B.Sc. Electrical Engineering, UET Lahore  
Session 2020

---

## Acknowledgements

Lab manuals designed by the **Electrical Engineering Department, UET Lahore** as part of the Control Systems Lab CEP series.
