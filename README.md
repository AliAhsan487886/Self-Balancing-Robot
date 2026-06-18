# 🤖 BalanceBot

A self-balancing robot with light-following capability, built on an **ESP32** with an **MPU6050** IMU, dual DC motors via a **TB6612FNG** driver, and three **LDR** sensors. Includes a real-time Wi-Fi dashboard for live telemetry and PID tuning.

<img width="896" height="1100" alt="Self_Balancing_Robot" src="https://github.com/user-attachments/assets/cf159660-c0c8-45e2-882d-1a2d5c47c9c3" />

---

## Features

- **Self-Balancing** — Complementary filter + PID control running at 100 Hz
- **Light Following** — Three-LDR array with a separate PID loop at 20 Hz
- **Live Web Dashboard** — Served directly from the ESP32; no app required
- **OTA PID Tuning** — Adjust Balance and Light PID gains from the browser in real time
- **Motor Telemetry** — View angle, error, motor outputs, and sensor values live

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | ESP32 |
| IMU | MPU6050 (I²C) |
| Motor Driver | TB6612FNG |
| Motors | 2× DC gear motors |
| Light Sensors | 3× LDR (Left / Center / Right) |
| Power | LiPo battery (recommended) |

---

## Pin Map

### Motor Driver (TB6612FNG)

| Signal | ESP32 GPIO |
|---|---|
| PWMA | 4 |
| AIN1 | 5 |
| AIN2 | 6 |
| PWMB | 16 |
| BIN1 | 7 |
| BIN2 | 15 |
| STBY | 10 |

### MPU6050 (I²C)

| Signal | ESP32 GPIO |
|---|---|
| SDA | 8 |
| SCL | 9 |

### LDR Sensors (Analog)

| Sensor | ESP32 GPIO |
|---|---|
| Left | 3 |
| Center | 2 |
| Right | 1 |

### Circuit Diagram

<img width="1385" height="752" alt="Circuit_Diagram" src="https://github.com/user-attachments/assets/7a1e4367-3f9d-4cf4-b411-6472ec2b8107" />

---

## Software Dependencies

Install these libraries via the **Arduino Library Manager** or **PlatformIO**:

- `Wire` (built-in)
- `WiFi` (built-in with ESP32 core)
- `WebServer` (built-in with ESP32 core)
- `math.h` (built-in)

**ESP32 Arduino Core** is required. Install it through the Arduino Boards Manager:
`https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`

---

## Configuration

Open `BalanceBot.ino` and update the Wi-Fi credentials before uploading:

```cpp
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

### Default PID Values

| Parameter | Default |
|---|---|
| Kp (Balance) | 18.1 |
| Ki (Balance) | 0.16 |
| Kd (Balance) | 2.7 |
| Angle Setpoint | -0.8° |
| Kp (Light) | 0.050 |
| Ki (Light) | 0.000 |
| Kd (Light) | 0.006 |

### Calibration Offsets

```cpp
float gyroYOffset = -0.40;
float accZOffset  = 1939.25;
```

These may need adjustment for your specific MPU6050 unit. Place the robot perfectly upright, read the raw values via Serial, and update accordingly.

---

## Getting Started

1. Clone this repository
   ```bash
   https://github.com/AliAhsan487886/Self-Balancing-Robot.git
   ```
2. Open `BalanceBot.ino` in the Arduino IDE
3. Set your Wi-Fi credentials
4. Select **ESP32** as the target board
5. Upload the sketch
6. Open the Serial Monitor at **115200 baud** — the ESP32's IP address will be printed once connected
7. Navigate to `http://<IP_ADDRESS>` in any browser on the same network

---

## Web Dashboard

The dashboard is served directly by the ESP32 and provides:

- **Live Telemetry** — Angle, error, PID outputs, motor signals, and light error
- **Tilt Indicator** — Visual bar showing ±30° tilt in real time
- **Motor Output Bars** — Left and right motor signals
- **LDR Readout** — Live values for all three light sensors with active-sensor highlighting
- **Balance PID Panel** — Adjust Kp, Ki, Kd, and angle setpoint on the fly
- **Light PID Panel** — Adjust light-following gains without reflashing

Dashboard updates at **100 ms** intervals.

<img width="960" height="540" alt="CSV File Dashborad" src="https://github.com/user-attachments/assets/8da9db1c-7562-40dd-a336-1823ff771b02" />

---

## Control Architecture

```
MPU6050 ──► Complementary Filter ──► Angle Estimate
                                          │
                          balance_offset + forwardBoost
                                          │
                                    PID (100 Hz)
                                          │
                               balance signal (u_bal)
                                          │
           LDR Array ──► Light PID (20 Hz) ──► turn signal (u_dir)
                                          │
                         Left Motor = u_bal + u_dir
                         Right Motor = u_bal − u_dir
```

---

## How Light Following Works

- **Left/Right LDRs** generate a steering error: `error = leftL - rightL`
- A PID loop converts this error into a turn correction applied to both motors
- **Center LDR** detects a bright source directly ahead and applies a small forward lean (`forwardBoost = 1.2°`) while reducing turn authority — making the robot drive toward frontal light

---

## License

MIT License — free to use, modify, and distribute.

---

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you'd like to change.
