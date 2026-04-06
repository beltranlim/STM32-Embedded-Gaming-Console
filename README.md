# STM32L4S5 Embedded Gaming Console

An embedded systems project built on the **STM32L4S5VIT6** Discovery IoT board, extending a baseline 2-game assignment into a fully-featured multi-game interactive console with hardware peripherals, real-time sensor fusion, and a Python dashboard for live telemetry.

> **Board:** STM32L4S5VIT6 (IoT Discovery Kit) | **IDE:** STM32CubeIDE | **Language:** C (HAL), Python

---

## Features Overview

| Feature | Description |
|---|---|
| 3 Game Modes | Red Light Green Light, Catch & Run, Space Invaders |
| OLED Display | SSD1306 128×64 via I2C — game rendering & graphics |
| PWM Buzzer | TIM3 CH1 (PB4) — frequency-controlled audio feedback |
| Python Dashboard | UART serial dashboard for live sensor telemetry |
| IMU Sensor | LSM6DSL — accelerometer, gyroscope, tilt interrupt |
| Magnetometer | Proximity zone detection with 3-level state machine |
| Environmental | Temperature, humidity, pressure monitoring |

---

## Hardware Architecture

```
STM32L4S5VIT6
├── I2C1 (PB8/PB9)        → SSD1306 OLED (128×64)
├── TIM3 CH1 (PB4/D5)     → PWM Buzzer
├── UART4 (PA0/PA1)       → Python Serial Dashboard
├── GPIO PC13 (EXTI13)    → Push Button (interrupt-driven)
├── GPIO PD11 (EXTI11)    → LSM6DSL Tilt Interrupt
├── I2C1 (Internal)       → LSM6DSL IMU (Accel/Gyro)
├── I2C1 (Internal)       → LIS3MDL Magnetometer
├── I2C1 (Internal)       → HTS221 Temp/Humidity Sensor
└── I2C1 (Internal)       → LPS22HB Pressure Sensor
```

---

## Game Modes

### 1. Red Light, Green Light
- Alternates between Green Light and Red Light states every 10 seconds (non-blocking via `HAL_GetTick()`)
- **Green Light:** Transmits accelerometer, gyroscope, temperature, humidity, and pressure readings via UART every 2 seconds
- **Red Light:** Monitors 3-axis accelerometer and gyroscope readings against a calibrated baseline; triggers game-over if motion threshold exceeded
- Baseline captured on the 2nd sensor reading (1st discarded to avoid motion noise)
- PWM buzzer plays differentiated frequency chirps for each state

### 2. Catch & Run
- Magnetometer polled every 200ms; deviation from baseline classified into 3 proximity levels using a state machine (`MAG_LEVEL_SAFE`, `MAG_LEVEL_LOW`, `MAG_LEVEL_MED`, `MAG_LEVEL_HIGH`)
- Each proximity level triggers a distinct LED blink frequency and buzzer tone
- State transitions reset the escape timer — player must maintain safe state for 3 seconds to "escape"
- Push button press within 3 seconds of zone violation triggers "PLAYER CAPTURED" outcome
- Environmental sensors (temp, humidity, pressure) polled every 1 second with threshold violation alerts

### 3. Space Invaders *(Custom Enhancement)*
- Implemented entirely on the 128×64 SSD1306 OLED display
- Spaceship movement driven by LSM6DSL tilt interrupt on PD11 (EXTI15_10 vector)
- Accelerometer X-axis polarity determines left/right ship movement; boundary-clamped to display width
- Meteors spawn at randomised X positions every 5 seconds with variable descent speeds
- Collision detection via bounding-box pixel coordinate comparison
- Full non-blocking game loop using state machine (`SPACESHIP_STATE_INIT` → `WAIT_STABILIZE` → `CAPTURE_BASELINE` → `PLAYING`)
- Avoids `HAL_Delay()` entirely — all timing managed through `HAL_GetTick()` delta comparisons

---

## Key Technical Implementations

### Non-Blocking Architecture
All timing is handled via `HAL_GetTick()` delta comparisons throughout the codebase — no `HAL_Delay()` calls in the main game loops. This ensures the main loop remains responsive to interrupts and concurrent sensor polling.

### Interrupt Handling
Two EXTI interrupts share the `EXTI15_10_IRQn` vector:
- **PD11** → LSM6DSL tilt interrupt (higher software priority, checked first in callback)
- **PC13** → Push button (falling edge, with double-press detection logic using timestamp windowing)

### PWM Buzzer
TIM3 configured at 1MHz (80MHz SysCLk, prescaler = 79). Tone control is achieved by varying the PWM **frequency** (Period register) at a fixed 50% duty cycle — not the duty cycle itself. A non-blocking beep sequence manager uses global state flags (`buzzer_active`, `beep_seq_active`) to handle multi-tone sequences without blocking the main loop.

### I2C Peripheral Configuration
I2C1 initialised in open-drain mode (AF4) on PB8/PB9, standard mode at 100kHz. Shared bus serves the OLED and all on-board sensors.

### Sensor Baseline Calibration
All sensors use a consistent pattern: discard first reading (noisy), capture second reading as baseline, then compare subsequent readings against the baseline delta. Magnetometer values converted from raw LSB to milligauss (sensitivity: 6.842 LSB/gauss).

### Python Dashboard
A serial dashboard (`dashboard.py`) receives UART4 telemetry from the STM32 and plots live sensor readings (accelerometer XYZ, gyroscope XYZ, temperature, humidity, pressure) in real time.

---

## Enhancements Over Baseline

| Enhancement | Implementation Detail |
|---|---|
| SSD1306 OLED | I2C-driven, u8g2 library, 128×64 game display + bitmap rendering |
| PWM Buzzer | TIM3 CH1, frequency-modulated, non-blocking multi-tone sequences |
| Space Invaders Game | Full FSM, tilt-interrupt control, collision detection, OLED rendering |
| Python Dashboard | UART serial plotter for live multi-sensor telemetry visualisation |
| ADC Light Sensor | 12-bit ADC, averaged baseline, threshold delta detection with 2s cooldown |

---

## Project Structure

```
├── Core/
│   ├── Src/
│   │   ├── main.c              # Main game loop and mode switching
│   │   ├── sensor_configs.c    # Peripheral init (I2C, PWM, GPIO, ADC)
│   │   ├── helper_functions.c  # Threshold logic, state machines
│   │   └── interrupt_handlers.c# EXTI callbacks (tilt + button)
│   └── Inc/
├── Drivers/                    # STM32 HAL + BSP board support
├── dashboard.py                # Python serial telemetry dashboard
├── STM32L4S5VITX_FLASH.ld      # Linker script (Flash)
└── STM32L4S5VITX_RAM.ld        # Linker script (RAM)
```

---

## Setup & Flashing

1. Open project in **STM32CubeIDE**
2. Build the project (`Project > Build All`)
3. Connect the STM32L4 Discovery board via USB
4. Flash via `Run > Debug` or use ST-LINK programmer
5. For the Python dashboard, run:
   ```bash
   pip install pyserial matplotlib
   python dashboard.py
   ```
   Ensure the correct COM port is set in `dashboard.py` to match your ST-Link virtual COM port.

*NUS EEE2028 Microcontroller Module — AY25/26 Semester 1*
