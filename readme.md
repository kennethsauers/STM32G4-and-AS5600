# AS5600 I2C Driver for STM32G4

This project provides a simple I2C interface for communicating with the **AS5600** magnetic rotary encoder using an **STM32G4 Nucleo** development board.

---

## 🔍 What is the AS5600?

The **AS5600** is a 12-bit magnetic rotary position sensor capable of measuring angles from 0 to 360°. It outputs absolute angle data via an I2C or PWM interface. In this project, we use its **I2C interface**.

---

## 📌 Hardware Connections

- **Sensor:** AS5600 Breakout Board
- **MCU Board:** STM32G4 Nucleo (e.g., NUCLEO-G431KB)
- **Power Supply:** 3.3V from the Nucleo board

| AS5600 Pin | Connected to STM32G4 | Notes             |
|------------|----------------------|-------------------|
| VCC        | 3.3V pin             | Power from Nucleo |
| GND        | GND pin              | Ground            |
| SDA        | Pin 17 (I2C1 SDA)    | From .ioc config  |
| SCL        | Pin 21 (I2C1 SCL)    | From .ioc config  |

Ensure pull-up resistors (~4.7kΩ) are present on SDA and SCL lines (many breakout boards include them by default).

---

## 🧰 Functions Provided

These are the **three public functions** available in the `AS5600i2C` driver:

### `uint8_t AS5600_ReadStatus(void)`
Reads the sensor's status register (0x0B), which indicates magnetic field strength and magnet detection status.

---

### `uint16_t AS5600_ReadRawAngle(void)`
Reads the raw 12-bit angle value from registers `0x0C` (high byte) and `0x0D` (low byte). This is the uncalibrated angle reading.

---

### `uint16_t AS5600_ReadAngle(void)`
Reads the processed angle value from registers `0x0E` and `0x0F`. This value may be adjusted by internal zero position settings.

---

## 📄 Notes

- The I2C address used is `0x36` (7-bit), which is left-shifted by 1 to form the 8-bit address used by STM32 HAL: `0x6C`.
- The resolution of the sensor is **4096 steps per revolution** (12 bits).
- Make sure `MX_I2C1_Init()` is called before using any AS5600 functions.
- `printf()` should be configured for UART output for debugging.

---

## 🛠 Example Usage in `main.c`

```c
#include "AS5600i2C.h"

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();

  uint8_t status = AS5600_ReadStatus();
  uint16_t raw_angle = AS5600_ReadRawAngle();
  uint16_t angle = AS5600_ReadAngle();

  while (1)
  {
    printf("Status: 0x%02X | Raw: %u | Angle: %u\r\n",
           AS5600_ReadStatus(),
           AS5600_ReadRawAngle(),
           AS5600_ReadAngle());
    HAL_Delay(500);
  }
}
