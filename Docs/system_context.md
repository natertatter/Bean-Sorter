# System Architecture: UART Precision Control

## 1. Hardware Bill of Materials
* **Controller:** Raspberry Pi 4 Model B
* **Drivers:** 2x TMC2208 (UART Mode)
* **Encoders:** 2x AS5600 (I2C Mode)
* **Components:** 2x 1kΩ Resistors (Crucial for UART wiring)
* **Power:** 24V PSU (Motors) + 5V/3A PSU (Pi)

## 2. Communication Strategy
* **Motor Control (UART):**
    * **Left Motor:** Uses default UART0 (`/dev/serial0`).
    * **Right Motor:** Uses hardware UART5 (`/dev/ttyAMA1`).
    * **Protocol:** Single-Wire UART (Requires 1k resistor on TX line).
    * **Baudrate:** 115200.
* **Encoder Feedback (I2C):**
    * **Left Encoder:** Bus 1 (Default).
    * **Right Encoder:** Bus 3 (Overlay).

## 3. Pinout & Wiring Map
| Device | Signal | Physical Pin | BCM (GPIO) | Connection Note |
| :--- | :--- | :--- | :--- | :--- |
| **Left Encoder** | SDA | 3 | GPIO 2 | I2C Bus 1 |
| **Left Encoder** | SCL | 5 | GPIO 3 | I2C Bus 1 |
| **Left Motor** | RX | 10 | GPIO 15 | Direct to PDN_UART |
| **Left Motor** | TX | 8 | GPIO 14 | **Via 1kΩ Resistor** to PDN_UART |
| **Right Encoder**| SDA | 7 | GPIO 4 | I2C Bus 3 |
| **Right Encoder**| SCL | 29 | GPIO 5 | I2C Bus 3 |
| **Right Motor** | RX | 33 | GPIO 13 | Direct to PDN_UART |
| **Right Motor** | TX | 32 | GPIO 12 | **Via 1kΩ Resistor** to PDN_UART |
| **TMC2208** | VIO | 17 | 3V3 Power | **MUST BE 3.3V** |
| **TMC2208** | VM | Ext | N/A | 12V-24V External |

## 4. Software Config Requirements
Add to `/boot/config.txt`:
```bash
dtoverlay=i2c3,pins_4_5
dtoverlay=uart5
enable_uart=1