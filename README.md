# Battery Management System - Microcontroller Firmware

This repository contains the firmware implementation for a Battery Management System (BMS) using STM32F407VG microcontroller. The project utilizes CAN communication for real-time data transfer, with data encoding functionalities defined in `func.h` to ensure efficient and structured message handling.

## Features

- **CAN Communication**:
  - Configured for standard CAN messages using the `HAL_CAN` library.
  - Sends and receives battery-related data such as voltages, currents, and temperatures.
- **Data Encoding**:
  - Encodes `uint8_t`, `int16_t`, and `uint16_t` values into byte arrays.
  - Ensures proper little-endian format for transmission compatibility.
- **Time Logging**:
  - Captures date and time using the RTC (Real-Time Clock) module.
  - Adds timestamps to transmitted messages.
- **GPIO Control**:
  - Toggles LEDs on GPIOD to indicate transmission and system status.

## Installation

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/Shakilakp/BatteryManagementSystem-DataTransfer.git
2. **Setup STM32CubeIDE**:
   - Import the project into STM32CubeIDE.
   - Ensure you have the required STM32 HAL libraries.
3. **Build and Flash**:
   - Connect your STM32 microcontroller board.
   - Build and flash the firmware onto the board.
  
## Configuration

- **CAN Settings**:
  - Adjust the CAN channel ID and baud rate in the MX_CAN1_Init function.
- **RTC Settings**:
  - Ensure the RTC is properly initialized using MX_RTC_Init.

 ## Dependencies

- STM32CubeIDE
- STM32 HAL Drivers
- STM32F407VG Microcontroller

## Usage

1. Connect the STM32 board to the CAN bus system.
2. Power the board and ensure proper clock and power configuration.
3. Monitor the CAN bus messages to verify successful transmission.
