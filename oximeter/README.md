# PPG Recorder (PPGR)

## Introduction

The **PPG Recorder (PPGR)** firmware is designed to acquire, process, and transmit photoplethysmogram (PPG) signals using a biomedical optical sensor system. The firmware supports heart rate and SpO₂ detection algorithms and provides connectivity options via **USB (FTDI)** and **Bluetooth Low Energy (BLE)**.

## Hardware Specifications

- **Microcontroller:** NRF52832
- **PPG Sensor Module:** AFE4490
- **Connectivity Modules:**
  - **USB via FTDI**
  - **Bluetooth Low Energy (BLE) module**
- **Power Supply:** 3.3V / 5V DC

## Heart Rate Detection Algorithm

- Acquire PPG signal from AFE4490 sensor at 25 hz sampling rate.
- Determine the max of the signal. Invert the signal.
- Apply moving average filter with window size 4.
- Apply Kalman filter for enveloping. 
- Find the number of peaks with moving window of size 5 and store its occurrence. 
- calculate the time interval by the averaging the occurrence.
  ```
  Heart Rate (bpm) = 60 * (sampling rate) / (time interval)
  ```

## SpO₂ Detection Algorithm

- Use two different wavelengths (typically Red and IR).

- Calculate the AC and DC components of each signal.

- Apply the Ratio of Ratios (R) formula:

  ```
  R = (AC_Red / DC_Red) / (AC_IR / DC_IR)
  ```
- Use the linear expression for determining the SpO2
  ```
  SpO2 = 127.05f - 41.238f * R
  ```

- Use an empirical formula or lookup table to convert **R** to **SpO₂ percentage**.

## Pin Configuration

| Function             | Microcontroller Pin | Description              |
| -------------------- | ------------------- | ------------------------ |
| Data Ready	       |	20	     | Generates interrupts     |
| AFE Reset            | 	17           | Resets the AFE4490       |
| SPI Chip Select (CS) | 	28           | SPI CS for AFE4490       |
| FTDI TX              | 	6            | Data transmission to PC  |
| SPI		       |   nrf52832 spi_1    | Communication to AFE     |
| ON LED       |        2          | indicator LED|

## Connectivity

### Using FTDI (USB)

- Serial communication via **FTDI USB-to-UART converter**.
- Baud rate: 115200 bps.

### Using BLE

- BLE service exposes:
  - Service ID : 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xe0, 0xff, 0x00, 0x00
  - Characteristic ID: 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xe1, 0xff, 0x00, 0x00

- Frame Format:
  | 2 byte header | length | 3 byte IR data | 3 byte red data | SpO2 | heart rate | CRC |
  |---------------|--------|----------------|-----------------|------|------------|------|
  - header - 0xBEFF
  - length - 0x09

>Note: If size not mentioned then it is of 1 byte size.
  
- Connect via a mobile app or BLE terminal to access live health data.

