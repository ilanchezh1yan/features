/**
 * @file I2C.h
 * @brief I2C communication interface for MAX17261 driver.
 *
 * Contains function declarations for low-level I2C
 * communication between the Arduino Uno and I2C slave
 * devices such as the MAX17261 fuel gauge.
 *
 * Provides register-level read and write operations.
 */
#ifndef I2C_H
#define I2C_H

#include <Arduino.h>
#include <Wire.h>

#define STANDARD 100000
#define FAST    400000
#define HIGH_SPEED 3400000

#define MODE STANDARD



/**
* @brief Initialize I2C peripheral.
*
* Configures Arduino Wire library and prepares
* communication with I2C slave devices.
  */
  void I2C_Init(void); 

/**
* @brief Read a 16-bit register from an I2C device.
*
* @param deviceAddr 7-bit I2C slave address.
* @param reg Register address.
*
* @return Register contents.
  */
  uint16_t I2C_ReadRegister(uint8_t deviceAddr, uint8_t reg);

/**
* @brief Write a 16-bit value to an I2C device register.
*
* @param deviceAddr 7-bit I2C slave address.
* @param reg Register address.
* @param data Data to write.
  */
  void I2C_WriteRegister(uint8_t deviceAddr,
  uint8_t reg,
  uint16_t data);

#endif /* I2C_H */
