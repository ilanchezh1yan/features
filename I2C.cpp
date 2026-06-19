/**
 * @file I2C.cpp
 * @brief Low-level I2C communication driver.
 *
 * Implements initialization and register-level I2C
 * transactions used by the MAX17261 fuel gauge driver.
 *
 * Provides abstraction of Arduino Wire library
 * functions for device communication.
 */

#include "I2C.h"

/******************************************************************************
 * Function Definitions
 *****************************************************************************/

/**
 * @brief Initialize I2C peripheral.
 *
 * Configures Arduino Wire library for communication
 * with I2C slave devices.
 *
 * @return None.
 */
void I2C_Init(void)
{
    Wire.begin();
    Wire.setClock(MODE);
}

/**
 * @brief Write a 16-bit value to an I2C device register.
 *
 * Data is transmitted LSB first as required by MAX17261.
 *
 * @param deviceAddr 7-bit I2C slave address.
 * @param reg Register address.
 * @param data 16-bit data value.
 *
 * @return None.
 */
void I2C_WriteRegister(uint8_t deviceAddr,
                       uint8_t reg,
                       uint16_t data)
{
    Wire.beginTransmission(deviceAddr);

    Wire.write(reg);

    /* LSB first */
    Wire.write((uint8_t)(data & 0xFF));
    Wire.write((uint8_t)((data >> 8) & 0xFF));

    Wire.endTransmission();
}

/**
 * @brief Read a 16-bit value from an I2C device register.
 *
 * Data is returned LSB first.
 *
 * @param deviceAddr 7-bit I2C slave address.
 * @param reg Register address.
 *
 * @return Register contents.
 */
uint16_t I2C_ReadRegister(uint8_t deviceAddr,
                          uint8_t reg)
{
    uint16_t value = 0;

    Wire.beginTransmission(deviceAddr);
    Wire.write(reg);

    uint8_t txStatus =
        Wire.endTransmission(false);

    if (txStatus != 0)
    {
        Serial.print("I2C TX ERR=");
        Serial.println(txStatus);

        return 0;
    }

    uint8_t count =
        Wire.requestFrom((int)deviceAddr, 2);

    if (count != 2)
    {
        Serial.print("I2C RX COUNT=");
        Serial.println(count);

        return 0;
    }

    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();

    value =
        ((uint16_t)msb << 8) |
        lsb;

    return value;
}