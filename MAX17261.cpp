/** 
* @file MAX17261.cpp 
* @brief MAX17261 Fuel Gauge Driver Implementation. 
* 
* Implements initialization, configuration and battery 
* parameter acquisition functions for the MAX17261. 
*/
#include "MAX17261.h"
#include "MAX17261_CONFIG.h"
#include "I2C.h"

#define RSENSE_VALUE    (0.01f)    /* 10mΩ */

/**
 * @brief Clear Power-On Reset flag.
 */
void MAX17261_ClearPOR(void)
{
    uint16_t status;

    status = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        STATUS_REG);

    status &= ~STATUS_POR;

    I2C_WriteRegister(
        MAX17261_I2C_ADDR,
        STATUS_REG,
        status);
}

/**
 * @brief Initialize MAX17261 fuel gauge.
 */
void MAX17261_Init(void)
{
    uint16_t modelCfg;
    uint16_t status;

    modelCfg =
        MODELCFG_REFRESH |
        MODELCFG_MODELID_LICOO2;

    I2C_WriteRegister(
        MAX17261_I2C_ADDR,
        MODELCFG_REG,
        modelCfg);

    delay(1000);

    status = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        STATUS_REG);

    if (status & STATUS_POR)
    {
        MAX17261_ClearPOR();
    }
}

/**
 * @brief Read battery voltage.
 */
float MAX17261_GetVoltage(uint8_t reg)
{
    uint16_t raw;

    raw = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        reg);

    if (raw == 0xFFFF)
    {
        return -1.0f;
    }

    return ((float)raw * 78.125e-6f);
}

/**
 * @brief Read battery current.
 */
float MAX17261_GetCurrent(uint8_t reg)
{
    int16_t raw;

    raw = (int16_t)I2C_ReadRegister(MAX17261_I2C_ADDR,reg);

    return ((float)raw *
            (1.5625e-6f / RSENSE_VALUE));
}

/**
 * @brief Read battery temperature or state of charge.
 */
float MAX17261_Temp_SOC(uint8_t reg)
{
    uint16_t raw;

    raw = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        reg);

    return ((float)raw / 256.0f);
}

/**
 * @brief Read remaining battery capacity.
 */
float MAX17261_GetRemainingCapacity(void)
{
    uint16_t raw;

    raw = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        REPCAP_REG);

    return (float)raw;
}

/**
 * @brief Read capacity register.
 */
float MAX17261_GetCapacity(uint8_t reg)
{
    uint16_t raw;

    raw = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        reg);

    return (float)raw;
}

/**
 * @brief Read device parameter register.
 */
float MAX17261_GetParameter(uint8_t reg)
{
    uint16_t raw;

    raw = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        reg);

    return (float)raw;
}



/**
 * @brief Read Time-To-Empty and Time-To-Full estimate.
 */
uint16_t MAX17261_GetTime(uint8_t reg)
{
    uint16_t raw;

    raw = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        reg);

    return (uint16_t)
           ((raw * 5.625f) / 60.0f);
}

/**
 * @brief Read maximum and minimum cell voltage.
 */
void MAX17261_GetMaxMinVoltage(
    float *maxVoltage,
    float *minVoltage)
{
    uint16_t raw;

    raw = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        MAXMINVOLT_REG);

    uint8_t maxRaw =
        (raw >> 8) & 0xFF;

    uint8_t minRaw =
        raw & 0xFF;

    *maxVoltage =
        maxRaw * 0.02f;

    *minVoltage =
        minRaw * 0.02f;
}

/**
 * @brief Read maximum and minimum current values.
 *
 * Reads the MAXMINCURR register and converts the
 * signed 8-bit maximum and minimum current fields
 * into amperes using the configured sense resistor.
 */
void MAX17261_GetMaxMinCurrent(
    float *maxCurrent,
    float *minCurrent)
{
    uint16_t raw;

    raw = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        MAXMINCURR_REG);

    int8_t maxRaw =
        (raw >> 8) & 0xFF;

    int8_t minRaw =
        raw & 0xFF;

    *maxCurrent =
        ((float)maxRaw * 0.0004f)
        / RSENSE_VALUE;

    *minCurrent =
        ((float)minRaw * 0.0004f)
        / RSENSE_VALUE;
}

/**
 * @brief Read maximum and minimum temperature values.
 *
 * Reads the MAXMINTEMP register and converts the
 * signed 8-bit maximum and minimum temperature
 * fields into degrees Celsius.
 */
void MAX17261_GetMaxMinTemperature(
    float *maxTemperature,
    float *minTemperature)
{
    uint16_t raw;

    raw = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        MAXMINTEMP_REG);

    int8_t maxRaw =
        (raw >> 8) & 0xFF;

    int8_t minRaw =
        raw & 0xFF;

    *maxTemperature =
        (float)maxRaw;

    *minTemperature =
        (float)minRaw;
}

/**
 * @brief Read VEmpty and Recovery voltage thresholds.
 */
void MAX17261_GetVEmpty(
    float *emptyVoltage,
    float *recoveryVoltage)
{
    uint16_t raw;

    raw = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        VEMPTY_REG);

    uint16_t emptyRaw =
        raw >> 7;

    uint16_t recoveryRaw =
        raw & 0x7F;

    *emptyVoltage =
        emptyRaw * 0.01f;

    *recoveryVoltage =
        recoveryRaw * 0.04f;
}

/**
 * @brief Read voltage alert thresholds.
 */
void MAX17261_GetVoltageAlertThreshold(
    float *maxVoltageAlert,
    float *minVoltageAlert)
{
    uint16_t raw;

    raw = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        VALRTTH_REG);

    uint8_t maxRaw =
        (raw >> 8) & 0xFF;

    uint8_t minRaw =
        raw & 0xFF;

    *maxVoltageAlert =
        maxRaw * 0.02f;

    *minVoltageAlert =
        minRaw * 0.02f;
}

/**
 * @brief Read temperature alert thresholds.
 */
void MAX17261_GetTemperatureAlertThreshold(
    float *maxTemperatureAlert,
    float *minTemperatureAlert)
{
    uint16_t raw;

    raw = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        TALRTTH_REG);

    int8_t maxRaw =
        (raw >> 8) & 0xFF;

    int8_t minRaw =
        raw & 0xFF;

    *maxTemperatureAlert =
        (float)maxRaw;

    *minTemperatureAlert =
        (float)minRaw;
}

/**
 * @brief Read SOC alert thresholds.
 */
void MAX17261_GetSOCAlertThreshold(
    float *maxSocAlert,
    float *minSocAlert)
{
    uint16_t raw;

    raw = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        SALRTTH_REG);

    uint8_t maxRaw =
        (raw >> 8) & 0xFF;

    uint8_t minRaw =
        raw & 0xFF;

    *maxSocAlert =
        (float)maxRaw;

    *minSocAlert =
        (float)minRaw;
}

/**
 * @brief Read current alert thresholds.
 */
void MAX17261_GetCurrentAlertThreshold(
    float *maxCurrentAlert,
    float *minCurrentAlert)
{
    uint16_t raw;

    raw = I2C_ReadRegister(
        MAX17261_I2C_ADDR,
        IALRTTH_REG);

    int8_t maxRaw =
        (raw >> 8) & 0xFF;

    int8_t minRaw =
        raw & 0xFF;

    *maxCurrentAlert =
        (float)maxRaw;

    *minCurrentAlert =
        (float)minRaw;
}
