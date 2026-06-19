/**
 * @file MAX17261.h
 * @brief Register definitions and driver interface for MAX17261 Fuel Gauge.
 *
 * Contains:
 * - Register address definitions
 * - Register bit-field definitions
 * - Device constants
 * - Public API function prototypes
 *
 * This file serves as the primary interface between the
 * application layer and the MAX17261 fuel gauge driver.
 */
#ifndef MAX17261_H
#define MAX17261_H

#include <Arduino.h>

/** MAX17261 7-bit I2C slave address */
#define MAX17261_I2C_ADDR    0x36

#define STATUS_REG      0x00 /**< Address of the status register. */

#define STATUS_BR        (1U << 15) /**< Battery removal alert flag. */
#define STATUS_SMX       (1U << 14) /**< Maximum SOC threshold exceeded. */
#define STATUS_TMX       (1U << 13) /**< Maximum temperature threshold exceeded. */
#define STATUS_VMX       (1U << 12) /**< Maximum voltage threshold exceeded. */
#define STATUS_BI        (1U << 11) /**< Battery insertion alert flag. */
#define STATUS_SMN       (1U << 10) /**< Minimum SOC threshold exceeded. */
#define STATUS_TMN       (1U << 9)  /**< Minimum temperature threshold exceeded. */
#define STATUS_VMN       (1U << 8)  /**< Minimum voltage threshold exceeded. */
#define STATUS_DSOCI     (1U << 7)  /**< Delta-SOC alert interrupt flag. */
#define STATUS_IMX       (1U << 6)  /**< Maximum current threshold exceeded. */
#define STATUS_BST       (1U << 3)  /**< Battery status change flag. */
#define STATUS_IMN       (1U << 2)  /**< Minimum current threshold exceeded. */
#define STATUS_POR       (1U << 1)  /**< Power-on reset detected. */


#define VALRTTH_REG     0x01 /**< Address of the voltage alert threshold register. */
#define TALRTTH_REG     0x02 /**< Address of the temperature alert threshold register. */
#define SALRTTH_REG     0x03 /**< Address of the state-of-charge alert threshold register. */
#define ATRATE_REG      0x04 /**< Address of the AtRate register used for fuel gauge calculations. */
#define REPCAP_REG      0x05 /**< Address of the reported remaining capacity register. */
#define REPSOC_REG      0x06 /**< Address of the reported state-of-charge register. */
#define AGE_REG         0x07 /**< Address of the battery age estimation register. */
#define TEMP_REG        0x08 /**< Address of the battery temperature register. */
#define VCELL_REG       0x09 /**< Address of the cell voltage measurement register. */
#define CURRENT_REG     0x0A /**< Address of the battery current measurement register. */
#define AVGCURRENT_REG  0x0B /**< Address of the average current measurement register. */
#define QRESIDUAL_REG   0x0C /**< Address of the residual capacity estimation register. */
#define MIXSOC_REG      0x0D /**< Address of the mixed state-of-charge register. */
#define AVSOC_REG       0x0E /**< Address of the filtered average state-of-charge register. */
#define MIXCAP_REG      0x0F /**< Address of the mixed remaining capacity register. */


#define FULLCAPREP_REG   0x10 /**< Address of the reported full capacity register. */
#define TTE_REG          0x11 /**< Address of the time-to-empty estimation register. */
#define QRTABLE00_REG    0x12 /**< Address of the QRTable00 characterization register. */
#define FULLSOCTHR_REG   0x13 /**< Address of the full state-of-charge threshold register. */
#define RCELL_REG        0x14 /**< Address of the cell resistance estimation register. */
#define RESERVED15_REG   0x15 /**< Reserved register address. */
#define AVGTA_REG        0x16 /**< Address of the average temperature register. */
#define CYCLES_REG       0x17 /**< Address of the battery cycle count register. */
#define DESIGNCAP_REG    0x18 /**< Address of the design capacity register. */
#define AVGVCELL_REG     0x19 /**< Address of the average cell voltage register. */
#define MAXMINTEMP_REG   0x1A /**< Address of the maximum and minimum temperature register. */
#define MAXMINVOLT_REG   0x1B /**< Address of the maximum and minimum voltage register. */
#define MAXMINCURR_REG   0x1C /**< Address of the maximum and minimum current register. */

#define CONFIG_REG       0x1D /**< Address of the configuration register. */
/******************************************************************************
 * CONFIG Register Bit Definitions
 *****************************************************************************/
#define CONFIG_BER          (1U << 15) /**< Battery removal alert enable. */
#define CONFIG_BEI          (1U << 14) /**< Battery insertion alert enable. */
#define CONFIG_AEN          (1U << 13) /**< Alert output enable. */
#define CONFIG_FTHRM        (1U << 12) /**< Force thermistor bias switch enable. */
#define CONFIG_ETHRM        (1U << 11) /**< Thermistor measurement enable. */
#define CONFIG_COMMSH       (1U << 10) /**< Communication shutdown enable. */
#define CONFIG_TEX          (1U << 9)  /**< Temperature external source selection. */
#define CONFIG_SHDN         (1U << 8)  /**< Shutdown mode control. */
#define CONFIG_TSEL         (1U << 7)  /**< Temperature source selection. */
#define CONFIG_VS           (1U << 6)  /**< Voltage alert sticky mode. */
#define CONFIG_TS           (1U << 5)  /**< Temperature alert sticky mode. */
#define CONFIG_SS           (1U << 4)  /**< SOC alert sticky mode. */
#define CONFIG_IS           (1U << 3)  /**< Current alert sticky mode. */
#define CONFIG_POR_CMD      (1U << 1)  /**< Software power-on reset command bit. */


#define ICHGTERM_REG     0x1E /**< Address of the charge termination current register. */
#define AVCAP_REG        0x1F /**< Address of the average capacity register. */


#define TTF_REG          0x20 /**< Address of the time-to-full estimation register. */
#define DEVNAME_REG      0x21 /**< Address of the device name register. */
#define QRTABLE10_REG    0x22 /**< Address of the QRTable10 characterization register. */
#define FULLCAPNOM_REG   0x23 /**< Address of the nominal full capacity register. */
#define RESERVED24_REG   0x24 /**< Reserved register address. */
#define RESERVED25_REG   0x25 /**< Reserved register address. */
#define RESERVED26_REG   0x26 /**< Reserved register address. */
#define AIN_REG          0x27 /**< Address of the auxiliary analog input register. */

#define LEARNCFG_REG     0x28 /**< Address of the learning configuration register. */
/******************************************************************************
 * LEARNCFG Register Bit Definitions
 *****************************************************************************/
/* Bits [15:13] */
#define LEARNCFG_FCLM_MASK         (0x07U << 13)
#define LEARNCFG_FCLM_SHIFT        13
/**< Full-capacity learning mode. */
/* Bits [12:8] */
#define LEARNCFG_LEARNT_MASK       (0x1FU << 8)
#define LEARNCFG_LEARNT_SHIFT      8
/**< Learning rate configuration. */
/* Bits [7:0] */
#define LEARNCFG_FILTER_MASK       (0xFFU)
#define LEARNCFG_FILTER_SHIFT      0
/**< Learning filter coefficient. */


#define FILTERCFG_REG    0x29 /**< Address of the filter configuration register. */
/******************************************************************************
 * FILTERCFG Register Bit Definitions
 *****************************************************************************/
/* Bits [15:12] */
#define FILTERCFG_NCURR_MASK        (0x0FU << 12)
#define FILTERCFG_NCURR_SHIFT       12
/**< Current filter constant. */
/* Bits [11:8] */
#define FILTERCFG_AVGVCELL_MASK     (0x0FU << 8)
#define FILTERCFG_AVGVCELL_SHIFT    8
/**< Average cell voltage filter constant. */
/* Bits [7:4] */
#define FILTERCFG_MIXCAP_MASK       (0x0FU << 4)
#define FILTERCFG_MIXCAP_SHIFT      4
/**< Mixed-capacity filter constant. */
/* Bits [3:0] */
#define FILTERCFG_MIXSOC_MASK       (0x0FU)
#define FILTERCFG_MIXSOC_SHIFT      0
/**< Mixed-SOC filter constant. */


#define RELAXCFG_REG     0x2A /**< Address of the relaxation configuration register. */
/******************************************************************************
 * RELAXCFG Register Bit Definitions
 *****************************************************************************/
/* Bits [15:8] */
#define RELAXCFG_LOAD_MASK         (0xFFU << 8)
#define RELAXCFG_LOAD_SHIFT        8
/**< Load threshold used for relaxation detection. */
/* Bits [7:0] */
#define RELAXCFG_DV_MASK           (0xFFU)
#define RELAXCFG_DV_SHIFT          0
/**< Voltage-change threshold used for relaxation detection. */


#define MISCCFG_REG      0x2B /**< Address of the miscellaneous configuration register. */
/******************************************************************************
 * MISCCFG Register Bit Definitions
 *****************************************************************************/
#define MISCCFG_NTCCFG_MASK        (0x07U << 13)
#define MISCCFG_NTCCFG_SHIFT       13
/**< Thermistor configuration selection. */
#define MISCCFG_ENBIAS             (1U << 12)
/**< Enable thermistor bias. */
#define MISCCFG_TGAIN_MASK         (0x0FFFU)
/**< Temperature gain adjustment field. */


#define TGAIN_REG        0x2C /**< Address of the temperature gain calibration register. */
#define TOFF_REG         0x2D /**< Address of the temperature offset calibration register. */
#define CGAIN_REG        0x2E /**< Address of the current gain calibration register. */
#define COFF_REG         0x2F /**< Address of the current offset calibration register. */


#define RESERVED30_REG    0x30 /**< Reserved register address. */
#define RESERVED31_REG    0x31 /**< Reserved register address. */
#define QRTABLE20_REG     0x32 /**< Address of the QRTable20 characterization register. */
#define RESERVED33_REG    0x33 /**< Reserved register address. */
#define DIETEMP_REG       0x34 /**< Address of the IC die temperature register. */
#define FULLCAP_REG       0x35 /**< Address of the learned full capacity register. */
#define RESERVED36_REG    0x36 /**< Reserved register address. */
#define RESERVED37_REG    0x37 /**< Reserved register address. */
#define RCOMP0_REG        0x38 /**< Address of the temperature compensation parameter register. */
#define TEMPCO_REG        0x39 /**< Address of the temperature coefficient register. */
#define VEMPTY_REG        0x3A /**< Address of the empty voltage threshold register. */
#define RESERVED3B_REG    0x3B /**< Reserved register address. */
#define RESERVED3C_REG    0x3C /**< Reserved register address. */

#define FSTAT_REG         0x3D /**< Address of the fuel gauge status register. */
/******************************************************************************
 * FSTAT Register Bit Definitions
 *****************************************************************************/
#define FSTAT_DNR        (1U << 0) /**< Data Not Ready. Fuel-gauge calculations are not yet complete. */
#define FSTAT_EDet       (1U << 1) /**< Model gauge data update event detected. */
#define FSTAT_FQ         (1U << 7) /**< Full qualification status flag. */


#define TIMER_REG         0x3E /**< Address of the timer register. */
#define SHDNTIMER_REG     0x3F /**< Address of the shutdown timer register. */


#define RESERVED40_REG    0x40 /**< Reserved register address. */
#define RESERVED41_REG    0x41 /**< Reserved register address. */
#define QRTABLE30_REG     0x42 /**< Address of the QRTable30 characterization register. */
#define RGAIN_REG         0x43 /**< Address of the voltage gain calibration register. */
#define RESERVED44_REG    0x44 /**< Reserved register address. */
#define DQACC_REG         0x45 /**< Address of the accumulated discharged capacity register. */
#define DPACC_REG         0x46 /**< Address of the accumulated discharged power register. */
#define RESERVED47_REG    0x47 /**< Reserved register address. */
#define RESERVED48_REG    0x48 /**< Reserved register address. */
#define CONVGCFG_REG      0x49 /**< Address of the convergence configuration register. */
#define VFREMCAP_REG      0x4A /**< Address of the voltage-filtered remaining capacity register. */
#define RESERVED4B_REG    0x4B /**< Reserved register address. */
#define RESERVED4C_REG    0x4C /**< Reserved register address. */
#define QH_REG            0x4D /**< Address of the accumulated charge register. */
#define RESERVED4E_REG    0x4E /**< Reserved register address. */
#define RESERVED4F_REG    0x4F /**< Reserved register address. */


#define STATUS2_REG      0xB0 /**< Address of the secondary status register. */
/******************************************************************************
 * STATUS2 Register Bit Definitions
 *****************************************************************************/
#define STATUS2_ATRATE_READY    (1U << 13)
/**< AtRate output registers contain valid data. */
#define STATUS2_DP_READY        (1U << 12)
/**< Dynamic Power output registers contain valid data. */
#define STATUS2_SN_READY        (1U << 8)
/**< Serial number available over I2C. */
#define STATUS2_FULLDET         (1U << 5)
/**< Full battery detected. */
#define STATUS2_HIB            (1U << 1)
/**< Hibernate mode status.
     1 = Hibernate mode
     0 = Active mode
*/


#define POWER_REG        0xB1 /**< Address of the instantaneous power measurement register. */
#define ID_USERMEM2_REG  0xB2 /**< Address of the User Memory 2 register. */
#define AVGPOWER_REG     0xB3 /**< Address of the average power measurement register. */
#define IALRTTH_REG      0xB4 /**< Address of the current alert threshold register. */
#define TTFCFG_REG       0xB5 /**< Address of the time-to-full configuration register. */
#define CVMIXCAP_REG     0xB6 /**< Address of the constant-voltage mixed capacity register. */
#define CVHALFTIME_REG   0xB7 /**< Address of the constant-voltage half-time configuration register. */
#define CGTEMPCO_REG     0xB8 /**< Address of the charge temperature coefficient register. */
#define CURVE_REG        0xB9 /**< Address of the battery characterization curve register. */

#define HIBCFG_REG       0xBA /**< Address of the hibernate configuration register. */
/******************************************************************************
 * HIBCFG Register Bit Definitions
 *****************************************************************************/
#define HIBCFG_ENHIB              (1U << 15)
/**< Enable hibernate mode. */
/* Bits [14:12] */
#define HIBCFG_ENTERTIME_MASK     (0x07U << 12)
#define HIBCFG_ENTERTIME_SHIFT    12
/**< Hibernate entry delay time. */
/* Bits [11:8] */
#define HIBCFG_THRESHOLD_MASK     (0x0FU << 8)
#define HIBCFG_THRESHOLD_SHIFT    8
/**< Hibernate current threshold. */
/* Bits [4:3] */
#define HIBCFG_EXITTIME_MASK      (0x03U << 3)
#define HIBCFG_EXITTIME_SHIFT     3
/**< Hibernate exit delay time. */
/* Bits [2:0] */
#define HIBCFG_SCALAR_MASK        (0x07U)
#define HIBCFG_SCALAR_SHIFT       0
/**< Hibernate task period scaler. */


#define CONFIG2_REG      0xBB /**< Address of the secondary configuration register. */
/******************************************************************************
 * CONFIG2 Register Bit Definitions
 *****************************************************************************/
#define CONFIG2_ATRATEEN      (1U << 13)
/**< Enable AtRate calculations. When cleared, AtQResidual/AtTTE/
     AtAvSOC/AtAvCap become general-purpose memory. */
#define CONFIG2_DPEN          (1U << 12)
/**< Enable Dynamic Power calculations. When cleared,
     MaxPeakPower/SusPeakPower/MPPCurrent/SPPCurrent become
     general-purpose memory. */
#define CONFIG2_POWR_MASK     (0x0FU << 8)
#define CONFIG2_POWR_SHIFT    8
/**< AvgPower time constant selection. */
#define CONFIG2_DSOCEN        (1U << 7)
/**< Enable 1% SOC change alert output. */
#define CONFIG2_TALRTEN       (1U << 6)
/**< Enable temperature alerts. */
#define CONFIG2_LDMDL         (1U << 5)
/**< Host sets to 1 after loading a model. Firmware clears when complete. */
#define CONFIG2_DRCFG_MASK    (0x03U << 2)
#define CONFIG2_DRCFG_SHIFT   2
/**< Deep-relax timing configuration. */
#define CONFIG2_CP_MODE       (1U << 1)
/**< Constant-power mode enable. */


#define VRIPPLE_REG      0xBC /**< Address of the voltage ripple measurement register. */

#define RIPPLECFG_REG    0xBD /**< Address of the ripple compensation configuration register. */
/******************************************************************************
 * RIPPLECFG Register Bit Definitions
 *****************************************************************************/
#define RIPPLECFG_KDV_MASK      (0xFFU << 8)
#define RIPPLECFG_KDV_SHIFT     8
/**< Ripple empty compensation coefficient. */
#define RIPPLECFG_NR_MASK       (0xFFU)
#define RIPPLECFG_NR_SHIFT      0
/**< Ripple measurement filter coefficient. */


#define TIMERH_REG       0xBE /**< Address of the high-resolution timer register. */


#define RSENSE_USERMEM3_REG   0xD0 /**< Address of the RSENSE/User Memory 3 register. */
#define SCOCVLIM_REG          0xD1 /**< Address of the SOC open-circuit voltage limit register. */
#define VGAIN_REG             0xD2 /**< Address of the voltage gain calibration register. */

#define SOCHOLD_REG           0xD3 /**< Address of the state-of-charge hold configuration register. */
/******************************************************************************
 * SOCHOLD Register Bit Definitions
 *****************************************************************************/
/* Bits [15:14] */
#define SOCHOLD_HOLDEN_MASK      (0x03U << 14)
#define SOCHOLD_HOLDEN_SHIFT     14
/**< SOC hold enable configuration. */
/* Bits [13:8] */
#define SOCHOLD_MAXHOLD_MASK     (0x3FU << 8)
#define SOCHOLD_MAXHOLD_SHIFT    8
/**< Maximum SOC hold time configuration. */
/* Bits [7:0] */
#define SOCHOLD_EMPTYHOLD_MASK   (0xFFU)
#define SOCHOLD_EMPTYHOLD_SHIFT  0
/**< Empty SOC hold threshold configuration. */


#define MAXPEAKPOWER_REG      0xD4 /**< Address of the maximum peak power register. */
#define SUSPEAKPOWER_REG      0xD5 /**< Address of the sustained peak power register. */
#define PACKRESISTANCE_REG    0xD6 /**< Address of the battery pack resistance register. */
#define SYSRESISTANCE_REG     0xD7 /**< Address of the system resistance register. */
#define MINSYSVOLTAGE_REG     0xD8 /**< Address of the minimum system voltage register. */
#define MPPCURRENT_REG        0xD9 /**< Address of the maximum peak power current register. */
#define SPPCURRENT_REG        0xDA /**< Address of the sustained peak power current register. */

#define MODELCFG_REG          0xDB /**< Address of the model configuration register. */
/******************************************************************************
 * MODELCFG Register Bit Definitions
 *****************************************************************************/
/* Bit 15 */
#define MODELCFG_REFRESH      (1U << 15)
/**< Reload battery model. Hardware clears this bit automatically after execution. */
/* Bit 13 */
#define MODELCFG_R100         (1U << 13)
/**< Thermistor selection:
 * 0 = 10kΩ NTC
 * 1 = 100kΩ NTC
 */
/* Bit 10 */
#define MODELCFG_VCHG         (1U << 10)
/**< Charge voltage selection:
 * 0 = 4.2V charger
 * 1 = 4.3V–4.4V charger
 */
/* Bits [7:4] */
#define MODELCFG_MODELID_MASK     (0x0F << 4)
#define MODELCFG_MODELID_SHIFT    4
/* Common Model IDs */
#define MODELCFG_MODELID_LICOO2   (0x0 << 4)
/**< Standard lithium cobalt chemistry (recommended default). */
#define MODELCFG_MODELID_NCA      (0x2 << 4)
/**< NCR / NCA chemistry (Panasonic type cells). */
#define MODELCFG_MODELID_LIFEPO4  (0x6 << 4)
/**< Lithium Iron Phosphate chemistry. */
#define MODELCFG_MODELID_NMC      (0x1 << 4)
#define MODELCFG_MODELID_NCA      (0x2 << 4)
#define MODELCFG_MODELID_LIFEPO4  (0x6 << 4)


#define ATQRESIDUAL_REG       0xDC /**< Address of the AtQResidual register used for AtRate calculations. */
#define ATTTE_REG             0xDD /**< Address of the AtRate time-to-empty register. */
#define ATAVSOC_REG           0xDE /**< Address of the AtRate average state-of-charge register. */
#define ATAVCAP_REG           0xDF /**< Address of the AtRate average capacity register. */


/**
 * @brief Initialize MAX17261 fuel gauge.
 */
void MAX17261_Init(void);

/**
 * @brief Read battery voltage.
 *
 * @return Voltage in volts.
 */
float MAX17261_GetVoltage(uint8_t reg);

/**
 * @brief Read battery current.
 *
 * @return Current in amperes.
 */
float MAX17261_GetCurrent(uint8_t reg);

/**
 * @brief Read battery temperature or State of charge.
 *
 * @return Temperature in degrees Celsius.
 * @return State of charge in percentage.
 */
float MAX17261_Temp_SOC(uint8_t reg);

/**
 * @brief Read remaining battery capacity.
 *
 * @return Capacity in mAh.
 */
float MAX17261_GetRemainingCapacity(void);

/**
 * @brief Read capacity register.
 *
 * @param reg Capacity register address.
 *
 * @return Capacity in mAh.
 */
float MAX17261_GetCapacity(uint8_t reg);

/**
 * @brief Read device parameter register.
 *
 * @param reg Register address.
 *
 * @return Register value.
 */
float MAX17261_GetParameter(uint8_t reg);


/**
 * @brief Read estimated Time-To-Empty and Time-To-Full.
 *
 * @return Time in minutes.
 */
uint16_t MAX17261_GetTime(uint8_t reg);

/**
 * @brief Read maximum and minimum cell voltage.
 *
 * @param maxVoltage Maximum voltage in volts.
 * @param minVoltage Minimum voltage in volts.
 */
void MAX17261_GetMaxMinVoltage(
    float *maxVoltage,
    float *minVoltage);

/**
 * @brief Read maximum and minimum current values.
 *
 * Reads the MAXMINCURR register and returns the
 * maximum and minimum current recorded since the
 * last reset of the register.
 *
 * @param maxCurrent Maximum current in amperes.
 * @param minCurrent Minimum current in amperes.
 */
void MAX17261_GetMaxMinCurrent(
    float *maxCurrent,
    float *minCurrent);

/**
 * @brief Clear Power-On-Reset flag.
 */
void MAX17261_ClearPOR(void);

/**
 * @brief Read voltage alert thresholds.
 *
 * @param maxVoltageAlert Upper threshold in volts.
 * @param minVoltageAlert Lower threshold in volts.
 */
void MAX17261_GetVoltageAlertThreshold(
    float *maxVoltageAlert,
    float *minVoltageAlert);

/**
 * @brief Read temperature alert thresholds.
 *
 * @param maxTemperatureAlert Upper threshold in °C.
 * @param minTemperatureAlert Lower threshold in °C.
 */
void MAX17261_GetTemperatureAlertThreshold(
    float *maxTemperatureAlert,
    float *minTemperatureAlert);

/**
 * @brief Read SOC alert thresholds.
 *
 * @param maxSocAlert Upper threshold in %.
 * @param minSocAlert Lower threshold in %.
 */
void MAX17261_GetSOCAlertThreshold(
    float *maxSocAlert,
    float *minSocAlert);

/**
 * @brief Read current alert thresholds.
 *
 * @param maxCurrentAlert Upper threshold.
 * @param minCurrentAlert Lower threshold.
 */
void MAX17261_GetCurrentAlertThreshold(
    float *maxCurrentAlert,
    float *minCurrentAlert);

/**
 * @brief Read maximum and minimum temperature values.
 *
 * Reads the MAXMINTEMP register and returns the
 * maximum and minimum temperatures recorded since
 * the last reset of the register.
 *
 * @param maxTemperature Maximum temperature in °C.
 * @param minTemperature Minimum temperature in °C.
 */
void MAX17261_GetMaxMinTemperature(
    float *maxTemperature,
    float *minTemperature);

/**
 * @brief Read VEmpty and Recovery voltage thresholds.
 *
 * @param emptyVoltage Empty voltage threshold in volts.
 * @param recoveryVoltage Recovery voltage threshold in volts.
 */
void MAX17261_GetVEmpty(
    float *emptyVoltage,
    float *recoveryVoltage);

#endif /* MAX17261_H */