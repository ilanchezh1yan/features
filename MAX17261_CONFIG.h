/**
 * @file MAX17261_Config.h
 * @brief Configuration settings for MAX17261 Fuel Gauge Driver.
 *
 * Contains user-configurable parameters including:
 * - Register group enable/disable selections
 * - Battery configuration options
 * - Driver feature settings
 * - Debug options
 *
 * This file allows application-specific customization
 * without modifying the driver source code.
 */
#ifndef MAX17261_CONFIG_H
#define MAX17261_CONFIG_H


/******************************************************************************
 * Voltage Registers
 *****************************************************************************/

#define ENABLE_VCELL_REG             0
#define ENABLE_AVGVCELL_REG          0
#define ENABLE_MAXMINVOLT_REG        0
#define ENABLE_MINSYSVOLTAGE_REG     0
#define ENABLE_VRIPPLE_REG           0

/******************************************************************************
 * Current Registers
 *****************************************************************************/

#define ENABLE_CURRENT_REG           0
#define ENABLE_AVGCURRENT_REG        0
#define ENABLE_MAXMINCURR_REG        0
#define ENABLE_MPPCURRENT_REG        0
#define ENABLE_SPPCURRENT_REG        0

/******************************************************************************
 * Temperature Registers
 *****************************************************************************/

#define ENABLE_TEMP_REG              0
#define ENABLE_DIETEMP_REG           0
#define ENABLE_AVGTA_REG             0
#define ENABLE_MAXMINTEMP_REG        0

/******************************************************************************
 * State of Charge Registers
 *****************************************************************************/

#define ENABLE_REPSOC_REG            0
#define ENABLE_MIXSOC_REG            0
#define ENABLE_AVSOC_REG             0
#define ENABLE_AGE_REG               0

/******************************************************************************
 * Capacity Registers
 *****************************************************************************/

#define ENABLE_REPCAP_REG            0
#define ENABLE_FULLCAPREP_REG        0
#define ENABLE_FULLCAP_REG           0
#define ENABLE_FULLCAPNOM_REG        0
#define ENABLE_AVCAP_REG             0

/******************************************************************************
 * Power Registers
 *****************************************************************************/

#define ENABLE_POWER_REG             0
#define ENABLE_AVGPOWER_REG          0
#define ENABLE_MAXPEAKPOWER_REG      0
#define ENABLE_SUSPEAKPOWER_REG      0

/******************************************************************************
 * Time Estimation Registers
 *****************************************************************************/

#define ENABLE_TTE_REG               0
#define ENABLE_TTF_REG               0

/******************************************************************************
 * Alert Registers
 *****************************************************************************/

#define ENABLE_STATUS_REG            0
#define ENABLE_STATUS2_REG           0
#define ENABLE_VALRTTH_REG           0
#define ENABLE_TALRTTH_REG           0
#define ENABLE_SALRTTH_REG           0
#define ENABLE_IALRTTH_REG           0
#define ENABLE_FSTAT_REG             0

/******************************************************************************
 * Configuration Registers
 *****************************************************************************/

#define ENABLE_CONFIG_REG            0
#define ENABLE_CONFIG2_REG           0
#define ENABLE_MODELCFG_REG          0
#define ENABLE_LEARNCFG_REG          0
#define ENABLE_FILTERCFG_REG         0
#define ENABLE_RELAXCFG_REG          0
#define ENABLE_MISCCFG_REG           0
#define ENABLE_HIBCFG_REG            0
#define ENABLE_CONVGCFG_REG          0
#define ENABLE_RIPPLECFG_REG         0

/******************************************************************************
 * Calibration Registers
 *****************************************************************************/

#define ENABLE_RCOMP0_REG            0
#define ENABLE_TEMPCO_REG            0
#define ENABLE_TGAIN_REG             0
#define ENABLE_TOFF_REG              0
#define ENABLE_CGAIN_REG             0
#define ENABLE_COFF_REG              0
#define ENABLE_VGAIN_REG             0

/******************************************************************************
 * Device Parameters
 *****************************************************************************/

#define ENABLE_DESIGNCAP_REG         0
#define ENABLE_ICHGTERM_REG          0
#define ENABLE_VEMPTY_REG            0
#define ENABLE_RSENSE_USERMEM3_REG   0
#define ENABLE_PACKRESISTANCE_REG    0
#define ENABLE_SYSRESISTANCE_REG     0

/******************************************************************************
 * Debug Options
 *****************************************************************************/

#define MAX17261_DEBUG_ENABLE        1


#define ENABLE_CYCLES_REG            0

#endif /* MAX17261_CONFIG_H */