/**
 * @file main.cpp
 *
 * @brief Application entry point for MAX17261 Fuel Gauge Demo.
 */

#include <Arduino.h>
#include "I2C.h"
#include "MAX17261.h"
#include "MAX17261_CONFIG.h"

const uint8_t reservedRegs[] =
{
    // Table 10 reserved
    0x15,0x24,0x25,0x26,0x30,0x31,0x33,0x36,0x37,0x3B,0x3C,0x40,0x41,0x44,0x47,0x48,0x4B,0x4C,0x4E,0x4F,

    // Undocumented E-page
    0xE0,0xE1,0xE2,0xE3, 0xE4, 0xE5, 0xE6,0xE7,0xE8,0xE9,0xEA,0xEB,0xEC,0xED, 0xEE,0xEF,

    // Undocumented F-page
    0xF0,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7, 0xF8,0xF9,0xFA,0xFC,0xFD,0xFE
};

void MAX17261_ReadReservedRegister(uint8_t address)
{
    uint16_t value =
        I2C_ReadRegister(
            MAX17261_I2C_ADDR,
            address);

    Serial.print("REG 0x");

    if (address < 0x10)
        Serial.print("0");

    Serial.print(address, HEX);

    Serial.print(" : 0x");

    Serial.println(value, HEX);
}

void setup()
{
    Serial.begin(9600);
    delay(1000);
    Serial.println();
    Serial.println("=================================");
    Serial.println("MAX17261 Fuel Gauge Demo");
    Serial.println("=================================");
    I2C_Init();
    Serial.println("I2C Initialized");
    Serial.print("Device Address = 0x");
    Serial.println(MAX17261_I2C_ADDR, HEX);
    MAX17261_Init();
    Serial.println("MAX17261 Initialized");
}

void loop()
{
    float maxVoltage;
    float minVoltage;
    float maxCurrent;
    float minCurrent;
    float maxVoltageAlert;
    float minVoltageAlert;
    float maxTempAlert;
    float minTempAlert;
    float maxSocAlert;
    float minSocAlert;
    float maxCurrentAlert;
    float minCurrentAlert;
    float maxTemperature;
    float minTemperature;   
    float emptyVoltage;
    float recoveryVoltage;
    
    float voltage = MAX17261_GetVoltage(VCELL_REG);
    float avgVoltage = MAX17261_GetVoltage(AVGVCELL_REG);
    float minSysVoltage = MAX17261_GetVoltage(MINSYSVOLTAGE_REG);
    MAX17261_GetMaxMinVoltage(&maxVoltage,&minVoltage);
    MAX17261_GetMaxMinCurrent(&maxCurrent,&minCurrent);
    MAX17261_GetMaxMinTemperature(&maxTemperature,&minTemperature);
    uint16_t rawMaxMinTemp = I2C_ReadRegister(MAX17261_I2C_ADDR,MAXMINTEMP_REG);
    float vRipple = MAX17261_GetVoltage(VRIPPLE_REG);
    uint16_t rawVRipple = I2C_ReadRegister(MAX17261_I2C_ADDR,VRIPPLE_REG);
    float current = MAX17261_GetCurrent(CURRENT_REG);
    float avgCurrent = MAX17261_GetCurrent(AVGCURRENT_REG);
    float mppCurrent =MAX17261_GetCurrent(MPPCURRENT_REG);
    float sppCurrent =MAX17261_GetCurrent(SPPCURRENT_REG);
    float temperature = MAX17261_Temp_SOC(TEMP_REG);
    float avgTemperature =MAX17261_Temp_SOC(AVGTA_REG);
    float dieTemperature = MAX17261_Temp_SOC(DIETEMP_REG);
    float soc         = MAX17261_Temp_SOC(REPSOC_REG);
    float mixSoc = MAX17261_Temp_SOC(MIXSOC_REG);
    float avSoc = MAX17261_Temp_SOC(AVSOC_REG);
    float age = MAX17261_Temp_SOC(AGE_REG);
    float capacity    = MAX17261_GetRemainingCapacity();
    float avgCapacity = MAX17261_GetCapacity(AVCAP_REG);
    float fullCapRep = MAX17261_GetCapacity(FULLCAPREP_REG);
    float fullCap = MAX17261_GetCapacity(FULLCAP_REG);
    float fullCapNom = MAX17261_GetCapacity(FULLCAPNOM_REG);
    uint16_t powerRaw = I2C_ReadRegister(MAX17261_I2C_ADDR,POWER_REG);
    uint16_t avgPowerRaw = I2C_ReadRegister(MAX17261_I2C_ADDR,AVGPOWER_REG);
    uint16_t maxPeakPowerRaw = I2C_ReadRegister(MAX17261_I2C_ADDR,MAXPEAKPOWER_REG);
    uint16_t susPeakPowerRaw = I2C_ReadRegister(MAX17261_I2C_ADDR,SUSPEAKPOWER_REG);
    uint16_t tte = MAX17261_GetTime(TTE_REG);
    uint16_t ttf = MAX17261_GetTime(TTF_REG);
    float designCap = MAX17261_GetParameter(DESIGNCAP_REG);
    float ichgTerm = MAX17261_GetParameter(ICHGTERM_REG);
    MAX17261_GetVEmpty(&emptyVoltage,&recoveryVoltage);
    uint16_t rawVEmpty = I2C_ReadRegister(MAX17261_I2C_ADDR,VEMPTY_REG);
    float packResistance = MAX17261_GetParameter(PACKRESISTANCE_REG);
    float sysResistance = MAX17261_GetParameter(SYSRESISTANCE_REG);
    float rsenseUserMem = MAX17261_GetParameter(RSENSE_USERMEM3_REG);
    float cycles = MAX17261_GetParameter(CYCLES_REG);
    MAX17261_GetVoltageAlertThreshold(&maxVoltageAlert,&minVoltageAlert);
    MAX17261_GetTemperatureAlertThreshold(&maxTempAlert,&minTempAlert);
    MAX17261_GetSOCAlertThreshold(&maxSocAlert,&minSocAlert);
    MAX17261_GetCurrentAlertThreshold(&maxCurrentAlert,&minCurrentAlert);
    uint16_t status = I2C_ReadRegister(MAX17261_I2C_ADDR,STATUS_REG);
    uint16_t status2 = I2C_ReadRegister(MAX17261_I2C_ADDR,STATUS2_REG);
    uint8_t fstat = I2C_ReadRegister(MAX17261_I2C_ADDR,FSTAT_REG) & 0xFF;
    uint16_t config = I2C_ReadRegister(MAX17261_I2C_ADDR,CONFIG_REG);
    uint16_t config2 = I2C_ReadRegister(MAX17261_I2C_ADDR,CONFIG2_REG);
    uint16_t modelCfg = I2C_ReadRegister(MAX17261_I2C_ADDR,MODELCFG_REG);
    uint16_t learnCfg = I2C_ReadRegister(MAX17261_I2C_ADDR,LEARNCFG_REG);
    uint16_t filterCfg = I2C_ReadRegister(MAX17261_I2C_ADDR,FILTERCFG_REG);
    uint16_t relaxCfg = I2C_ReadRegister(MAX17261_I2C_ADDR,RELAXCFG_REG);
    uint16_t miscCfg = I2C_ReadRegister(MAX17261_I2C_ADDR,MISCCFG_REG);
    uint16_t hibCfg = I2C_ReadRegister(MAX17261_I2C_ADDR,HIBCFG_REG);
    uint16_t convgCfg = I2C_ReadRegister(MAX17261_I2C_ADDR,CONVGCFG_REG);
    uint16_t rippleCfg = I2C_ReadRegister( MAX17261_I2C_ADDR,RIPPLECFG_REG);
    uint16_t rcomp0 = I2C_ReadRegister(MAX17261_I2C_ADDR, RCOMP0_REG);
    uint16_t tempco = I2C_ReadRegister(MAX17261_I2C_ADDR, TEMPCO_REG);
    uint16_t tgain = I2C_ReadRegister(MAX17261_I2C_ADDR, TGAIN_REG);
    uint16_t toff = I2C_ReadRegister(MAX17261_I2C_ADDR, TOFF_REG);
    uint16_t cgain = I2C_ReadRegister(MAX17261_I2C_ADDR, CGAIN_REG);
    uint16_t coff = I2C_ReadRegister(MAX17261_I2C_ADDR, COFF_REG);
    uint16_t vgain = I2C_ReadRegister(MAX17261_I2C_ADDR, VGAIN_REG);
    Serial.println("---------------------------------");

    /* Processed values */

    #if ENABLE_VCELL_REG
    Serial.print("Voltage      : ");
    if (voltage < 0.0f)
        Serial.println("INVALID");
    else
    {Serial.print(voltage, 3);
        Serial.println(" V"); }
    #endif

    #if ENABLE_AVGVCELL_REG
    Serial.print("AVG Voltage  : ");
    Serial.print(avgVoltage, 3);
    Serial.println(" V");
    #endif

    #if ENABLE_MINSYSVOLTAGE_REG
    Serial.print("Min Sys Volt : ");
    Serial.print(minSysVoltage, 3);
    Serial.println(" V");
    #endif

    #if ENABLE_MAXMINVOLT_REG
    Serial.print("Max Voltage  : ");
    Serial.print(maxVoltage, 3);
    Serial.println(" V");
    Serial.print("Min Voltage  : ");
    Serial.print(minVoltage, 3);
    Serial.println(" V");
    #endif

    #if ENABLE_VRIPPLE_REG
    Serial.print("VRipple      : ");
    Serial.print(vRipple, 3);
    Serial.println(" V");
    #endif

    #if ENABLE_CURRENT_REG
    Serial.print("Current      : ");
    Serial.print(current * 1000.0f, 3);
    Serial.println(" mA");
    #endif

    #if ENABLE_AVGCURRENT_REG
    Serial.print("Avg Current  : ");
    Serial.print(avgCurrent * 1000.0f, 3);
    Serial.println(" mA");
    #endif

    #if ENABLE_MPPCURRENT_REG
    Serial.print("MPP Current  : ");
    Serial.print(mppCurrent * 1000.0f, 3);
    Serial.println(" mA");
    #endif

    #if ENABLE_SPPCURRENT_REG
    Serial.print("SPP Current  : ");
    Serial.print(sppCurrent * 1000.0f, 3);
    Serial.println(" mA");
    #endif

    #if ENABLE_TEMP_REG
    Serial.print("Temperature  : ");
    Serial.print(temperature, 2);
    Serial.println(" C");
    #endif

    #if ENABLE_AVGTA_REG
    Serial.print("Avg Temp     : ");
    Serial.print(avgTemperature, 2);
    Serial.println(" C");
    #endif

    #if ENABLE_DIETEMP_REG
    Serial.print("Die Temp     : ");
    Serial.print(dieTemperature, 2);
    Serial.println(" C");
    #endif

    #if ENABLE_REPSOC_REG
    Serial.print("SOC          : ");
    Serial.print(soc, 2);
    Serial.println(" %");
    #endif

    #if ENABLE_MIXSOC_REG
    Serial.print("Mix SOC      : ");
    Serial.print(mixSoc, 2);
    Serial.println(" %");
    #endif

    #if ENABLE_AVSOC_REG
    Serial.print("Avg SOC      : ");
    Serial.print(avSoc, 2);
    Serial.println(" %");
    #endif

    #if ENABLE_AGE_REG
    Serial.print("Age          : ");
    Serial.print(age, 2);
    Serial.println(" %");
    #endif

    #if ENABLE_REPCAP_REG
    Serial.print("Capacity     : ");
    Serial.print(capacity, 0);
    Serial.println(" mAh");
    #endif

    #if ENABLE_AVCAP_REG
    Serial.print("Avg Capacity : ");
    Serial.print(avgCapacity, 0);
    Serial.println(" mAh");
    #endif

    #if ENABLE_FULLCAPREP_REG
    Serial.print("FullCapRep   : ");
    Serial.print(fullCapRep, 0);
    Serial.println(" mAh");
    #endif

    #if ENABLE_FULLCAP_REG
    Serial.print("FullCap      : ");
    Serial.print(fullCap, 0);
    Serial.println(" mAh");
    #endif

    #if ENABLE_FULLCAPNOM_REG
    Serial.print("FullCapNom   : ");
    Serial.print(fullCapNom, 0);
    Serial.println(" mAh");
    #endif

    #if ENABLE_POWER_REG
    Serial.print("POWER RAW    : 0x");
    Serial.println(powerRaw, HEX);
    #endif

    #if ENABLE_AVGPOWER_REG
    Serial.print("AVGPOWER RAW : 0x");
    Serial.println(avgPowerRaw, HEX);
    #endif

    #if ENABLE_MAXPEAKPOWER_REG
    Serial.print("MaxPeakPower : 0x");
    Serial.println(maxPeakPowerRaw, HEX);
    #endif

    #if ENABLE_SUSPEAKPOWER_REG
    Serial.print("SusPeakPower : 0x");
    Serial.println(susPeakPowerRaw, HEX);
    #endif

    #if ENABLE_TTE_REG
    Serial.print("TTE          : ");
    Serial.print(tte);
    Serial.println(" min");
    #endif

    #if ENABLE_TTF_REG
    Serial.print("TTF          : ");
    Serial.print(ttf);
    Serial.println(" min");
    #endif

    #if ENABLE_DESIGNCAP_REG
    Serial.print("DesignCap    : ");
    Serial.print(designCap, 0);
    Serial.println(" mAh");
    #endif

    #if ENABLE_ICHGTERM_REG
    Serial.print("IchgTerm     : ");
    Serial.println(ichgTerm);
    #endif

    #if ENABLE_VEMPTY_REG
    Serial.print("Empty Voltage    : ");
    Serial.print(emptyVoltage,2);
    Serial.println(" V");
    Serial.print("Recovery Voltage : ");
    Serial.print(recoveryVoltage,2);
    Serial.println(" V");
    Serial.print("VEMPTY RAW : 0x");
    Serial.println(rawVEmpty, HEX);
    #endif

    #if ENABLE_PACKRESISTANCE_REG
    Serial.print("PackRes      : ");
    Serial.println(packResistance);
    #endif

    #if ENABLE_SYSRESISTANCE_REG
    Serial.print("SysRes       : ");
    Serial.println(sysResistance);
    #endif

    #if ENABLE_RSENSE_USERMEM3_REG
    Serial.print("RsenseMem    : ");
    Serial.println(rsenseUserMem);
    #endif

    #if ENABLE_CYCLES_REG
    Serial.print("Cycles       : ");
    Serial.println(cycles);
    #endif

    #if ENABLE_VALRTTH_REG
    Serial.print("VALRT High : ");
    Serial.print(maxVoltageAlert, 3);
    Serial.println(" V");
    Serial.print("VALRT Low  : ");
    Serial.print(minVoltageAlert, 3);
    Serial.println(" V");
    #endif

    #if ENABLE_TALRTTH_REG
    Serial.print("TALRT High : ");
    Serial.print(maxTempAlert);
    Serial.println(" C");
    Serial.print("TALRT Low  : ");
    Serial.print(minTempAlert);
    Serial.println(" C");
    #endif

    #if ENABLE_SALRTTH_REG
    Serial.print("SALRT High : ");
    Serial.print(maxSocAlert);
    Serial.println(" %");
    Serial.print("SALRT Low  : ");
    Serial.print(minSocAlert);
    Serial.println(" %");
    #endif

    #if ENABLE_IALRTTH_REG
    Serial.print("IALRT High : ");
    Serial.println(maxCurrentAlert);
    Serial.print("IALRT Low  : ");
    Serial.println(minCurrentAlert);
    #endif

    #if ENABLE_STATUS_REG
    Serial.print("STATUS : 0x");
    Serial.println(status, HEX);
    #endif

    #if ENABLE_STATUS_REG
    if (status & STATUS_POR)
        Serial.println("POR Detected");
    if (status & STATUS_VMX)
        Serial.println("Voltage Above Maximum");
    if (status & STATUS_VMN)
        Serial.println("Voltage Below Minimum");
    if (status & STATUS_TMX)
        Serial.println("Temperature Above Maximum");
    if (status & STATUS_TMN)
        Serial.println("Temperature Below Minimum");
    if (status & STATUS_IMX)
        Serial.println("Current Above Maximum");
    if (status & STATUS_IMN)
        Serial.println("Current Below Minimum");
    if (status & STATUS_DSOCI)
        Serial.println("SOC Changed By 1%");
    #endif

    #if ENABLE_STATUS2_REG
    Serial.print("STATUS2 : 0x");
    Serial.println(status2, HEX);
    #endif

    #if ENABLE_STATUS2_REG
    if (status2 & STATUS2_ATRATE_READY)
        Serial.println("AtRate Outputs Ready");
    if (status2 & STATUS2_DP_READY)
        Serial.println("Dynamic Power Ready");
    if (status2 & STATUS2_SN_READY)
        Serial.println("Serial Number Available");
    if (status2 & STATUS2_FULLDET)
        Serial.println("Battery Full");
    if (status2 & STATUS2_HIB)
        Serial.println("Hibernate Mode");
    #endif

    #if ENABLE_FSTAT_REG
    Serial.print("FSTAT REG ADDRESS = 0x");
    Serial.println(FSTAT_REG, HEX);
    Serial.print("FSTAT : 0x");
    Serial.println(fstat, HEX);
    #endif

    #if ENABLE_FSTAT_REG
    if (fstat & FSTAT_DNR)
        Serial.println("Data Not Ready");
    if (fstat & FSTAT_EDet)
        Serial.println("Model Gauge Event Detected");
    if (fstat & FSTAT_FQ)
        Serial.println("Full Qualification Achieved");
    #endif

    #if ENABLE_MAXMINCURR_REG
    Serial.print("Max Current : ");
    Serial.print(maxCurrent * 1000,0);
    Serial.println(" mA");

    Serial.print("Min Current : ");
    Serial.print(minCurrent * 1000,0);
    Serial.println(" mA");

    Serial.print("MAXMINCURR RAW : 0x");
    Serial.println(rawMaxMinCurr, HEX);
    #endif

    #if ENABLE_MAXMINTEMP_REG
    Serial.print("Max Temp : ");
    Serial.print(maxTemperature,0);
    Serial.println(" C");
    Serial.print("Min Temp : ");
    Serial.print(minTemperature,0);
    Serial.println(" C");
    Serial.print("MAXMINTEMP RAW : 0x");
    Serial.println(rawMaxMinTemp,HEX);
    #endif

    #if ENABLE_CONFIG_REG
    Serial.print("CONFIG : 0x");
    Serial.println(config, HEX);
    #endif

    #if ENABLE_CONFIG2_REG
    Serial.print("CONFIG2 : 0x");
    Serial.println(config2, HEX);
    #endif

    #if ENABLE_MODELCFG_REG
    Serial.print("MODELCFG : 0x");
    Serial.println(modelCfg, HEX);
    #endif

    #if ENABLE_LEARNCFG_REG
    Serial.print("LEARNCFG : 0x");
    Serial.println(learnCfg, HEX);
    #endif

    #if ENABLE_FILTERCFG_REG
    Serial.print("FILTERCFG : 0x");
    Serial.println(filterCfg, HEX);
    #endif

    #if ENABLE_RELAXCFG_REG
    Serial.print("RELAXCFG : 0x");
    Serial.println(relaxCfg, HEX);
    #endif

    #if ENABLE_MISCCFG_REG
    Serial.print("MISCCFG : 0x");
    Serial.println(miscCfg, HEX);
    #endif

    #if ENABLE_HIBCFG_REG
    Serial.print("HIBCFG : 0x");
    Serial.println(hibCfg, HEX);
    #endif

    #if ENABLE_CONVGCFG_REG
    Serial.print("CONVGCFG : 0x");
    Serial.println(convgCfg, HEX);
    #endif

    #if ENABLE_RIPPLECFG_REG
    Serial.print("RIPPLECFG : 0x");
    Serial.println(rippleCfg, HEX);
    #endif

    #if ENABLE_RCOMP0_REG
    Serial.print("RCOMP0 : 0x");
    Serial.println(rcomp0, HEX);
    #endif

    #if ENABLE_TEMPCO_REG
    Serial.print("TEMPCO : 0x");
    Serial.println(tempco, HEX);
    #endif

    #if ENABLE_TGAIN_REG
    Serial.print("TGAIN : 0x");
    Serial.println(tgain, HEX);
    #endif

    #if ENABLE_TOFF_REG
    Serial.print("TOFF : 0x");
    Serial.println(toff, HEX);
    #endif

    #if ENABLE_CGAIN_REG
    Serial.print("CGAIN : 0x");
    Serial.println(cgain, HEX);
    #endif

    #if ENABLE_COFF_REG
    Serial.print("COFF : 0x");
    Serial.println(coff, HEX);
    #endif

    #if ENABLE_VGAIN_REG
    Serial.print("VGAIN : 0x");
    Serial.println(vgain, HEX);
    #endif

    Serial.println();
    
    delay(1000);

    if (Serial.available())
    {
        String input = Serial.readStringUntil('\r');

        uint8_t address =
            strtol(
                input.c_str(),
                NULL,
                16);

        MAX17261_ReadReservedRegister(address);
    }
}