
/* File generated by gen_cfile.py. Should not be modified. */

#ifndef CANOPEN_MASTER_M200_H
#define CANOPEN_MASTER_M200_H

#include "data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 _valueRangeTest (UNS8 typeValue, void * value);
const indextable * _scanIndexOD (CO_Data *d, UNS16 wIndex, UNS32 * errorCode);

/* Master node data struct */
extern CO_Data _Data;
extern UNS8 driverBoard_deviceType;		/* Mapped at index 0x3000, subindex 0x01 */
extern UNS8 driverBoard_errorRegister;		/* Mapped at index 0x3000, subindex 0x02 */
extern UNS8 driverBoard_deviceName[10];		/* Mapped at index 0x3000, subindex 0x03 */
extern UNS8 driverBoard_harewareVersion[10];		/* Mapped at index 0x3000, subindex 0x04 */
extern UNS8 driverBoard_softwareVersion[10];		/* Mapped at index 0x3000, subindex 0x05 */
extern INTEGER16 driverBoard_mcuTempRise;		/* Mapped at index 0x3000, subindex 0x06 */
extern UNS8 driverBoard_scramStop;		/* Mapped at index 0x3000, subindex 0x07 */
extern UNS8 driverBoard_carLight;		/* Mapped at index 0x3000, subindex 0x08 */
extern INTEGER16 driverBoard_mpu6050Temp;		/* Mapped at index 0x3000, subindex 0x09 */
extern INTEGER16 driverBoard_PT100Temp1;		/* Mapped at index 0x3000, subindex 0x0A */
extern INTEGER16 driverBoard_PT100Temp2;		/* Mapped at index 0x3000, subindex 0x0B */
extern UNS8 driverBoard_uncoverFlag;		/* Mapped at index 0x3000, subindex 0x0C */
extern UNS8 driverBoard_carFanSpeed;		/* Mapped at index 0x3000, subindex 0x0D */
extern UNS8 driverBoard_boxFanSpeed;		/* Mapped at index 0x3000, subindex 0x0E */
extern UNS8 driverBoard_draughtFanspeed;		/* Mapped at index 0x3000, subindex 0x0F */
extern UNS8 driverBoard_pumpSpeed;		/* Mapped at index 0x3000, subindex 0x10 */
extern INTEGER16 driverBoard_carPitchAngle;		/* Mapped at index 0x3000, subindex 0x11 */
extern INTEGER16 driverBoard_carRollAngle;		/* Mapped at index 0x3000, subindex 0x12 */
extern UNS8 driverBoard_contactorState;		/* Mapped at index 0x3000, subindex 0x13 */
extern UNS32 softSwitch_deviceType;		/* Mapped at index 0x3001, subindex 0x01 */
extern UNS8 softSwitch_errorRegister;		/* Mapped at index 0x3001, subindex 0x02 */
extern UNS8 softSwitch_deviceName[10];		/* Mapped at index 0x3001, subindex 0x03 */
extern UNS8 softSwitch_harewareVersion[10];		/* Mapped at index 0x3001, subindex 0x04 */
extern UNS8 softSwitch_softwareVersion[10];		/* Mapped at index 0x3001, subindex 0x05 */
extern INTEGER16 softSwitch_mcuTempRise;		/* Mapped at index 0x3001, subindex 0x06 */
extern UNS8 softSwitch_switchFlag;		/* Mapped at index 0x3001, subindex 0x07 */
extern UNS8 softSwitch_switchBatteySOC;		/* Mapped at index 0x3001, subindex 0x08 */
extern INTEGER32 speaker_deviceType;		/* Mapped at index 0x3002, subindex 0x01 */
extern UNS8 speaker_errorRegister;		/* Mapped at index 0x3002, subindex 0x02 */
extern UNS8 speaker_deviceName[10];		/* Mapped at index 0x3002, subindex 0x03 */
extern UNS8 speaker_harewareVersion[10];		/* Mapped at index 0x3002, subindex 0x04 */
extern UNS8 speaker_softwareVersion[10];		/* Mapped at index 0x3002, subindex 0x05 */
extern INTEGER16 speaker_mcuTempRise;		/* Mapped at index 0x3002, subindex 0x06 */
extern UNS8 speaker_speakerItem;		/* Mapped at index 0x3002, subindex 0x07 */
extern UNS32 s48100Battery_batteryFaultState;		/* Mapped at index 0x3003, subindex 0x01 */
extern UNS32 s48100Battery_BMSStatus;		/* Mapped at index 0x3003, subindex 0x02 */
extern UNS8 s48100Battery_batteryCloseControl;		/* Mapped at index 0x3003, subindex 0x03 */
extern UNS8 s48100Battery_chargeDischargeControl;		/* Mapped at index 0x3003, subindex 0x04 */
extern UNS16 s48100Battery_batterySOC;		/* Mapped at index 0x3003, subindex 0x05 */
extern UNS8 s48100Battery_batterySOH;		/* Mapped at index 0x3003, subindex 0x06 */
extern UNS16 s48100Battery_capRemain;		/* Mapped at index 0x3003, subindex 0x07 */
extern UNS16 s48100Battery_cycleIndex;		/* Mapped at index 0x3003, subindex 0x08 */
extern INTEGER32 s48100Battery_currentCurrent;		/* Mapped at index 0x3003, subindex 0x09 */
extern UNS16 s48100Battery_currentVoltage;		/* Mapped at index 0x3003, subindex 0x0A */
extern UNS16 s48100Battery_packVoltage;		/* Mapped at index 0x3003, subindex 0x0B */
extern INTEGER16 s48100Battery_cellTempMax;		/* Mapped at index 0x3003, subindex 0x0C */
extern INTEGER16 s48100Battery_cellTempMin;		/* Mapped at index 0x3003, subindex 0x0D */
extern UNS16 s48100Battery_cellVoltageMax;		/* Mapped at index 0x3003, subindex 0x0E */
extern UNS16 s48100Battery_cellVoltageMin;		/* Mapped at index 0x3003, subindex 0x0F */
extern UNS16 s48100Battery_cellVoltageDiffMax;		/* Mapped at index 0x3003, subindex 0x10 */
extern UNS16 s48100Battery_chargeVoltageAllow;		/* Mapped at index 0x3003, subindex 0x11 */
extern UNS16 s48100Battery_chargeCurrentAllow;		/* Mapped at index 0x3003, subindex 0x12 */
extern UNS16 s48100Battery_nominalVoltage;		/* Mapped at index 0x3003, subindex 0x13 */
extern UNS16 s48100Battery_capDesign;		/* Mapped at index 0x3003, subindex 0x14 */
extern UNS8 s48100Battery_manufacturer[10];		/* Mapped at index 0x3003, subindex 0x15 */
extern UNS8 s48100Battery_softwareVersion[10];		/* Mapped at index 0x3003, subindex 0x16 */
extern UNS8 s48100Battery_hardwareVersion[10];		/* Mapped at index 0x3003, subindex 0x17 */
extern UNS8 s48100Battery_seriesParallelNum;		/* Mapped at index 0x3003, subindex 0x18 */
extern UNS8 s48100Battery_batterySN[10];		/* Mapped at index 0x3003, subindex 0x19 */
extern UNS8 s48100Battery_cellSN[10];		/* Mapped at index 0x3003, subindex 0x1A */
extern UNS16 s48100Battery_warningState;		/* Mapped at index 0x3003, subindex 0x1B */
extern UNS16 s48100Battery_protectState;		/* Mapped at index 0x3003, subindex 0x1C */
extern UNS8 s48100Battery_tempTestPoint;		/* Mapped at index 0x3003, subindex 0x1D */
extern INTEGER16 s48100Battery_batteryTemp1;		/* Mapped at index 0x3003, subindex 0x1E */
extern INTEGER16 s48100Battery_batteryTemp2;		/* Mapped at index 0x3003, subindex 0x1F */
extern INTEGER16 s48100Battery_batteryTemp3;		/* Mapped at index 0x3003, subindex 0x20 */
extern INTEGER16 s48100Battery_batteryTemp4;		/* Mapped at index 0x3003, subindex 0x21 */
extern INTEGER16 s48100Battery_batteryTemp5;		/* Mapped at index 0x3003, subindex 0x22 */
extern INTEGER16 s48100Battery_batteryTemp6;		/* Mapped at index 0x3003, subindex 0x23 */
extern INTEGER16 s48100Battery_batteryTemp7;		/* Mapped at index 0x3003, subindex 0x24 */
extern INTEGER16 s48100Battery_batteryTemp8;		/* Mapped at index 0x3003, subindex 0x25 */
extern UNS8 s48100Battery_subIndexNum;		/* Mapped at index 0x3003, subindex 0x26 */
extern UNS32 spraySensor_deviceType;		/* Mapped at index 0x3004, subindex 0x01 */
extern UNS8 spraySensor_errorRegister;		/* Mapped at index 0x3004, subindex 0x02 */
extern UNS8 spraySensor_deviceName;		/* Mapped at index 0x3004, subindex 0x03 */
extern UNS8 spraySensor_harewareVersion;		/* Mapped at index 0x3004, subindex 0x04 */
extern UNS8 spraySensor_softwareVersion;		/* Mapped at index 0x3004, subindex 0x05 */
extern INTEGER16 spraySensor_mcuTempRise;		/* Mapped at index 0x3004, subindex 0x06 */
extern UNS16 spraySensor_waterPressure;		/* Mapped at index 0x3004, subindex 0x07 */
extern INTEGER16 spraySensor_waterFlow;		/* Mapped at index 0x3004, subindex 0x08 */
extern UNS16 spraySensor_waterLevel;		/* Mapped at index 0x3004, subindex 0x09 */
extern UNS32 waterFlowSensor_deviceType;		/* Mapped at index 0x3005, subindex 0x01 */
extern UNS8 waterFlowSensor_errorRegister;		/* Mapped at index 0x3005, subindex 0x02 */
extern UNS8 waterFlowSensor_deviceName;		/* Mapped at index 0x3005, subindex 0x03 */
extern UNS8 waterFlowSensor_harewareVersion;		/* Mapped at index 0x3005, subindex 0x04 */
extern UNS8 waterFlowSensor_softwareVersion;		/* Mapped at index 0x3005, subindex 0x05 */
extern INTEGER16 waterFlowSensor_mcuTempRise;		/* Mapped at index 0x3005, subindex 0x06 */
extern UNS8 waterFlowSensor_waterFlow;		/* Mapped at index 0x3005, subindex 0x07 */
extern UNS32 waterLevelSensor_deviceType;		/* Mapped at index 0x3006, subindex 0x01 */
extern UNS8 waterLevelSensor_errorRegister;		/* Mapped at index 0x3006, subindex 0x02 */
extern UNS8 waterLevelSensor_deviceName;		/* Mapped at index 0x3006, subindex 0x03 */
extern UNS8 waterLevelSensor_harewareVersion;		/* Mapped at index 0x3006, subindex 0x04 */
extern UNS8 waterLevelSensor_softwareVersion;		/* Mapped at index 0x3006, subindex 0x05 */
extern INTEGER16 waterLevelSensor_mcuTempRise;		/* Mapped at index 0x3006, subindex 0x06 */
extern UNS8 waterLevelSensor_waterLevel;		/* Mapped at index 0x3006, subindex 0x07 */
extern UNS32 LED12VL_deviceType;		/* Mapped at index 0x3007, subindex 0x01 */
extern UNS8 LED12VL_errorRegister;		/* Mapped at index 0x3007, subindex 0x02 */
extern UNS8 LED12VL_deviceName[10];		/* Mapped at index 0x3007, subindex 0x03 */
extern UNS8 LED12VL_harewareVersion[10];		/* Mapped at index 0x3007, subindex 0x04 */
extern UNS8 LED12VL_softwareVersion[10];		/* Mapped at index 0x3007, subindex 0x05 */
extern INTEGER16 LED12VL_mcuTempRise;		/* Mapped at index 0x3007, subindex 0x06 */
extern UNS8 LED12VL_LED12VLState;		/* Mapped at index 0x3007, subindex 0x07 */
extern UNS8 LED12VL_LED12VLStateReset;		/* Mapped at index 0x3007, subindex 0x08 */
extern UNS32 LED12VR_deviceType;		/* Mapped at index 0x3008, subindex 0x01 */
extern UNS8 LED12VR_errorRegister;		/* Mapped at index 0x3008, subindex 0x02 */
extern UNS8 LED12VR_deviceName[10];		/* Mapped at index 0x3008, subindex 0x03 */
extern UNS8 LED12VR_harewareVersion[10];		/* Mapped at index 0x3008, subindex 0x04 */
extern UNS8 LED12VR_softwareVersion[10];		/* Mapped at index 0x3008, subindex 0x05 */
extern INTEGER16 LED12VR_mcuTempRise;		/* Mapped at index 0x3008, subindex 0x06 */
extern UNS8 LED12VR_LED12VRState;		/* Mapped at index 0x3008, subindex 0x07 */
extern UNS8 LED12VR_LED12VRStateReset;		/* Mapped at index 0x3008, subindex 0x08 */
extern UNS32 SHT30CarBox_deviceType;		/* Mapped at index 0x300A, subindex 0x01 */
extern UNS8 SHT30CarBox_errorRegister;		/* Mapped at index 0x300A, subindex 0x02 */
extern UNS8 SHT30CarBox_deviceName[10];		/* Mapped at index 0x300A, subindex 0x03 */
extern UNS8 SHT30CarBox_harewareVersion[10];		/* Mapped at index 0x300A, subindex 0x04 */
extern UNS8 SHT30CarBox_softwareVersion[10];		/* Mapped at index 0x300A, subindex 0x05 */
extern INTEGER16 SHT30CarBox_mcuTempRise;		/* Mapped at index 0x300A, subindex 0x06 */
extern INTEGER16 SHT30CarBox_temperature;		/* Mapped at index 0x300A, subindex 0x07 */
extern UNS8 SHT30CarBox_humidity;		/* Mapped at index 0x300A, subindex 0x08 */
extern UNS32 SHT30Environment_deviceType;		/* Mapped at index 0x300B, subindex 0x01 */
extern UNS8 SHT30Environment_errorRegister;		/* Mapped at index 0x300B, subindex 0x02 */
extern UNS8 SHT30Environment_deviceName[10];		/* Mapped at index 0x300B, subindex 0x03 */
extern UNS8 SHT30Environment_harewareVersion[10];		/* Mapped at index 0x300B, subindex 0x04 */
extern UNS8 SHT30Environment_softwareVersion[10];		/* Mapped at index 0x300B, subindex 0x05 */
extern INTEGER16 SHT30Environment_mcuTempRise;		/* Mapped at index 0x300B, subindex 0x06 */
extern INTEGER16 SHT30Environment_temperature;		/* Mapped at index 0x300B, subindex 0x07 */
extern UNS8 SHT30Environment_humidity;		/* Mapped at index 0x300B, subindex 0x08 */

#endif // CANOPEN_MASTER_M200_H
