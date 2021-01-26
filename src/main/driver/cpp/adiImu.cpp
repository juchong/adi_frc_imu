#include <stdint.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include <hal/SPI.h>
#include <hal/DIO.h>
#include <hal/Errors.h>
#include <hal/FRCUsageReporting.h>
#include <hal/HALBase.h>

#include "adi/adiIMUPrivate.h"
#include "adi/adiIMU.h"

extern "C" {

/* Helpful conversion functions */
static inline int32_t ToInt(const uint32_t *buf){
  return (int32_t)( (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3] );
}

static inline uint16_t BuffToUShort(const uint32_t* buf) {
  return ((uint16_t)(buf[0]) << 8) | buf[1];
}

static inline uint8_t BuffToUByte(const uint32_t* buf) {
  return ((uint8_t)buf[0]);
}

static inline int16_t BuffToShort(const uint32_t* buf) {
  return ((int16_t)(buf[0]) << 8) | buf[1];
}

static inline uint16_t ToUShort(const uint8_t* buf) {
  return ((uint16_t)(buf[0]) << 8) | buf[1];
}

static inline int16_t ToShort(const uint8_t* buf) {
  return (int16_t)(((uint16_t)(buf[0]) << 8) | buf[1]);
}

c_AnalogDevicesIMU_Handle c_AnalogDevicesIMU_Create(c_AnalogDevicesIMU_IMUType deviceType, c_AnalogDevicesIMU_YawAxis yawAxis) {
    int32_t spi_status, dio_status = 0;
    c_AnalogDevicesIMU_Handle handle = (c_AnalogDevicesIMU_Handle)malloc(sizeof(struct c_AnalogDevicesIMU_Obj));
    handle->m_deviceType = deviceType;
    handle->m_yawAxis = yawAxis;
    
    if (deviceType == ADIS16448) {
        HAL_InitializeSPI(HAL_SPI_kMXP, &spi_status);
        handle->m_spiPort = HAL_SPI_kMXP;
        handle->m_resetPinHandle = HAL_InitializeDIOPort(18, false, &dio_status);
        handle->m_dataReadyPinHandle = HAL_InitializeDIOPort(10, true, &dio_status);
        handle->m_LEDPinHandle = HAL_InitializeDIOPort(19, false, &dio_status);
    } else {
        // ADIS16470
        HAL_InitializeSPI(HAL_SPI_kOnboardCS0, &spi_status);
        handle->m_spiPort = HAL_SPI_kOnboardCS0;
        handle->m_resetPinHandle = HAL_InitializeDIOPort(27, false, &dio_status);
        handle->m_dataReadyPinHandle = HAL_InitializeDIOPort(28, true, &dio_status);
        handle->m_LEDPinHandle = HAL_InitializeDIOPort(26, false, &dio_status);
    }

    if (spi_status != 0 || dio_status != 0) {
        //TODO: Handle errors correctly. Any error on DIO or SPI indicates the port is already in use (sensor already initialized)
        // Also report the error to the user
    }
    return handle;
}

void c_AnalogDevicesIMU_Destroy(c_AnalogDevicesIMU_Handle handle) {
    int32_t status = 0;
    if (handle == NULL) {
        return;
    }
    if (handle->m_autoSpiActive) {
        HAL_StopSPIAuto(handle->m_spiPort, &status);
    }
    HAL_CloseSPI(handle->m_spiPort);
    //TODO: Do I need to do anything to clean up the DIOs?

    if (status < 0) {
        //TODO: Process SPI HAL error codes
    }
}

c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_ReadRegister(c_AnalogDevicesIMU_Handle handle, uint8_t reg, uint16_t *regData) {
    int32_t status = 0;
    c_AnalogDevicesIMU_ErrorCode errorCode = c_AnalogDevicesIMU_ErrorNone;
    uint8_t txBuf[2];
    uint8_t rxBuf[2];
    txBuf[0] = reg & 0x7F;
    txBuf[1] = 0x00;

    /* Note: I can't use TransactionSPI because there is no way to guarantee stall time */
    status = HAL_WriteSPI(handle->m_spiPort, txBuf, 2);
    //TODO: Check stall time here
    status = HAL_ReadSPI(handle->m_spiPort, rxBuf, 2);

    *regData = (uint16_t)((rxBuf[0] << 8) | rxBuf[1]);

    if (status < 0) {
        //TODO: Process SPI HAL error codes
    }

    return errorCode;
}
c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_WriteRegister(c_AnalogDevicesIMU_Handle handle, uint8_t reg, uint16_t val) {
    int32_t status = 0;
    c_AnalogDevicesIMU_ErrorCode errorCode = c_AnalogDevicesIMU_ErrorNone;
    uint8_t txBuf[4];
    txBuf[0] = (0x80 | reg);
    txBuf[1] = val & 0xFF;
    txBuf[2] = ((0x80 | reg) + 1);
    txBuf[3] = ((val >> 8) & 0xFF);

    //TODO: Check that this works as expected. Telling the API that I'm writing two bytes should drop CS after every two bytes are transmitted.
    status = HAL_WriteSPI(handle->m_spiPort, txBuf, 2);

    if (status < 0) {
        //TODO: Process SPI HAL error codes
    }
    
    return errorCode;
}

c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_GetMetadata(c_AnalogDevicesIMU_Handle handle, c_AnalogDevicesIMU_SensorMetadata* metadata) {
    c_AnalogDevicesIMU_ErrorCode errorCode = c_AnalogDevicesIMU_ErrorNone;
    uint16_t *tmp = 0x00;
    if(handle->m_deviceType == ADIS16448) {
        /* Program year */
        metadata->programYear = 0xFF; /* The 448 was never programmed with this ID */
        /* FW Rev */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, LOT_ID1_448, tmp);
        metadata->fwRev = *tmp;
        /* Product ID */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, PROD_ID_448, tmp);
        metadata->prodId = *tmp;
        /* Serial Number */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, SERIAL_NUM_448, tmp);
        metadata->serialNum = *tmp;
        /* Status Reg */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, DIAG_STAT_448, tmp);
        metadata->diagStatus = *tmp;
        /* Flash Count */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, FLASH_CNT_448, tmp);
        metadata->flashCnt = *tmp;
    }
    else {
        /* Program year */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, USER_SCR1_470, tmp);
        metadata->programYear = *tmp;
        /* FW Rev */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, FIRM_REV_470, tmp);
        metadata->fwRev = *tmp;
        /* Product ID */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, PROD_ID_470, tmp);
        metadata->prodId = *tmp;
        /* Serial Number */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, SERIAL_NUM_470, tmp);
        metadata->serialNum = *tmp;
        /* Status Reg */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, DIAG_STAT_470, tmp);
        metadata->diagStatus = *tmp;
        /* Flash Count */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, FLSHCNT_LOW_470, tmp);
        metadata->flashCnt = *tmp;
    }
    return errorCode;
}

c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_GetSettings(c_AnalogDevicesIMU_Handle handle, c_AnalogDevicesIMU_SensorSettings* settings) {
    c_AnalogDevicesIMU_ErrorCode errorCode = c_AnalogDevicesIMU_ErrorNone;
    uint16_t *tmp = 0x00;
    if(handle->m_deviceType == ADIS16448) {
        /* Filter Setting */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, SENS_AVG_448, tmp);
        settings->filtCtrl = (*tmp & 0x03); /* Mask range and unused bits */
        /* MSC_CTRL */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, MSC_CTRL_448, tmp);
        settings->mscCtrl = *tmp;
        /* Sample Rate*/
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, MSC_CTRL_448, tmp);
        settings->sampleRate = (*tmp & 0x1F00); /* Mask sample clock setting and unused bits */
        /* NULL_CNFG */
        settings->nullCfg = 0xFF; /* The 448 does not have this setting */
    }
    else {
        /* Filter Setting */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, FILT_CTRL_470, tmp);
        settings->filtCtrl = (*tmp & 0x03); /* Mask unused bits */
        /* MSC_CTRL */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, MSC_CTRL_470, tmp);
        settings->mscCtrl = *tmp;
        /* Sample Rate*/
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, DEC_RATE_470, tmp);
        settings->sampleRate = *tmp;
        /* NULL_CNFG */
        errorCode = c_AnalogDevicesIMU_ReadRegister(handle, NULL_CNFG_470, tmp);
        settings->nullCfg = *tmp;
    }
    return errorCode;
}



}
