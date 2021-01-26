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
}



}
