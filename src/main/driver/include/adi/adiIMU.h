#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>
#include <set>
#include <mutex>

#include "adi/adiIMUErrors.h"
#include "adi/adis16470.h"
#include "adi/adis16448.h"

#if defined(__cplusplus) && __has_include(<array>)
#else
#error This project requires C++11 or greater
#endif

extern "C" {

/* Useful constants */
const double c_AnalogDevicesIMU_krad_to_deg = 57.2957795;
const double c_AnalogDevicesIMU_kdeg_to_rad = 0.0174532;
const double c_AnalogDevicesIMU_kgrav = 9.81;

/* IMU handle */
typedef struct c_AnalogDevicesIMU_Obj* c_AnalogDevicesIMU_Handle;

typedef enum {
    ADIS16470 = 0,
    ADIS16448 = 1
} c_AnalogDevicesIMU_IMUType;

/* Sensor orientation setting enum (yaw axis) */
typedef enum {
  x_up = 0,
  y_up = 1,
  z_up = 2
} c_AnalogDevicesIMU_YawAxis;

/*  Continuous Calibration Time Enum Class */
typedef enum { 
  _32ms = 0,
  _64ms = 1,
  _128ms = 2,
  _256ms = 3,
  _512ms = 4,
  _1s = 5,
  _2s = 6,
  _4s = 7,
  _8s = 8,
  _16s = 9,
  _32s = 10,
  _64s = 11
} c_AnalogDevicesIMU_ContinuousCalTime;

c_AnalogDevicesIMU_Handle c_AnalogDevicesIMU_Create(c_AnalogDevicesIMU_IMUType deviceType, c_AnalogDevicesIMU_YawAxis yawAxis);
void c_AnalogDevicesIMU_Destroy(c_AnalogDevicesIMU_Handle handle);

c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_GetMetadata(c_AnalogDevicesIMU_Handle handle);
c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_GetSettings(c_AnalogDevicesIMU_Handle handle);
c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_GetAngle(c_AnalogDevicesIMU_Handle handle);
c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_GetUnscaledData(c_AnalogDevicesIMU_Handle handle);
c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_GetScaledData(c_AnalogDevicesIMU_Handle handle);
c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_WriteSampleRate(c_AnalogDevicesIMU_Handle handle, int sampleRate);
c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_WriteContinuousCalTime(c_AnalogDevicesIMU_Handle handle, c_AnalogDevicesIMU_ContinuousCalTime calTime);
c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_WriteFilterSetting(c_AnalogDevicesIMU_Handle handle, int filtSetting);
c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_WriteSettingsToFlash(c_AnalogDevicesIMU_Handle handle);
c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_ReadRegister(c_AnalogDevicesIMU_Handle handle, uint8_t reg, uint16_t* rxBuf);
c_AnalogDevicesIMU_ErrorCode c_AnalogDevicesIMU_WriteRegister(c_AnalogDevicesIMU_Handle handle, uint8_t reg, uint16_t* txBuf);


}
