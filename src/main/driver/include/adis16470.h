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

#if defined(__cplusplus) && __has_include(<array>)
#else
#error This project requires C++11 or greater
#endif

extern "C" {

/* ADIS16470 Calibration Time Enum Class */
enum class ADIS16470CalibrationTime { 
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
};

/* ADIS16470 Register Map Declaration */
static constexpr uint8_t FLASH_CNT      =   0x00;  //Flash memory write count
static constexpr uint8_t DIAG_STAT      =   0x02;  //Diagnostic and operational status
static constexpr uint8_t X_GYRO_LOW     =   0x04;  //X-axis gyroscope output, lower word
static constexpr uint8_t X_GYRO_OUT     =   0x06;  //X-axis gyroscope output, upper word
static constexpr uint8_t Y_GYRO_LOW     =   0x08;  //Y-axis gyroscope output, lower word
static constexpr uint8_t Y_GYRO_OUT     =  	0x0A;  //Y-axis gyroscope output, upper word
static constexpr uint8_t Z_GYRO_LOW     = 	0x0C;  //Z-axis gyroscope output, lower word
static constexpr uint8_t Z_GYRO_OUT     =   0x0E;  //Z-axis gyroscope output, upper word
static constexpr uint8_t X_ACCL_LOW     =   0x10;  //X-axis accelerometer output, lower word
static constexpr uint8_t X_ACCL_OUT     =   0x12;  //X-axis accelerometer output, upper word
static constexpr uint8_t Y_ACCL_LOW     =   0x14;  //Y-axis accelerometer output, lower word
static constexpr uint8_t Y_ACCL_OUT     =   0x16;  //Y-axis accelerometer output, upper word
static constexpr uint8_t Z_ACCL_LOW     =   0x18;  //Z-axis accelerometer output, lower word
static constexpr uint8_t Z_ACCL_OUT     =   0x1A;  //Z-axis accelerometer output, upper word
static constexpr uint8_t TEMP_OUT       =   0x1C;  //Temperature output (internal, not calibrated)
static constexpr uint8_t TIME_STAMP     =   0x1E;  //PPS mode time stamp
static constexpr uint8_t X_DELTANG_LOW  =   0x24;  //X-axis delta angle output, lower word
static constexpr uint8_t X_DELTANG_OUT  =   0x26;  //X-axis delta angle output, upper word
static constexpr uint8_t Y_DELTANG_LOW  =   0x28;  //Y-axis delta angle output, lower word
static constexpr uint8_t Y_DELTANG_OUT  =   0x2A;  //Y-axis delta angle output, upper word
static constexpr uint8_t Z_DELTANG_LOW  =   0x2C;  //Z-axis delta angle output, lower word
static constexpr uint8_t Z_DELTANG_OUT  =   0x2E;  //Z-axis delta angle output, upper word
static constexpr uint8_t X_DELTVEL_LOW  =   0x30;  //X-axis delta velocity output, lower word
static constexpr uint8_t X_DELTVEL_OUT  =   0x32;  //X-axis delta velocity output, upper word
static constexpr uint8_t Y_DELTVEL_LOW  =   0x34;  //Y-axis delta velocity output, lower word
static constexpr uint8_t Y_DELTVEL_OUT  =   0x36;  //Y-axis delta velocity output, upper word
static constexpr uint8_t Z_DELTVEL_LOW  =   0x38;  //Z-axis delta velocity output, lower word
static constexpr uint8_t Z_DELTVEL_OUT  =   0x3A;  //Z-axis delta velocity output, upper word
static constexpr uint8_t XG_BIAS_LOW    =   0x40;  //X-axis gyroscope bias offset correction, lower word
static constexpr uint8_t XG_BIAS_HIGH   =   0x42;  //X-axis gyroscope bias offset correction, upper word
static constexpr uint8_t YG_BIAS_LOW    =   0x44;  //Y-axis gyroscope bias offset correction, lower word
static constexpr uint8_t YG_BIAS_HIGH 	=   0x46;  //Y-axis gyroscope bias offset correction, upper word
static constexpr uint8_t ZG_BIAS_LOW    =   0x48;  //Z-axis gyroscope bias offset correction, lower word
static constexpr uint8_t ZG_BIAS_HIGH   =   0x4A;  //Z-axis gyroscope bias offset correction, upper word
static constexpr uint8_t XA_BIAS_LOW    =   0x4C;  //X-axis accelerometer bias offset correction, lower word
static constexpr uint8_t XA_BIAS_HIGH   =   0x4E;  //X-axis accelerometer bias offset correction, upper word
static constexpr uint8_t YA_BIAS_LOW    =   0x50;  //Y-axis accelerometer bias offset correction, lower word
static constexpr uint8_t YA_BIAS_HIGH   =   0x52;  //Y-axis accelerometer bias offset correction, upper word
static constexpr uint8_t ZA_BIAS_LOW    =   0x54;  //Z-axis accelerometer bias offset correction, lower word
static constexpr uint8_t ZA_BIAS_HIGH   =   0x56;  //Z-axis accelerometer bias offset correction, upper word
static constexpr uint8_t FILT_CTRL      =   0x5C;  //Filter control
static constexpr uint8_t MSC_CTRL       =   0x60;  //Miscellaneous control
static constexpr uint8_t UP_SCALE       =   0x62;  //Clock scale factor, PPS mode
static constexpr uint8_t DEC_RATE       =   0x64;  //Decimation rate control (output data rate)
static constexpr uint8_t NULL_CNFG      =   0x66;  //Auto-null configuration control
static constexpr uint8_t GLOB_CMD       =   0x68;  //Global commands
static constexpr uint8_t FIRM_REV       =   0x6C;  //Firmware revision
static constexpr uint8_t FIRM_DM        =   0x6E;  //Firmware revision date, month and day
static constexpr uint8_t FIRM_Y         =   0x70;  //Firmware revision date, year
static constexpr uint8_t PROD_ID        =   0x72;  //Product identification 
static constexpr uint8_t SERIAL_NUM     =   0x74;  //Serial number (relative to assembly lot)
static constexpr uint8_t USER_SCR1      =   0x76;  //User scratch register 1 
static constexpr uint8_t USER_SCR2      =   0x78;  //User scratch register 2 
static constexpr uint8_t USER_SCR3      =   0x7A;  //User scratch register 3 
static constexpr uint8_t FLSHCNT_LOW    =   0x7C;  //Flash update count, lower word 
static constexpr uint8_t FLSHCNT_HIGH   =   0x7E;  //Flash update count, upper word 

}  // extern "C"