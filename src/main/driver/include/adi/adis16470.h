#include <stdint.h>

#if defined(__cplusplus) && __has_include(<array>)
#else
#error This project requires C++11 or greater
#endif

extern "C" {

/* ADIS16470 specific Constants */
const double c_AnalogDevicesIMU_k470DeltaAngleSF = 2160.0 / 2147483648.0; /* 2160 / (2^31) */
const int c_AnalogDevicesIMU_k470ValidProdID = 16982;
const double c_AnalogDevicesIMU_k470GyroScale = 10;
const double c_AnalogDevicesIMU_k470AccelScale = 800;

/* ADIS16470 Register Map Declaration */
/* This IMU does not support a paged address scheme */
static const uint8_t DIAG_STAT_470      =   0x02;  //Diagnostic and operational status
static const uint8_t X_GYRO_LOW_470     =   0x04;  //X-axis gyroscope output, lower word
static const uint8_t X_GYRO_OUT_470     =   0x06;  //X-axis gyroscope output, upper word
static const uint8_t Y_GYRO_LOW_470     =   0x08;  //Y-axis gyroscope output, lower word
static const uint8_t Y_GYRO_OUT_470     =   0x0A;  //Y-axis gyroscope output, upper word
static const uint8_t Z_GYRO_LOW_470     = 	0x0C;  //Z-axis gyroscope output, lower word
static const uint8_t Z_GYRO_OUT_470     =   0x0E;  //Z-axis gyroscope output, upper word
static const uint8_t X_ACCL_LOW_470     =   0x10;  //X-axis accelerometer output, lower word
static const uint8_t X_ACCL_OUT_470     =   0x12;  //X-axis accelerometer output, upper word
static const uint8_t Y_ACCL_LOW_470     =   0x14;  //Y-axis accelerometer output, lower word
static const uint8_t Y_ACCL_OUT_470     =   0x16;  //Y-axis accelerometer output, upper word
static const uint8_t Z_ACCL_LOW_470     =   0x18;  //Z-axis accelerometer output, lower word
static const uint8_t Z_ACCL_OUT_470     =   0x1A;  //Z-axis accelerometer output, upper word
static const uint8_t TEMP_OUT_470       =   0x1C;  //Temperature output (internal, not calibrated)
static const uint8_t TIME_STAMP_470     =   0x1E;  //PPS mode time stamp
static const uint8_t DATA_CNTR_470      =   0x22;  //New data counter
static const uint8_t X_DELTANG_LOW_470  =   0x24;  //X-axis delta angle output, lower word
static const uint8_t X_DELTANG_OUT_470  =   0x26;  //X-axis delta angle output, upper word
static const uint8_t Y_DELTANG_LOW_470  =   0x28;  //Y-axis delta angle output, lower word
static const uint8_t Y_DELTANG_OUT_470  =   0x2A;  //Y-axis delta angle output, upper word
static const uint8_t Z_DELTANG_LOW_470  =   0x2C;  //Z-axis delta angle output, lower word
static const uint8_t Z_DELTANG_OUT_470  =   0x2E;  //Z-axis delta angle output, upper word
static const uint8_t X_DELTVEL_LOW_470  =   0x30;  //X-axis delta velocity output, lower word
static const uint8_t X_DELTVEL_OUT_470  =   0x32;  //X-axis delta velocity output, upper word
static const uint8_t Y_DELTVEL_LOW_470  =   0x34;  //Y-axis delta velocity output, lower word
static const uint8_t Y_DELTVEL_OUT_470  =   0x36;  //Y-axis delta velocity output, upper word
static const uint8_t Z_DELTVEL_LOW_470  =   0x38;  //Z-axis delta velocity output, lower word
static const uint8_t Z_DELTVEL_OUT_470  =   0x3A;  //Z-axis delta velocity output, upper word
static const uint8_t XG_BIAS_LOW_470    =   0x40;  //X-axis gyroscope bias offset correction, lower word
static const uint8_t XG_BIAS_HIGH_470   =   0x42;  //X-axis gyroscope bias offset correction, upper word
static const uint8_t YG_BIAS_LOW_470    =   0x44;  //Y-axis gyroscope bias offset correction, lower word
static const uint8_t YG_BIAS_HIGH_470 	=   0x46;  //Y-axis gyroscope bias offset correction, upper word
static const uint8_t ZG_BIAS_LOW_470    =   0x48;  //Z-axis gyroscope bias offset correction, lower word
static const uint8_t ZG_BIAS_HIGH_470   =   0x4A;  //Z-axis gyroscope bias offset correction, upper word
static const uint8_t XA_BIAS_LOW_470    =   0x4C;  //X-axis accelerometer bias offset correction, lower word
static const uint8_t XA_BIAS_HIGH_470   =   0x4E;  //X-axis accelerometer bias offset correction, upper word
static const uint8_t YA_BIAS_LOW_470    =   0x50;  //Y-axis accelerometer bias offset correction, lower word
static const uint8_t YA_BIAS_HIGH_470   =   0x52;  //Y-axis accelerometer bias offset correction, upper word
static const uint8_t ZA_BIAS_LOW_470    =   0x54;  //Z-axis accelerometer bias offset correction, lower word
static const uint8_t ZA_BIAS_HIGH_470   =   0x56;  //Z-axis accelerometer bias offset correction, upper word
static const uint8_t FILT_CTRL_470      =   0x5C;  //Filter control
static const uint8_t MSC_CTRL_470       =   0x60;  //Miscellaneous control
static const uint8_t UP_SCALE_470       =   0x62;  //Clock scale factor, PPS mode
static const uint8_t DEC_RATE_470       =   0x64;  //Decimation rate control (output data rate)
static const uint8_t NULL_CNFG_470      =   0x66;  //Auto-null configuration control
static const uint8_t GLOB_CMD_470       =   0x68;  //Global commands
static const uint8_t FIRM_REV_470       =   0x6C;  //Firmware revision
static const uint8_t FIRM_DM_470        =   0x6E;  //Firmware revision date, month and day
static const uint8_t FIRM_Y_470         =   0x70;  //Firmware revision date, year
static const uint8_t PROD_ID_470        =   0x72;  //Product identification 
static const uint8_t SERIAL_NUM_470     =   0x74;  //Serial number (relative to assembly lot)
static const uint8_t USER_SCR1_470      =   0x76;  //User scratch register 1 
static const uint8_t USER_SCR2_470      =   0x78;  //User scratch register 2 
static const uint8_t USER_SCR3_470      =   0x7A;  //User scratch register 3 
static const uint8_t FLSHCNT_LOW_470    =   0x7C;  //Flash update count, lower word 
static const uint8_t FLSHCNT_HIGH_470   =   0x7E;  //Flash update count, upper word 

/* Scaled Angle Outputs */
typedef struct {
  double yawAngle;
} c_AnalogDevicesIMU_470GetAngle;

/* Unscaled Inertial Outputs */
typedef struct {
  uint32_t yawAngle;
  uint16_t xGyro;
  uint16_t yGyro;
  uint16_t zGyro;
  uint16_t xAccel;
  uint16_t yAccel;
  uint16_t zAccel;
} c_AnalogDevicesIMU_470UnscaledData;

/* Scaled Inertial Outputs */
typedef struct {
  double yawAngle;
  double xGyro;
  double yGyro;
  double zGyro;
  double xAccel;
  double yAccel;
  double zAccel;
} c_AnalogDevicesIMU_470ScaledData;

/* Auto SPI Data Packets */
const uint8_t c_AnalogDevicesIMU_470AutoSpiXPacket [16] = {
X_DELTANG_OUT_470, 
0x00, 
X_DELTANG_LOW_470, 
0x00, 
X_GYRO_OUT_470,
0x00, 
Y_GYRO_OUT_470, 
0x00, 
Z_GYRO_OUT_470, 
0x00, 
X_ACCL_OUT_470, 
0x00, 
Y_ACCL_OUT_470, 
0x00,
Z_ACCL_OUT_470,
0x00
};

const uint8_t c_AnalogDevicesIMU_470AutoSpiYPacket [16] = {
Y_DELTANG_OUT_470, 
0x00, 
Y_DELTANG_LOW_470, 
0x00, 
X_GYRO_OUT_470,
0x00, 
Y_GYRO_OUT_470, 
0x00, 
Z_GYRO_OUT_470, 
0x00, 
X_ACCL_OUT_470, 
0x00, 
Y_ACCL_OUT_470, 
0x00,
Z_ACCL_OUT_470,
0x00
};

const uint8_t c_AnalogDevicesIMU_470AutoSpiZPacket [16] = {
Z_DELTANG_OUT_470, 
0x00, 
Z_DELTANG_LOW_470, 
0x00, 
X_GYRO_OUT_470,
0x00, 
Y_GYRO_OUT_470, 
0x00, 
Z_GYRO_OUT_470, 
0x00, 
X_ACCL_OUT_470, 
0x00, 
Y_ACCL_OUT_470, 
0x00,
Z_ACCL_OUT_470,
0x00
};

}  // extern "C"