#include <stdint.h>

#if defined(__cplusplus) && __has_include(<array>)
#else
#error This project requires C++11 or greater
#endif

extern "C" {

/* ADIS16448 specific Constants */
const int c_AnalogDevicesIMU_k448ValidProdID = 16448;
const double c_AnalogDevicesIMU_k448GyroScale = 25;
const double c_AnalogDevicesIMU_k448AccelScale = 1200;
const double c_AnalogDevicesIMU_k448MagnScale = 7;
const double c_AnalogDevicesIMU_k448BaroScale = 50;

/* ADIS16448 Register Map Declaration */
/* This IMU does not support a paged address scheme */
static const uint8_t FLASH_CNT_448    =   0x00;   // Flash memory write count
static const uint8_t XGYRO_OUT_448    =   0x04;   // X-axis gyroscope output
static const uint8_t YGYRO_OUT_448    =   0x06;   // Y-axis gyroscope output
static const uint8_t ZGYRO_OUT_448    =   0x08;   // Z-axis gyroscope output
static const uint8_t XACCL_OUT_448    =   0x0A;   // X-axis accelerometer output
static const uint8_t YACCL_OUT_448    =   0x0C;   // Y-axis accelerometer output
static const uint8_t ZACCL_OUT_448    =   0x0E;   // Z-axis accelerometer output
static const uint8_t XMAGN_OUT_448    =   0x10;   // X-axis magnetometer output
static const uint8_t YMAGN_OUT_448    =   0x12;   // Y-axis magnetometer output
static const uint8_t ZMAGN_OUT_448    =   0x14;   // Z-axis magnetometer output
static const uint8_t BARO_OUT_448     =   0x16;   // Barometer pressure measurement, high word
static const uint8_t TEMP_OUT_448     =   0x18;   // Temperature output
static const uint8_t XGYRO_OFF_448    =   0x1A;   // X-axis gyroscope bias offset factor
static const uint8_t YGYRO_OFF_448    =   0x1C;   // Y-axis gyroscope bias offset factor
static const uint8_t ZGYRO_OFF_448    =   0x1E;   // Z-axis gyroscope bias offset factor
static const uint8_t XACCL_OFF_448    =   0x20;   // X-axis acceleration bias offset factor
static const uint8_t YACCL_OFF_448    =   0x22;   // Y-axis acceleration bias offset factor
static const uint8_t ZACCL_OFF_448    =   0x24;   // Z-axis acceleration bias offset factor
static const uint8_t XMAGN_HIC_448    =   0x26;   // X-axis magnetometer, hard iron factor
static const uint8_t YMAGN_HIC_448    =   0x28;   // Y-axis magnetometer, hard iron factor
static const uint8_t ZMAGN_HIC_448    =   0x2A;   // Z-axis magnetometer, hard iron factor
static const uint8_t XMAGN_SIC_448    =   0x2C;   // X-axis magnetometer, soft iron factor
static const uint8_t YMAGN_SIC_448    =   0x2E;   // Y-axis magnetometer, soft iron factor
static const uint8_t ZMAGN_SIC_448    =   0x30;   // Z-axis magnetometer, soft iron factor
static const uint8_t GPIO_CTRL_448    =   0x32;   // GPIO control
static const uint8_t MSC_CTRL_448     =   0x34;   // MISC control
static const uint8_t SMPL_PRD_448     =   0x36;   // Sample clock/Decimation filter control
static const uint8_t SENS_AVG_448     =   0x38;   // Digital filter control
static const uint8_t SEQ_CNT_448      =   0x3A;   // MAGN_OUT and BARO_OUT counter
static const uint8_t DIAG_STAT_448    =   0x3C;   // System status
static const uint8_t GLOB_CMD_448     =   0x3E;   // System command
static const uint8_t ALM_MAG1_448     =   0x40;   // Alarm 1 amplitude threshold
static const uint8_t ALM_MAG2_448     =   0x42;   // Alarm 2 amplitude threshold
static const uint8_t ALM_SMPL1_448    =   0x44;   // Alarm 1 sample size
static const uint8_t ALM_SMPL2_448    =   0x46;   // Alarm 2 sample size
static const uint8_t ALM_CTRL_448     =   0x48;   // Alarm control
static const uint8_t LOT_ID1_448      =   0x52;   // Lot identification number
static const uint8_t LOT_ID2_448      =   0x54;   // Lot identification number
static const uint8_t PROD_ID_448      =   0x56;   // Product identifier
static const uint8_t SERIAL_NUM_448   =   0x58;   // Lot-specific serial number

/* Scaled Angle Outputs */
typedef struct {
  double yawAngle;
} c_AnalogDevicesIMU_448GetAngle;

/* Unscaled Inertial Outputs */
typedef struct {
  uint32_t yawAngle;
  uint16_t xGyro;
  uint16_t yGyro;
  uint16_t zGyro;
  uint16_t xAccel;
  uint16_t yAccel;
  uint16_t zAccel;
  uint16_t xMag;
  uint16_t yMag;
  uint16_t zMag;
  uint16_t baro;
} c_AnalogDevicesIMU_448UnscaledData;

/* Scaled Inertial Outputs */
typedef struct {
  double yawAngle;
  double xGyro;
  double yGyro;
  double zGyro;
  double xAccel;
  double yAccel;
  double zAccel;
  double xMag;
  double yMag;
  double zMag;
  double baro;
} c_AnalogDevicesIMU_448ScaledData;

/* Auto SPI Data Packet */
const uint8_t c_AnalogDevicesIMU_448AutoSpiPacket = GLOB_CMD_448;

// CRC-16 Look-Up Table
  const uint16_t adiscrc[256] = {
  0x0000, 0x17CE, 0x0FDF, 0x1811, 0x1FBE, 0x0870, 0x1061, 0x07AF,
  0x1F3F, 0x08F1, 0x10E0, 0x072E, 0x0081, 0x174F, 0x0F5E, 0x1890,
  0x1E3D, 0x09F3, 0x11E2, 0x062C, 0x0183, 0x164D, 0x0E5C, 0x1992,
  0x0102, 0x16CC, 0x0EDD, 0x1913, 0x1EBC, 0x0972, 0x1163, 0x06AD,
  0x1C39, 0x0BF7, 0x13E6, 0x0428, 0x0387, 0x1449, 0x0C58, 0x1B96,
  0x0306, 0x14C8, 0x0CD9, 0x1B17, 0x1CB8, 0x0B76, 0x1367, 0x04A9,
  0x0204, 0x15CA, 0x0DDB, 0x1A15, 0x1DBA, 0x0A74, 0x1265, 0x05AB,
  0x1D3B, 0x0AF5, 0x12E4, 0x052A, 0x0285, 0x154B, 0x0D5A, 0x1A94,
  0x1831, 0x0FFF, 0x17EE, 0x0020, 0x078F, 0x1041, 0x0850, 0x1F9E,
  0x070E, 0x10C0, 0x08D1, 0x1F1F, 0x18B0, 0x0F7E, 0x176F, 0x00A1,
  0x060C, 0x11C2, 0x09D3, 0x1E1D, 0x19B2, 0x0E7C, 0x166D, 0x01A3,
  0x1933, 0x0EFD, 0x16EC, 0x0122, 0x068D, 0x1143, 0x0952, 0x1E9C,
  0x0408, 0x13C6, 0x0BD7, 0x1C19, 0x1BB6, 0x0C78, 0x1469, 0x03A7,
  0x1B37, 0x0CF9, 0x14E8, 0x0326, 0x0489, 0x1347, 0x0B56, 0x1C98,
  0x1A35, 0x0DFB, 0x15EA, 0x0224, 0x058B, 0x1245, 0x0A54, 0x1D9A,
  0x050A, 0x12C4, 0x0AD5, 0x1D1B, 0x1AB4, 0x0D7A, 0x156B, 0x02A5,
  0x1021, 0x07EF, 0x1FFE, 0x0830, 0x0F9F, 0x1851, 0x0040, 0x178E,
  0x0F1E, 0x18D0, 0x00C1, 0x170F, 0x10A0, 0x076E, 0x1F7F, 0x08B1,
  0x0E1C, 0x19D2, 0x01C3, 0x160D, 0x11A2, 0x066C, 0x1E7D, 0x09B3,
  0x1123, 0x06ED, 0x1EFC, 0x0932, 0x0E9D, 0x1953, 0x0142, 0x168C,
  0x0C18, 0x1BD6, 0x03C7, 0x1409, 0x13A6, 0x0468, 0x1C79, 0x0BB7,
  0x1327, 0x04E9, 0x1CF8, 0x0B36, 0x0C99, 0x1B57, 0x0346, 0x1488,
  0x1225, 0x05EB, 0x1DFA, 0x0A34, 0x0D9B, 0x1A55, 0x0244, 0x158A,
  0x0D1A, 0x1AD4, 0x02C5, 0x150B, 0x12A4, 0x056A, 0x1D7B, 0x0AB5,
  0x0810, 0x1FDE, 0x07CF, 0x1001, 0x17AE, 0x0060, 0x1871, 0x0FBF,
  0x172F, 0x00E1, 0x18F0, 0x0F3E, 0x0891, 0x1F5F, 0x074E, 0x1080,
  0x162D, 0x01E3, 0x19F2, 0x0E3C, 0x0993, 0x1E5D, 0x064C, 0x1182,
  0x0912, 0x1EDC, 0x06CD, 0x1103, 0x16AC, 0x0162, 0x1973, 0x0EBD,
  0x1429, 0x03E7, 0x1BF6, 0x0C38, 0x0B97, 0x1C59, 0x0448, 0x1386,
  0x0B16, 0x1CD8, 0x04C9, 0x1307, 0x14A8, 0x0366, 0x1B77, 0x0CB9,
  0x0A14, 0x1DDA, 0x05CB, 0x1205, 0x15AA, 0x0264, 0x1A75, 0x0DBB,
  0x152B, 0x02E5, 0x1AF4, 0x0D3A, 0x0A95, 0x1D5B, 0x054A, 0x1284
  };

}  // extern "C"