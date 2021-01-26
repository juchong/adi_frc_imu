#pragma once

extern "C" {

#if 0
#ifdef __GNUC__
#define __FORMAT_TEXT(A, B) __attribute__((__format__(__printf__, (A), (B))))
#else
#define __FORMAT_TEXT(A, B)
#endif
#endif

typedef enum {
    c_AnalogDevicesIMU_ErrorNone = 0,
    c_AnalogDevicesIMU_ErrorGeneral,
    c_AnalogDevicesIMU_ErrorSPITimeout,
    c_AnalogDevicesIMU_ErrorNotImplemented,
    c_AnalogDevicesIMU_ErrorSensorNotFound,
    c_AnalogDevicesIMU_ErrorInvalidSensorFound,
    c_AnalogDevicesIMU_ErrorBufferOverflow,
    c_AnalogDevicesIMU_ErrorDataReadyFailed,
    c_AnalogDevicesIMU_ErrorBadChecksumCRC,
    c_AnalogDevicesIMU_ErrorIMUTypeAlreadyInit
} c_AnalogDevicesIMU_ErrorCode;


}  //extern "C"