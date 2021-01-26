#pragma once

#include <hal/Types.h>

#include "adi/adiIMU.h"

extern "C" {

struct c_AnalogDevicesIMU_Obj {
    int m_deviceType;
    
    HAL_GyroHandle m_gyroHandle;

};



}