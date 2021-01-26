#pragma once

#include <hal/Types.h>
#include <hal/SPI.h>

extern "C" {

struct c_AnalogDevicesIMU_Obj {
    int m_deviceType;
    int m_yawAxis;
    HAL_SPIPort m_spiPort;
    volatile bool m_autoSpiActive;
    HAL_DigitalHandle m_resetPinHandle;
    HAL_DigitalHandle m_dataReadyPinHandle;
    HAL_DigitalHandle m_LEDPinHandle;
};



}