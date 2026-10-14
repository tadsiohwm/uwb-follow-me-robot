#pragma once
#include "SPI.h"
struct DW1000Class {
    byte MODE_SHORTDATA_FAST_LOWPOWER[3]={};
    void setAntennaDelay(uint16_t){}
};
inline DW1000Class DW1000;
