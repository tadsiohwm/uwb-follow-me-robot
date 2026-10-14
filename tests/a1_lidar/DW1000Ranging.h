#pragma once
#include "DW1000.h"
struct DW1000Device {
    struct Timestamp {uint64_t getTimestamp(){return 0;}} timePollSent;
    uint16_t getShortAddress(){return 0x007D;}
    float getRange(){return 1;}
};
struct Ranging {
    void initCommunication(int,int,int){}
    void attachNewRange(void(*)()){}
    void attachBlinkDevice(void(*)(DW1000Device*)){}
    void attachInactiveDevice(void(*)(DW1000Device*)){}
    void startAsAnchor(char*,const byte*,bool){}
    void loop(){}
    DW1000Device* getDistantDevice(){return nullptr;}
};
inline Ranging DW1000Ranging;
