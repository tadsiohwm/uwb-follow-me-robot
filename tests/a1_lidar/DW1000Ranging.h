#pragma once
#include "DW1000.h"
struct DW1000Device {
    struct Timestamp {uint64_t value=0; uint64_t getTimestamp(){return value;}} timePollSent;
    uint16_t address=0x007D;
    float range=1;
    uint16_t getShortAddress(){return address;}
    float getRange(){return range;}
};
struct Ranging {
    void initCommunication(int,int,int){}
    void attachNewRange(void(*)()){}
    void attachBlinkDevice(void(*)(DW1000Device*)){}
    void attachInactiveDevice(void(*)(DW1000Device*)){}
    void startAsAnchor(char*,const byte*,bool){}
    void (*loopHook)()=nullptr;
    DW1000Device* device=nullptr;
    unsigned loopCalls=0;
    void loop(){++loopCalls; if(loopHook) loopHook();}
    DW1000Device* getDistantDevice(){return device;}
};
inline Ranging DW1000Ranging;
