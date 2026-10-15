#include <cassert>
#include <iostream>
#include <vector>
#include "Arduino.h"
#include "DW1000Ranging.h"
// Arduino IDE adds these declarations automatically; host C++ does not.
void newRange(); void connectionChanged(DW1000Device*); void resetMeasurements();
void readUart(uint32_t); void readLine(uint32_t); bool getPair(float&,float&,uint32_t);
void calibrate(float,float,bool,uint32_t); void calculateAngle(float,float,uint32_t);
void filterAngle(float,float,float,uint32_t); void finishVote(uint32_t);
void readLidar(uint32_t); void printFront(uint32_t); void printRanges(uint32_t);
#include "../../firmware/A1_Lidar/A1_Lidar.ino"

void resetAll() {
    a1={}; a2={}; cal={}; calibrated=filterReady=havePair=tagChanged=false;
    offset=angle=variance=0; direction=3; lastDirection=-1; clearVotes();
    lastPair=lastAngle=voteStart=lastStatus=lastUart=mockTime=0;
    remotePoll=usedPoll=NO_POLL; discardUart=false; lineLength=-1;
    frontDistance=INFINITY; frontTime=packetTime=0; packetSize=0; packetNeed=10;
    Serial.output.clear(); Serial.input.clear(); Uart.input.clear(); LidarSerial.input.clear();
}
void sendA2(const char* body, uint32_t now, bool corrupt=false) {
    uint8_t crc=0; for(const char* p=body; *p; ++p) crc^=uint8_t(*p);
    char msg[80]; snprintf(msg,sizeof(msg),"$%s*%02X\n",body,crc^(corrupt?1:0));
    for(const char* p=msg; *p; ++p) Uart.input.push_back(*p);
    lastUart=now; readUart(now);
}
void feed(const std::vector<uint8_t>& bytes, uint32_t now) {
    for(auto b:bytes) LidarSerial.input.push_back(b);
    while(LidarSerial.available()) readLidar(now);
}
std::vector<uint8_t> point(float angle, int mm, int count=1) {
    std::vector<uint8_t> p(10+2*count);
    auto put=[&](int i,uint16_t v){p[i]=v&255; p[i+1]=v>>8;};
    put(0,0x55AA); p[3]=count;
    uint16_t raw=(uint16_t(angle*64)<<1)|1; put(4,raw); put(6,raw);
    uint16_t crc=0x55AA^(count<<8);
    for(int i=0;i<count;++i){put(10+2*i,mm*4); crc^=mm*4;}
    put(8,crc); return p;
}
int main() {
    resetAll();
    for(uint32_t t=0;t<=5100;t+=100) calibrate(1.1f,1.2f,true,t);
    assert(calibrated && fabsf(offset-0.1f)<1e-5f);
    calculateAngle(1.1f+offset,1.2f,5200);
    assert(filterReady && fabsf(angle)<0.001f);
    filterReady=false; clearVotes(); calculateAngle(-0.2f,1.2f,5300);
    assert(!filterReady);
    resetAll();
    for(uint32_t t=0;t<=5100;t+=100) calibrate(1.3f,1.2f,true,t);
    assert(calibrated && fabsf(offset+0.1f)<1e-5f);

    resetAll(); float d1,d2;
    sendA2("A2,123,1500,0,1",100);
    a1={1.5f,122,100,true}; assert(!getPair(d1,d2,100));
    a1.poll=123; assert(getPair(d1,d2,100)); assert(!getPair(d1,d2,100));
    sendA2("A2,123,1500,0,1",110); assert(!a2.fresh);
    sendA2("A2,124,1500,0,1",120,true); assert(remotePoll==123);
    sendA2("A2,125,1500,79,1",130); assert(!a2.fresh);

    resetAll();
    // Manufacturer's zero-point packet fixture: valid checksum, zero range.
    feed({0xAA,0x55,0x01,0x01,0x53,0xAE,0x53,0xAE,0xAB,0x54,0,0},10);
    assert(!isfinite(frontDistance));
    auto p=point(10,1000);
    feed({p[0]},20); assert(packetSize==1);
    feed(std::vector<uint8_t>(p.begin()+1,p.end()),21);
    assert(fabsf(frontDistance-1.0f)<0.001f); // Corrected angle ~3.24 deg, inside F.
    feed(point(90,300),22); assert(fabsf(frontDistance-1)<0.001f); // Side ignored.
    feed(point(10,2000),23); assert(fabsf(frontDistance-1)<0.001f); // Farther last point.
    p=point(10,500); p[8]^=1; feed(p,24); assert(fabsf(frontDistance-1)<0.001f);
    printFront(24); assert(Serial.output==" | F=1.000 m");
    Serial.output.clear(); printFront(25); assert(Serial.output==" | F=--");
    feed(point(10,1000),30); Serial.output.clear(); printFront(531);
    assert(Serial.output==" | F=--");
    feed({0xAA},600); readLidar(661); assert(packetSize==0);
    feed(point(10,1500,255),700); assert(packetSize==0 && fabsf(frontDistance-1.5f)<0.001f);
    frontDistance=INFINITY; feed(point(10,0),710); assert(!isfinite(frontDistance));
    feed(point(10,9000),720); assert(!isfinite(frontDistance));

    resetAll(); calibrated=true; offset=0.054f;
    a1={1,10,100,true}; a2={1.054f,10,100,true};
    calculateAngle(a1.range+offset,a2.range,100);
    feed(point(10,1000),450); finishVote(500);
    assert(Serial.output.find("A1=1.054 m | A2=1.054 m")!=std::string::npos);
    assert(Serial.output.find("UWB=1 | F=1.000 m")!=std::string::npos);
    Serial.output.clear(); feed(point(10,2000),950); finishVote(1000);
    assert(Serial.output.find("A1=-- | A2=-- | goc=-- | UWB=0 | F=2.000 m")!=std::string::npos);
    // No state or lock gate: F still prints when UWB is absent.
    Serial.output.clear(); calibrated=false; lastStatus=0;
    feed(point(10,800),1950); calibrate(0,0,false,2000);
    assert(Serial.output.find("CALIB:")!=std::string::npos &&
           Serial.output.find("F=0.800 m")!=std::string::npos);
    mockTime=2050; Serial.input.push_back('c'); calibrated=true; loop();
    assert(!calibrated && offset==0);
    resetAll(); feed(point(10,1000),UINT32_MAX-100); Serial.output.clear(); printFront(50);
    assert(Serial.output==" | F=1.000 m");
    std::cout<<"PASS: BU01 calibration/pairing, X3 framing/checksum/front-only, independent display, expiry and recalibration\n";
}
