#pragma once
#include <stdint.h>
#include <stddef.h>
#include <deque>
#include <string>
#include <cstdio>
using byte = uint8_t;
constexpr int SERIAL_8N1=0;
constexpr double PI=3.141592653589793;
inline uint32_t mockTime=0;
inline uint32_t millis(){return mockTime;}
inline void delay(unsigned long n){mockTime+=n;}
struct HardwareSerial {
    std::deque<uint8_t> input;
    std::string output;
    HardwareSerial(int=0){}
    void begin(int,int=0,int=0,int=0){}
    void setRxBufferSize(int){}
    int available(){return input.size();}
    int read(){if(input.empty()) return -1; int c=input.front();input.pop_front();return c;}
    int availableForWrite(){return 512;}
    void setTxBufferSize(int){}
    size_t write(const uint8_t* p, size_t n){output.append(reinterpret_cast<const char*>(p), n); return n;}
    void print(const char* p){output += p;}
    void println(const char* p=""){output += p; output += '\n';}
    template<class... T> void printf(const char* format,T... args){
        char buffer[1024]; int n=std::snprintf(buffer,sizeof(buffer),format,args...);
        if(n>0) output.append(buffer, size_t(n)<sizeof(buffer) ? size_t(n) : sizeof(buffer)-1);
    }
};
inline HardwareSerial Serial;
struct SPIClass {void begin(int,int,int){}};
inline SPIClass SPI;

inline bool ledcAttach(int,int,int){return true;}
inline void ledcWrite(int,int){}
