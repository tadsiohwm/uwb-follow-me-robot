#pragma once
#include <stdint.h>
#include <stddef.h>
#include <deque>
using byte = uint8_t;
constexpr int SERIAL_8N1=0;
constexpr double PI=3.141592653589793;
inline uint32_t mockTime=0;
inline uint32_t millis(){return mockTime;}
inline void delay(unsigned long n){mockTime+=n;}
struct HardwareSerial {
    std::deque<uint8_t> input;
    HardwareSerial(int=0){}
    void begin(int,int=0,int=0,int=0){}
    void setRxBufferSize(int){}
    int available(){return input.size();}
    int read(){if(input.empty()) return -1; int c=input.front();input.pop_front();return c;}
    int availableForWrite(){return 512;}
    void setTxBufferSize(int){}
    size_t write(const uint8_t*, size_t n){return n;}
    template<class... T> void print(T...){}
    template<class... T> void println(T...){}
    template<class... T> void printf(const char*,T...){}
};
inline HardwareSerial Serial;
struct SPIClass {void begin(int,int,int){}};
inline SPIClass SPI;

inline bool ledcAttach(int,int,int){return true;}
inline void ledcWrite(int,int){}
