#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <string>
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0
#define F(x) x
#define HEX 16
#define DEC 10
typedef uint8_t byte;
extern uint32_t g_millis;
extern int g_pinState[64];
inline void pinMode(uint8_t,uint8_t){}
inline void digitalWrite(uint8_t p,uint8_t v){ if(p<64) g_pinState[p]=v; }
inline int  digitalRead(uint8_t p){ return (p<64)?g_pinState[p]:1; }
inline uint32_t millis(){ return g_millis; }
inline void delay(uint32_t){}
inline void delayMicroseconds(uint32_t){}
inline void noInterrupts(){}
inline void interrupts(){}
template<class T,class A,class B> T constrain(T v,A lo,B hi){ return v<(T)lo?(T)lo:(v>(T)hi?(T)hi:v); }
using std::min; using std::max;
struct SerialStub {
  void begin(uint32_t){}
  void setTX(uint8_t){} void setRX(uint8_t){}
  int available(){return 0;} int read(){return -1;}
  void print(const char*s){ printf("%s",s); }
  void print(int v){ printf("%d",v); }
  void print(long v){ printf("%ld",v); }
  void print(unsigned v){ printf("%u",v); }
  void print(unsigned long v){ printf("%lu",v); }
  void print(unsigned long v,int base){ printf(base==16?"%lX":"%lu",v); }
  void print(bool v){ printf("%d",v?1:0); }
  void print(char c){ printf("%c",c); }
  void print(float v,int d=2){ printf("%.*f",d,v); }
  void print(double v,int d=2){ printf("%.*f",d,v); }
  void println(){ printf("\n"); }
  void println(const char*s){ printf("%s\n",s); }
  void println(int v){ printf("%d\n",v); }
  void println(long v){ printf("%ld\n",v); }
  void println(unsigned v){ printf("%u\n",v); }
  void println(float v,int d=2){ printf("%.*f\n",d,v); }
  void println(double v,int d=2){ printf("%.*f\n",d,v); }
  void reset_input_buffer(){}
};
typedef SerialStub Stream;
extern SerialStub Serial, Serial1, Serial2;
