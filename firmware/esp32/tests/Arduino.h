#pragma once
// Minimal host-test Arduino interface. This header is not used by board builds.
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>

#define PI 3.14159265358979323846
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2

static uint32_t mockMillis=0;
static int outputWrites=0, outputPinSetups=0;
inline uint32_t millis() { return mockMillis; }
inline uint32_t micros() { return mockMillis*1000; }
inline void delay(unsigned long ms) { mockMillis+=ms; }
inline void delayMicroseconds(unsigned int) {}
inline void pinMode(int, int mode) { if(mode==OUTPUT) ++outputPinSetups; }
inline void digitalWrite(int, int) { ++outputWrites; }
inline int digitalRead(int) { return HIGH; }
inline int analogRead(int) { return 0; }
inline void analogReadResolution(int) {}

struct MockSerial {
  char lastLine[512]={};
  void begin(unsigned long) {}
  void flush() {}
  int available() { return 0; }
  char read() { return 0; }
  void println(const char *s) { snprintf(lastLine,sizeof(lastLine),"%s",s); }
  void printf(const char *format,...) {
    va_list args; va_start(args,format);
    vsnprintf(lastLine,sizeof(lastLine),format,args); va_end(args);
  }
};
static MockSerial Serial;
