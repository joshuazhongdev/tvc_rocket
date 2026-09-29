#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <string>
typedef uint8_t byte;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define PI 3.14159265358979323846f

extern uint32_t g_millis;
inline uint32_t millis() { return g_millis; }
inline uint32_t micros() { return g_millis * 1000; }
inline void delay(uint32_t ms) { g_millis += ms; }
inline void delayMicroseconds(uint32_t) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
extern int g_touchIrq;
extern uint16_t g_rawX, g_rawY;
inline int digitalRead(int p) { return p == 36 ? g_touchIrq : 0; }
template <class T> T constrain(T v, T a, T b) { return v < a ? a : v > b ? b : v; }
inline long lroundf(float f) { return (long)::lroundf(f); }

struct SerialStub {
  void begin(unsigned long) {}
  template <class... A> void printf(const char *f, A... a) { ::printf(f, a...); }
  void println(const char *s = "") { ::printf("%s\n", s); }
  void print(const char *s) { ::printf("%s", s); }
};
extern SerialStub Serial;

typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) do{(void)(x);}while(0)
#define portEXIT_CRITICAL(x)  do{(void)(x);}while(0)
#define ESP_ARDUINO_VERSION_MAJOR 2
