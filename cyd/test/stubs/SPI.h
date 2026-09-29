#pragma once
#include <Arduino.h>
#define VSPI 3
struct SPIClass { SPIClass(int) {} void begin(int,int,int,int) {} };
