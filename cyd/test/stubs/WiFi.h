#pragma once
#include <Arduino.h>
#define WIFI_STA 1
struct WiFiStub { void mode(int) {} void disconnect(bool, bool) {} };
extern WiFiStub WiFi;
