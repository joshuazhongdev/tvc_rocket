#pragma once
#include <Arduino.h>
enum class textdatum_t { top_left, top_center, middle_center };
struct LovyanGFX {
  void fillRect(int,int,int,int,int) {}
  void fillScreen(int) {}
  void fillRoundRect(int,int,int,int,int,int) {}
  void drawRoundRect(int,int,int,int,int,int) {}
  void drawCircle(int,int,int,int) {}
  void fillCircle(int,int,int,int) {}
  void drawFastHLine(int,int,int,int) {}
  void drawFastVLine(int,int,int,int) {}
  void setTextColor(int,int) {}
  void setTextSize(int) {}
  void setTextDatum(textdatum_t) {}
  void drawString(const char*,int,int) {}
};
struct LGFX_Device : LovyanGFX {
  void init() {} void setRotation(int) {} void setBrightness(int) {}
};
struct LGFX_Sprite : LovyanGFX {
  LGFX_Sprite(void*) {}
  void setColorDepth(int) {}
  void *createSprite(int,int) { static char b; return &b; }
  void pushSprite(int,int) {}
};
