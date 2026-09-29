#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <esp_log.h>
#include <stdlib.h>

#define T_CLK   25
#define T_MOSI  32
#define T_MISO  39
#define T_CS    33
#define T_IRQ   36

#define XPT_CMD_X   0xD0
#define XPT_CMD_Y   0x90
#define XPT_CMD_Z1  0xB0
#define XPT_CMD_Z2  0xC0

struct TouchCal { float a, b, c, d, e, f; bool valid; };
static TouchCal tcal = {0, 0, 0, 0, 0, 0, false};

static inline void tclk() {
  digitalWrite(T_CLK, HIGH);
  delayMicroseconds(1);
  digitalWrite(T_CLK, LOW);
  delayMicroseconds(1);
}

static uint16_t xptRead(uint8_t cmd) {
  digitalWrite(T_CS, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(T_MOSI, (cmd >> i) & 1);
    tclk();
  }
  tclk();
  uint16_t v = 0;
  for (int i = 0; i < 12; i++) {
    digitalWrite(T_CLK, HIGH);
    delayMicroseconds(1);
    v = (uint16_t)((v << 1) | (digitalRead(T_MISO) & 1));
    digitalWrite(T_CLK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(T_CS, HIGH);
  return v;
}

static void touchBegin() {
  pinMode(T_CLK, OUTPUT);
  pinMode(T_MOSI, OUTPUT);
  pinMode(T_CS, OUTPUT);
  pinMode(T_MISO, INPUT);
  pinMode(T_IRQ, INPUT);
  digitalWrite(T_CS, HIGH);
  digitalWrite(T_CLK, LOW);
}

static int cmp16(const void *p, const void *q) {
  return (int)(*(const uint16_t *)p) - (int)(*(const uint16_t *)q);
}

#define TOUCH_SAMPLES 7
static bool touchRaw(uint16_t &rx, uint16_t &ry) {
  if (digitalRead(T_IRQ) == HIGH) return false;
  uint16_t z1 = xptRead(XPT_CMD_Z1);
  uint16_t z2 = xptRead(XPT_CMD_Z2);
  int z = (int)z1 + 4095 - (int)z2;
  if (z < 600) return false;

  xptRead(XPT_CMD_Y);
  xptRead(XPT_CMD_X);

  uint16_t xs[TOUCH_SAMPLES], ys[TOUCH_SAMPLES];
  for (int i = 0; i < TOUCH_SAMPLES; i++) {
    ys[i] = xptRead(XPT_CMD_Y);
    xs[i] = xptRead(XPT_CMD_X);
  }
  qsort(xs, TOUCH_SAMPLES, sizeof(uint16_t), cmp16);
  qsort(ys, TOUCH_SAMPLES, sizeof(uint16_t), cmp16);
  rx = xs[TOUCH_SAMPLES / 2];
  ry = ys[TOUCH_SAMPLES / 2];
  return rx > 100 && ry > 100 && rx < 4000 && ry < 4000;
}

static bool touchGet(int &sx, int &sy) {
  uint16_t rx, ry;
  if (!touchRaw(rx, ry)) return false;
  if (!tcal.valid) return false;
  sx = (int)lroundf(tcal.a * rx + tcal.b * ry + tcal.c);
  sy = (int)lroundf(tcal.d * rx + tcal.e * ry + tcal.f);
  return true;
}

static bool fitAxis(const float *rx, const float *ry, const float *out, int n,
                    float &p, float &q, float &r) {
  if (n < 3) return false;
  double M[3][4] = {{0}};
  for (int i = 0; i < n; i++) {
    double v[3] = {rx[i], ry[i], 1.0};
    for (int a = 0; a < 3; a++) {
      for (int b = 0; b < 3; b++) M[a][b] += v[a] * v[b];
      M[a][3] += v[a] * out[i];
    }
  }
  for (int col = 0; col < 3; col++) {
    int piv = col;
    for (int r2 = col + 1; r2 < 3; r2++)
      if (fabs(M[r2][col]) > fabs(M[piv][col])) piv = r2;
    if (fabs(M[piv][col]) < 1e-9) return false;
    if (piv != col) for (int k = 0; k < 4; k++) { double t = M[col][k]; M[col][k] = M[piv][k]; M[piv][k] = t; }
    for (int r2 = 0; r2 < 3; r2++) {
      if (r2 == col) continue;
      double f = M[r2][col] / M[col][col];
      for (int k = col; k < 4; k++) M[r2][k] -= f * M[col][k];
    }
  }
  p = (float)(M[0][3] / M[0][0]);
  q = (float)(M[1][3] / M[1][1]);
  r = (float)(M[2][3] / M[2][2]);
  return true;
}

static void touchSave() {
  Preferences pr;
  pr.begin("tvchandset", false);
  pr.putBytes("tcal", &tcal, sizeof(tcal));
  pr.end();
}

static bool touchLoad() {
  esp_log_level_set("ARDUINO", ESP_LOG_NONE);
  Preferences pr;
  pr.begin("tvchandset", true);
  bool ok = pr.getBytesLength("tcal") == sizeof(tcal) &&
            pr.getBytes("tcal", &tcal, sizeof(tcal)) == sizeof(tcal);
  pr.end();
  esp_log_level_set("ARDUINO", ESP_LOG_ERROR);
  if (!ok) tcal.valid = false;
  return tcal.valid;
}

static void touchDefault() {
  tcal.a = 0.0f;      tcal.b = 320.0f / 3500.0f;  tcal.c = -200.0f * 320.0f / 3500.0f;
  tcal.d = -240.0f / 3600.0f; tcal.e = 0.0f;      tcal.f = 240.0f + 300.0f * 240.0f / 3600.0f;
  tcal.valid = true;
}
