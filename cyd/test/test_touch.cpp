#include <Arduino.h>
#include <Preferences.h>
uint32_t g_millis = 0;
int g_touchIrq = HIGH;
uint16_t g_rawX = 0, g_rawY = 0;
SerialStub Serial;
#include "touch.h"

static int fails = 0, checks = 0;
static void ck(bool c, const char *what) { checks++; if (!c) { fails++; printf("  FAIL  %s\n", what); } }

static void trial(const char *name, int mode) {
  const float SX[4] = {28, 292, 292, 28};
  const float SY[4] = {28, 28, 212, 212};
  float rx[4], ry[4];
  for (int i = 0; i < 4; i++) {
    float u = SX[i] / 320.0f, v = SY[i] / 240.0f;
    float a = 300 + u * 3600, b = 200 + v * 3500;
    switch (mode) {
      case 0: rx[i] = a;        ry[i] = b;        break;
      case 1: rx[i] = 3900 - a; ry[i] = b;        break;
      case 2: rx[i] = a;        ry[i] = 3700 - b; break;
      case 3: rx[i] = b;        ry[i] = a;        break;
      case 4: rx[i] = 3700 - b; ry[i] = 3900 - a; break;
    }
  }
  TouchCal c = {};
  bool ok = fitAxis(rx, ry, SX, 4, c.a, c.b, c.c) && fitAxis(rx, ry, SY, 4, c.d, c.e, c.f);
  ck(ok, name);
  if (!ok) return;
  float worst = 0;
  for (int i = 0; i < 4; i++) {
    float px = c.a * rx[i] + c.b * ry[i] + c.c;
    float py = c.d * rx[i] + c.e * ry[i] + c.f;
    worst = fmaxf(worst, fmaxf(fabsf(px - SX[i]), fabsf(py - SY[i])));
  }
  char m[96]; snprintf(m, sizeof(m), "%s recovers the corners (worst %.2f px)", name, worst);
  ck(worst < 1.0f, m);
}

int main() {
  printf("-- affine touch calibration --\n");
  trial("aligned", 0);
  trial("x mirrored", 1);
  trial("y mirrored", 2);
  trial("axes swapped", 3);
  trial("swapped and flipped", 4);

  float rx[4] = {1000,1000,1000,1000}, ry[4] = {1000,1000,1000,1000};
  float out[4] = {0,1,2,3};
  float a,b,c2;
  ck(!fitAxis(rx, ry, out, 4, a, b, c2), "degenerate input is refused, not fitted");

  printf("\n-- nine point fit on a bowed panel --\n");
  {
    const int N = 9;
    float rx[N], ry[N], sx[N], sy[N];
    int k = 0;
    for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++, k++) {
      float u = c * 0.5f, v = r * 0.5f;
      sx[k] = 28 + u * 264; sy[k] = 28 + v * 184;
      float bowx = 1.0f + 0.015f * (v - 0.5f) * (v - 0.5f) * 4;
      float bowy = 1.0f + 0.015f * (u - 0.5f) * (u - 0.5f) * 4;
      rx[k] = (300 + u * 3600) * bowx;
      ry[k] = (200 + v * 3500) * bowy;
    }
    TouchCal c = {};
    bool ok = fitAxis(rx, ry, sx, N, c.a, c.b, c.c) && fitAxis(rx, ry, sy, N, c.d, c.e, c.f);
    ck(ok, "nine point fit succeeds");
    float worst = 0;
    for (int i = 0; i < N; i++) {
      float px = c.a*rx[i] + c.b*ry[i] + c.c, py = c.d*rx[i] + c.e*ry[i] + c.f;
      worst = fmaxf(worst, fmaxf(fabsf(px - sx[i]), fabsf(py - sy[i])));
    }
    char m[96];
    snprintf(m, sizeof(m), "a bowed panel shows a non-zero residual (worst %.1f px)", worst);
    ck(worst > 1.0f, m);
  }
  {
    const int N = 9;
    float rx[N], ry[N], sx[N], sy[N];
    int k = 0;
    for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++, k++) {
      float u = c * 0.5f, v = r * 0.5f;
      sx[k] = 28 + u * 264; sy[k] = 28 + v * 184;
      rx[k] = 3900 - (300 + u * 3600);
      ry[k] = 200 + v * 3500;
    }
    TouchCal c = {};
    fitAxis(rx, ry, sx, N, c.a, c.b, c.c); fitAxis(rx, ry, sy, N, c.d, c.e, c.f);
    float worst = 0;
    for (int i = 0; i < N; i++) {
      float px = c.a*rx[i] + c.b*ry[i] + c.c, py = c.d*rx[i] + c.e*ry[i] + c.f;
      worst = fmaxf(worst, fmaxf(fabsf(px - sx[i]), fabsf(py - sy[i])));
    }
    char m[96];
    snprintf(m, sizeof(m), "a flat mirrored panel fits nine points cleanly (worst %.2f px)", worst);
    ck(worst < 0.5f, m);
  }

  printf("%d checks, %d failures\n", checks, fails);
  return fails ? 1 : 0;
}
