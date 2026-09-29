#pragma once
#ifndef TVC_TOUCH_STUB
#define TVC_TOUCH_STUB
#include <Arduino.h>
#include <Preferences.h>
struct TouchCal { float a,b,c,d,e,f; bool valid; };
extern TouchCal tcal;
extern bool g_touched; extern int g_tx, g_ty;
inline void touchBegin() {}
inline bool touchRaw(uint16_t &x, uint16_t &y) { x = g_rawX; y = g_rawY; return g_touched; }
inline bool touchGet(int &x, int &y) { x = g_tx; y = g_ty; return g_touched; }
inline void touchDefault() { tcal.valid = true; }
inline bool touchLoad() { return tcal.valid; }
inline void touchSave() {}
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
#endif
