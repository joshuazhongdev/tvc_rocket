//linkage.h - four-bar linkage solver for a TVC gimbal
//converts gimbal angle to servo angle by solving the geometry instead of a fixed ratio
//S = servo shaft, G = gimbal pivot
//A = pushrod hole on the horn, hornR from S
//B = pushrod hole on the arm, armR from G
//pushrod A-B, length rodL
//A is where two circles intersect: radius hornR about S, radius rodL about B
//see linkage.py for the derivation
//each servo has its own Linkage struct, independent pitch/yaw geometry
#pragma once
#include <Arduino.h>

//limits are expressed in gimbal degrees, not servo degrees
//defined here so bench.cpp uses the same values
#ifndef SERVO_CENTER
#define SERVO_CENTER    90.0f
#endif
#ifndef SERVO_SWING_DEG
#define SERVO_SWING_DEG 30.0f
#endif

//90 always represents centre
//servoTrim = SERVO_CENTER + servoOffset, derived by linkageInit()
//servo window is +/- SERVO_SWING_DEG around each axis's own trim
//measured 2026-09-10, both offsets +10, ratio remains above 0.30 across the working range
#ifndef MAX_GIMBAL_DEG
#define MAX_GIMBAL_DEG  10.0f
#endif

struct Linkage {
  //measured with calipers, in millimetres
  float hornR;          //servo shaft to pushrod hole in the horn
  float rodL;            //pushrod, hole to hole
  float armR;            //gimbal pivot to pushrod hole in the arm
  float pivotX, pivotY;  //gimbal pivot position, relative to the servo shaft

  float armNeutral;   //gimbal arm angle, degrees, when the nozzle is centred
  float servoOffset;  //measured degrees from SERVO_CENTER to centred, tuned per axis
  bool  branchUp;      //selects which circle intersection this linkage uses
  float dir;            //+1 or -1, sets the sign of positive deflection

  //assigned by linkageInit(), not set manually
  float servoTrim;
  float hornNeutral;
  bool  ready;
};

//returns the horn angle, in degrees, that places the arm at absolute angle armDeg
//returns false if the pushrod cannot reach that position
inline bool linkageHornAngle(const Linkage &k, float armDeg, float &hornDeg) {
  const float a = armDeg * PI / 180.0f;
  const float px = k.pivotX + k.armR * cosf(a);
  const float py = k.pivotY + k.armR * sinf(a);
  const float d = sqrtf(px * px + py * py);
  if (d < 1e-6f) return false;

  const float x1 = (k.hornR * k.hornR + d * d - k.rodL * k.rodL) / (2.0f * d);
  const float h2 = k.hornR * k.hornR - x1 * x1;
  /*
  !(h2 >= 0) is used instead of (h2 < 0) because NaN comparisons are always
  false, so h2 < 0 would allow a NaN to reach servo.write() through a
  pass-through constrain().
  */
  if (!(h2 >= 0.0f)) return false;
  const float y1 = (k.branchUp ? 1.0f : -1.0f) * sqrtf(h2);

  const float ang = atan2f(py, px);
  const float ca = cosf(ang), sa = sinf(ang);
  hornDeg = atan2f(x1 * sa + y1 * ca, x1 * ca - y1 * sa) * 180.0f / PI;
  return true;
}

//called once at startup
//derives servoTrim and the neutral horn angle
inline bool linkageInit(Linkage &k) {
  k.servoTrim = SERVO_CENTER + k.servoOffset;
  k.ready = linkageHornAngle(k, k.armNeutral, k.hornNeutral);
  return k.ready;
}

inline float linkageServoMin(const Linkage &k) { return k.servoTrim - SERVO_SWING_DEG; }
inline float linkageServoMax(const Linkage &k) { return k.servoTrim + SERVO_SWING_DEG; }

//returns the horn travel from neutral required to deflect the gimbal by deflectionDeg
inline bool linkageHornMove(const Linkage &k, float deflectionDeg, float &hornMoveDeg) {
  if (!k.ready) return false;
  float h;
  if (!linkageHornAngle(k, k.armNeutral + deflectionDeg, h)) return false;
  hornMoveDeg = h - k.hornNeutral;
  return true;
}

inline bool linkageHornMoveBetween(const Linkage &k, float fromDefl, float toDefl,
                                   float &hornMoveDeg) {
  float a, b;
  if (!linkageHornMove(k, fromDefl, a)) return false;
  if (!linkageHornMove(k, toDefl, b)) return false;
  hornMoveDeg = b - a;
  return true;
}

//returns the value to pass to servo.write()
//used for the PID output
inline bool linkageServoAngle(const Linkage &k, float deflectionDeg, float &servoDeg) {
  float move;
  if (!linkageHornMove(k, deflectionDeg, move)) return false;
  servoDeg = k.servoTrim + k.dir * move;
  if (!(servoDeg > -1000.0f && servoDeg < 1000.0f)) return false;  //rejects NaN/inf
  return true;
}

//local gearing at a given deflection: gimbal degrees per horn degree
//not constant, varies with deflection
inline bool linkageRatio(const Linkage &k, float deflectionDeg, float &ratio) {
  const float e = 0.25f;
  float lo, hi;
  if (!linkageHornMove(k, deflectionDeg - e, lo)) return false;
  if (!linkageHornMove(k, deflectionDeg + e, hi)) return false;
  const float dh = hi - lo;
  if (fabsf(dh) < 1e-6f) return false;
  ratio = (2.0f * e) / dh;
  return true;
}

//determines whether this deflection is reachable without passing through a lockup
//past a toggle point the two-circle solve returns the mirror-image solution, valid geometry on the wrong side of the linkage
//increments outward in small steps, rejects any deflection past the first failure
inline bool linkageReachable(const Linkage &k, float deflectionDeg, float step = 0.5f) {
  if (!k.ready) return false;
  const float sgn = deflectionDeg >= 0.0f ? 1.0f : -1.0f;
  const float end = fabsf(deflectionDeg);
  float h;
  for (float x = 0.0f; x < end; x += step)
    if (!linkageHornMove(k, sgn * x, h)) return false;
  return linkageHornMove(k, deflectionDeg, h);
}

//maximum deflection before lockup or the servo limits
//computed by scanning outward in 0.1 degree increments
inline void linkageRange(const Linkage &k, float servoMin, float servoMax,
                         float &minDefl, float &maxDefl, float searchLimit = 45.0f) {
  minDefl = maxDefl = 0.0f;
  float s;
  const int steps = (int)(searchLimit * 10.0f + 0.5f);
  for (int i = 1; i <= steps; i++) {
    float x = i * 0.1f;
    if (!linkageServoAngle(k, x, s) || s < servoMin || s > servoMax) break;
    maxDefl = x;
  }
  for (int i = 1; i <= steps; i++) {
    float x = -i * 0.1f;
    if (!linkageServoAngle(k, x, s) || s < servoMin || s > servoMax) break;
    minDefl = x;
  }
}

//constrains the requested deflection to the reachable range, then converts
//used in flight so a large PID output saturates cleanly instead of failing
inline float linkageServoClamped(const Linkage &k, float deflectionDeg,
                                 float minDefl, float maxDefl) {
  const float wanted = constrain(deflectionDeg, minDefl, maxDefl);
  float s;
  if (linkageServoAngle(k, wanted, s)) return s;
  return k.servoTrim;
}
