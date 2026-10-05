//bench test suite for the TVC stack
//shares main.cpp's estimator; benchRun() is called from main.cpp on 't'
//run before the static fire: tests 2-4 produce the config numbers, test 8 moves servos
//console redirect must precede every other include: redefines Serial as `con` for USB + handset output
#define TVC_USE_CONSOLE
#include "console.h"

#include <Arduino.h>
#include <string.h>
#include <ESP32Servo.h>
#include "flightlog.h"
#include "linkage.h"

// --- shared with main.cpp --------------------------------------------------
extern Servo servoPitch, servoYaw;
extern Linkage linkPitch, linkYaw;
extern float pitchDeflMin, pitchDeflMax, yawDeflMin, yawDeflMax;
extern float A[3], G[3], V[3];
extern float kalAnglePitch, kalAngleYaw, rawPitch, rawYaw;
extern float gyroBiasRaw[3];
extern int8_t mapIdx[3], mapSign[3];
extern int16_t accRaw[3], gyrRaw[3];
extern float Q_angle, Q_bias, R_measure;
extern float ACC_LSB_PER_G;
extern bool  benchForceGyroOnly;   //forces the flight configuration on the bench
extern bool  benchActive;          //inhibits arming and boost while the bench runs
extern float mixPP, mixPY, mixYP, mixYY;   //the axis mix currently compiled in
extern float gimbalPitch, gimbalYaw;       //attitude already in gimbal axes
extern float lastCmdP, lastCmdY;           //what pidControl last asked the gimbal for
extern bool  accelTrusted;
extern bool  armed, hasZeroed;
extern uint32_t AUTO_ARM_STILL_MS;
bool readRawIMU();
void applyAxisMap();
void updateAttitude();
void pidControl(float targetPitch, float targetYaw, float kP, float kI, float kD);

//gains used by the live demo, changed over serial with p/i/d commands
static float benchKp = 2.0f, benchKi = 0.10f, benchKd = 0.20f;

// ---------------------------------------------------------------------------
static void banner() {
  Serial.println("#");
  Serial.println("# ================= TVC BENCH =================");
  Serial.printf ("# zeroed %-3s | armed %-3s | accel %-10s | gains %.2f %.2f %.2f\n",
                 hasZeroed ? "yes" : "no", armed ? "yes" : "no",
                 benchForceGyroOnly ? "GYRO ONLY" : "accel+gyro",
                 benchKp, benchKi, benchKd);
  Serial.printf ("# gimbal reach  pitch %+.1f to %+.1f   yaw %+.1f to %+.1f deg\n",
                 pitchDeflMin, pitchDeflMax, yawDeflMin, yawDeflMax);
  Serial.println("#");
  Serial.println("# TESTS   run in listed order");
  Serial.println("#   1  axis + sign map     run first; may require a reflash");
  Serial.println("#   2  servo travel        binding, deflection at limits");
  Serial.println("#   3  IMU health          magnitude, noise, residual bias");
  Serial.println("#   4  filter noise        30 s, servos still");
  Serial.println("#   5  filter + servos     10 s, servos sweeping, coupling");
  Serial.println("#   6  gyro-only drift     30 s, FLIGHT CONFIG");
  Serial.println("#   7  gyro-only return    tilt away and back");
  Serial.println("#   8  closed loop         SERVOS MOVE. needs 1 applied first");
  Serial.println("#");
  Serial.println("# GIMBAL  G<deg> | G<pitch>,<yaw>   geometry solved");
  Serial.println("#         W<radius>                 envelope walk, 12 bearings");
  Serial.println("#         S<servo> | S<p>,<y>       RAW servo, model bypassed");
  Serial.println("#");
  Serial.println("# LOG     L dump  F list  E erase  M save now  T<unix> clock");
  Serial.println("#         C live CSV");
  Serial.println("# SET     p/i/d <num> gains   g gyro-only   ? menu   q exit");
  Serial.println("#         letters: either case, except G and g,");
  Serial.println("#         which are distinct commands");
  Serial.println("#");
}


//running mean and standard deviation without keeping the samples
struct Stat {
  double n = 0, mean = 0, m2 = 0, first = 0, last = 0;
  void add(double x) {
    if (n == 0) first = x;
    last = x;
    n++;
    double d = x - mean;
    mean += d / n;
    m2 += d * (x - mean);
  }
  double sd() const { return n > 1 ? sqrt(m2 / (n - 1)) : 0.0; }
  double drift() const { return last - first; }
};

static float tiltFromArrow() {
  float c = constrain(V[2], -1.0f, 1.0f);
  return acosf(c) * 180.0f / PI;
}

//parks both servos at neutral before a measurement; a deflected axis tilts the thrust line
static void centreGimbal(uint16_t settleMs = 600) {
  servoPitch.write(linkPitch.servoTrim);
  servoYaw.write(linkYaw.servoTrim);
  delay(settleMs);
}

// --- 1: servo travel -------------------------------------------------------
static void testServoTravel() {
  //sweeps each axis around its measured neutral, not a hardcoded 90
  Servo *s[2] = {&servoPitch, &servoYaw};
  Linkage *lk[2] = {&linkPitch, &linkYaw};
  const char *nm[2] = {"pitch", "yaw"};
  for (int k = 0; k < 2; k++) {
    const int mid = (int)lroundf(lk[k]->servoTrim);
    const int lo  = (int)lroundf(linkageServoMin(*lk[k]));
    const int hi  = (int)lroundf(linkageServoMax(*lk[k]));
    Serial.printf("# %s: neutral %d (%+.1f from centre %.0f), sweeping %d to %d\n",
                  nm[k], mid, lk[k]->servoOffset, SERVO_CENTER, lo, hi);
    for (int a = mid; a >= lo; a--) { s[k]->write(a); delay(25); }
    delay(300);
    for (int a = lo; a <= hi; a++)  { s[k]->write(a); delay(25); }
    delay(300);
    for (int a = hi; a >= mid; a--) { s[k]->write(a); delay(25); }
    delay(300);
  }
  Serial.println("# check: binding, buzz at ends, linkage fouling");
  Serial.println("# both axes back at measured neutral");
  Serial.println("# nozzle square by eye; if not, adjust servoOffset not servoTrim");
  Serial.println("# nozzle_defl: measure at both ends");
  Serial.println("# compare against travel from boot banner");
}

// --- 2: IMU health ---------------------------------------------------------
static void testImuHealth() {
  Serial.println("# imu_health: 300 samples, HOLD STILL");
  Stat am, gx, gy, gz;
  double axs = 0, ays = 0, azs = 0;
  int n = 0, fails = 0;
  unsigned long t0 = millis();
  for (int i = 0; i < 300; i++) {
    if (readRawIMU()) {
      applyAxisMap();
      float mag = sqrtf(A[0]*A[0] + A[1]*A[1] + A[2]*A[2]) / ACC_LSB_PER_G;
      am.add(mag);
      axs += A[0]; ays += A[1]; azs += A[2];
      gx.add(G[0]); gy.add(G[1]); gz.add(G[2]);
      n++;
    } else fails++;
    delay(5);
  }
  float hz = n * 1000.0f / (millis() - t0);
  Serial.printf("# samples %d, i2c failures %d, effective rate %.0f Hz\n", n, fails, hz);
  Serial.printf("# accel magnitude %.4f g  (want 1.000 +/- 0.03)\n", am.mean);
  Serial.printf("# canonical accel mean  %.0f %.0f %.0f LSB  (want ~0 0 %.0f)\n",
                axs/n, ays/n, azs/n, ACC_LSB_PER_G);
  Serial.printf("# gyro noise sd  %.2f %.2f %.2f deg/s  (want under ~1.5 at rest)\n",
                gx.sd(), gy.sd(), gz.sd());
  Serial.printf("# gyro residual  %.2f %.2f %.2f deg/s  (want under ~0.3 after calibration)\n",
                gx.mean, gy.mean, gz.mean);
  Serial.printf("# axis map idx %d %d %d  sign %d %d %d\n",
                mapIdx[0], mapIdx[1], mapIdx[2], mapSign[0], mapSign[1], mapSign[2]);
  if (fabs(am.mean - 1.0) > 0.03)
    Serial.println("# WARN accel magnitude off, check range register or LSB constant");
  if (gx.sd() > 3 || gy.sd() > 3 || gz.sd() > 3)
    Serial.println("# WARN gyro noisy, check DLPF and vibration");
  if (fabs(gx.mean) > 0.5 || fabs(gy.mean) > 0.5 || fabs(gz.mean) > 0.5)
    Serial.println("# WARN gyro bias remains, recalibrate while still");
}

// --- 3 and 4: filter noise and drift ---------------------------------------
static void filterNoiseRun(uint32_t seconds, bool shakeServos) {
  Serial.printf("# filter_run %lu s, %s, %s. DO NOT TOUCH\n",
                (unsigned long)seconds, shakeServos ? "servos sweeping" : "servos still",
                accelTrusted ? "accel + gyro" : "GYRO ONLY");
  Stat rp, ry, kp, ky, tilt;
  unsigned long t0 = millis(), lastImu = 0;
  while (millis() - t0 < seconds * 1000UL) {
    unsigned long now = micros();
    if (now - lastImu >= 5000) {
      lastImu = now;
      updateAttitude();
      rp.add(rawPitch); ry.add(rawYaw);
      kp.add(kalAnglePitch); ky.add(kalAngleYaw);
      tilt.add(tiltFromArrow());
    }
    if (shakeServos) {
      float ph = (millis() - t0) * 0.004f;          //about 2 Hz
      //shakes about each servo's own neutral, not a hardcoded 90
      servoPitch.write(linkPitch.servoTrim + 15 * sinf(ph * PI));
      servoYaw.write(linkYaw.servoTrim + 15 * cosf(ph * PI));
    }
    if ((millis() - t0) % 5000 < 2) Serial.printf("# %lus\n", (millis() - t0) / 1000);
  }
  if (shakeServos) { servoPitch.write(linkPitch.servoTrim);
                     servoYaw.write(linkYaw.servoTrim); }

  Serial.println("#            noise_sd (deg)   drift_over_run (deg)");
  Serial.printf("# raw pitch      %7.3f            %7.3f\n", rp.sd(), rp.drift());
  Serial.printf("# raw yaw       %7.3f            %7.3f\n", ry.sd(), ry.drift());
  Serial.printf("# kal pitch      %7.3f            %7.3f\n", kp.sd(), kp.drift());
  Serial.printf("# kal yaw       %7.3f            %7.3f\n", ky.sd(), ky.drift());
  Serial.printf("# arrow tilt     %7.3f            %7.3f\n", tilt.sd(), tilt.drift());
  Serial.printf("# noise reduction: pitch %.1fx, yaw %.1fx\n",
                kp.sd() > 0 ? rp.sd() / kp.sd() : 0.0, ky.sd() > 0 ? ry.sd() / ky.sd() : 0.0);
  Serial.println("# R_measure higher: smoother, more drift. lower: reverse");
  Serial.println("# record these values against the tuning in use");
}

// --- 5: live closed loop ---------------------------------------------------
static void testClosedLoop() {
  centreGimbal();
  Serial.printf ("# gimbal centred at measured neutral: pitch %.1f, yaw %.1f\n",
                 linkPitch.servoTrim, linkYaw.servoTrim);
  Serial.println("# SERVOS MOVE. hands clear. hold airframe.");
  Serial.printf("# gains kP %.2f  kI %.2f  kD %.2f\n", benchKp, benchKi, benchKd);
  Serial.println("# tilt by hand; gimbal opposes");
  Serial.println("# slam-to-stop and hold = TVC sign inverted");
  for (int i = 3; i > 0; i--) { Serial.printf("# %d...\n", i); delay(1000); }
  //discards input queued during the countdown
  while (Serial.available() > 0) Serial.read();
  Serial.println("# sign check: cmdP opposes gimPitch,");
  Serial.println("# cmdY opposes gimYaw. same sign = that axis");
  Serial.println("# drives the rocket over instead of catching it");
  Serial.println("# t_ms,gimPitch,gimYaw,arrowTilt,accelG,cmdP,cmdY,svP,svY");

  unsigned long t0 = millis(), lastImu = 0, lastPrint = 0;
  double ccPP = 0, ccYY = 0, ccN = 0;
  char stopKey = 0;                       //0 = ran to time, else what stopped it
  while (millis() - t0 < 30000UL) {
    if (Serial.available() > 0) {
      char k = Serial.read();
      if (k != '\n' && k != '\r') { stopKey = k; break; }   //EOL is not a keypress
    }
    unsigned long now = micros();
    if (now - lastImu >= 5000) {
      lastImu = now;
      updateAttitude();
      pidControl(0, 0, benchKp, benchKi, benchKd);
      //scores only while tilted past 5 deg; sign of tilt times command is what matters
      //no mixing here: attitude is already in gimbal axes; re-applying the matrix would double-rotate it
      float eP = gimbalPitch;
      float eY = gimbalYaw;
      if (fabsf(eP) > 5.0f || fabsf(eY) > 5.0f) {
        ccPP += eP * lastCmdP;
        ccYY += eY * lastCmdY;
        ccN  += 1.0;
      }
    }
    if (now - lastPrint >= 20000) {
      lastPrint = now;
      float mag = sqrtf(A[0]*A[0] + A[1]*A[1] + A[2]*A[2]) / ACC_LSB_PER_G;
      float svP, svY;
      if (!linkageServoAngle(linkPitch, lastCmdP, svP)) svP = linkPitch.servoTrim;
      if (!linkageServoAngle(linkYaw,   lastCmdY, svY)) svY = linkYaw.servoTrim;
      Serial.printf("%lu,%.2f,%.2f,%.2f,%.3f,%.2f,%.2f,%.1f,%.1f\n",
                    millis() - t0, gimbalPitch, gimbalYaw, tiltFromArrow(), mag,
                    lastCmdP, lastCmdY, svP, svY);
    }
  }
  servoPitch.write(linkPitch.servoTrim);
  servoYaw.write(linkYaw.servoTrim);
  unsigned long ran = millis() - t0;
  if (stopKey)
    Serial.printf("# stopped %lu ms, key '%c' (0x%02X). not a fault\n"
                  "# any character stops this test\n"
                  "# re-run with 8, no input until done\n",
                  ran, (stopKey >= 32 && stopKey < 127) ? stopKey : '?', (unsigned)stopKey);
  else
    Serial.printf("# ran full %lu ms, servos centred\n", ran);

  //correlation between tilt and command; negative means the gimbal opposed the tilt
  if (ccN > 20) {
    Serial.printf("# corr tilt-vs-cmd: pitch %+.2f, yaw %+.2f  (%d scored samples)\n",
                  ccPP / ccN, ccYY / ccN, (int)ccN);
    Serial.println("# both must be NEGATIVE = gimbal opposing tilt");
    if (ccPP > 0) Serial.println("# FAIL pitch positive feedback. DO NOT FLY. recheck mix matrix");
    if (ccYY > 0) Serial.println("# FAIL yaw positive feedback. DO NOT FLY. recheck mix matrix");
  } else {
    //too few samples passed the tilt threshold, not that the test failed
    Serial.printf("# NO VERDICT: %d samples past the 5 deg scoring threshold\n"
                  "# 20 needed. tilt further\n"
                  "# hold each tilt 1-2 s, no waving\n", (int)ccN);
  }
}

// --- 6: raw CSV log --------------------------------------------------------
static void testCsvLog() {
  Serial.println("# t_ms,rawPitch,rawYaw,kalPitch,kalYaw,vx,vy,vz,accelG,gx,gy,gz");
  unsigned long t0 = millis(), lastImu = 0, lastPrint = 0;
  while (true) {
    if (Serial.available() > 0) {
      char k = Serial.read();
      if (k != '\n' && k != '\r') break;      //line endings are not keypresses
    }
    unsigned long now = micros();
    if (now - lastImu >= 5000) { lastImu = now; updateAttitude(); }
    if (now - lastPrint >= 10000) {
      lastPrint = now;
      float mag = sqrtf(A[0]*A[0] + A[1]*A[1] + A[2]*A[2]) / ACC_LSB_PER_G;
      Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.3f,%.2f,%.2f,%.2f\n",
                    millis() - t0, rawPitch, rawYaw, kalAnglePitch, kalAngleYaw,
                    V[0], V[1], V[2], mag, G[0], G[1], G[2]);
    }
  }
  Serial.println("# log stopped");
}

// --- 7: gyro-only drift, the flight configuration --------------------------
//accelerometer is off during the burn; this 30 s drift is the flight error budget
static void testGyroOnlyDrift() {
  bool saved = benchForceGyroOnly;
  Serial.println("# settling 3 s, accel on");
  benchForceGyroOnly = false;
  unsigned long t0 = millis(), lastImu = 0;
  while (millis() - t0 < 3000) {
    unsigned long now = micros();
    if (now - lastImu >= 5000) { lastImu = now; updateAttitude(); }
  }
  float p0 = kalAnglePitch, r0 = kalAngleYaw, v0 = tiltFromArrow();

  Serial.println("# accel OFF. hold still 30 s");
  benchForceGyroOnly = true;
  t0 = millis();
  while (millis() - t0 < 30000UL) {
    unsigned long now = micros();
    if (now - lastImu >= 5000) { lastImu = now; updateAttitude(); }
    if ((millis() - t0) % 5000 < 2)
      Serial.printf("# %lus  pitch %+.2f  yaw %+.2f  tilt %+.2f\n",
                    (millis() - t0) / 1000, kalAnglePitch - p0,
                    kalAngleYaw - r0, tiltFromArrow() - v0);
  }
  benchForceGyroOnly = saved;

  float dp = kalAnglePitch - p0, dr = kalAngleYaw - r0, dv = tiltFromArrow() - v0;
  Serial.println("# --- gyro_only_drift over 30 s ---");
  Serial.printf("# angle pitch %+.2f deg   yaw %+.2f deg\n", dp, dr);
  Serial.printf("# arrow tilt  %+.2f deg\n", dv);
  Serial.printf("# scaled to 0.8 s burn: pitch %+.3f  yaw %+.3f  tilt %+.3f deg\n",
                dp * 0.8f / 30.0f, dr * 0.8f / 30.0f, dv * 0.8f / 30.0f);
  Serial.println("# WARN over ~5 deg in 30 s = gyro cal taken while moving");
  Serial.println("# recalibrate while still");
}

// --- 8: gyro-only return to mark -------------------------------------------
//tests gyro scale factor: a nonzero residual means the deg/s constant is wrong for the range in use
static void testGyroOnlyReturn() {
  bool saved = benchForceGyroOnly;
  Serial.println("# settling 3 s, accel on. rocket on mark");
  benchForceGyroOnly = false;
  unsigned long t0 = millis(), lastImu = 0;
  while (millis() - t0 < 3000) {
    unsigned long now = micros();
    if (now - lastImu >= 5000) { lastImu = now; updateAttitude(); }
  }
  float p0 = kalAnglePitch, r0 = kalAngleYaw;

  Serial.println("# accel OFF. 20 s: tilt well over, wave around,");
  Serial.println("# then return exactly to mark and hold");
  benchForceGyroOnly = true;
  t0 = millis();
  while (millis() - t0 < 20000UL) {
    unsigned long now = micros();
    if (now - lastImu >= 5000) { lastImu = now; updateAttitude(); }
    if ((millis() - t0) % 5000 < 2) Serial.printf("# %lus\n", (millis() - t0) / 1000);
  }
  benchForceGyroOnly = saved;

  Serial.println("# --- residual after return to mark ---");
  Serial.printf("# pitch %+.2f deg   yaw %+.2f deg\n",
                kalAnglePitch - p0, kalAngleYaw - r0);
  Serial.println("# under ~2 deg is good. residual growing with tilt size =");
  Serial.println("# GYRO_SENSITIVITY wrong for the range in use");
}

// --- 9: axis and sign mapping ----------------------------------------------
//servo-to-axis mapping is not auto-detected and must be measured
//under thrust the nose swings opposite the nozzle, same way the tail does
static void holdUntilKey() {
  unsigned long lastImu = 0;
  while (Serial.available() <= 0) {
    unsigned long now = micros();
    if (now - lastImu >= 5000) { lastImu = now; updateAttitude(); }
  }
  while (Serial.available() > 0) Serial.read();
}

struct AxisProbe { float dPitch, dYaw; };

//probes via GIMBAL command, not raw servo angle, so probe size is trim-independent and exercises Linkage::dir
static AxisProbe probeServo(Servo &s, Linkage &k, const char *name) {
  centreGimbal();                  //both axes neutral, not just this one
  Serial.println("#");
  Serial.println("# gimbal centred. hold rocket vertical, launch attitude");
  Serial.println("# key to continue");
  holdUntilKey();
  float p0 = kalAnglePitch, r0 = kalAngleYaw;

  const float probeDeg = 8.0f;     //well inside the 10 deg envelope, easy to see
  float sv;
  if (!linkageServoAngle(k, probeDeg, sv)) {
    Serial.printf("# %s: linkage cannot reach %+.0f deg, axis not probed\n",
                  name, probeDeg);
    return AxisProbe{0.0f, 0.0f};
  }
  s.write(sv);
  Serial.printf("# %s: nozzle commanded %+.0f deg (servo %.1f). SERVOS MOVE.\n"
                "# hands clear. note nozzle aim\n", name, probeDeg, sv);
  Serial.println("# under thrust the nose swings the same way");
  Serial.println("# leave servo. TILT WHOLE ROCKET, nose that way");
  Serial.println("# tilt 20-30 deg, hold, key to continue");
  Serial.println("# only tilt DIRECTION is read. magnitude is signal");
  Serial.println("# strength only: bigger reads easier, need not match");
  Serial.println("# the other axis");
  holdUntilKey();

  AxisProbe r = { kalAnglePitch - p0, kalAngleYaw - r0 };
  centreGimbal();
  Serial.printf("# measured change: pitch %+.1f deg, yaw %+.1f deg\n", r.dPitch, r.dYaw);
  return r;
}

static void testAxisMapping() {
  Serial.println("# ===== axis and sign mapping =====");
  Serial.println("# 2 probes, one per servo. SERVOS MOVE. hands clear.");
  Serial.println("#");
  centreGimbal(1000);
  Serial.printf ("# servos at measured neutral: pitch %.1f, yaw"
                 " %.1f\n", linkPitch.servoTrim, linkYaw.servoTrim);
  Serial.println("# expect nozzle square with airframe, no deflection,");
  Serial.println("# aimed down the body axis");
  Serial.println("# sight along tube to check");
  Serial.println("#");
  Serial.println("# if visibly off: adjust servoOffset");
  Serial.println("# in setupLinkages(), or move the horn one spline for a large");
  Serial.println("# error. every value below is");
  Serial.println("# relative to this neutral; a crooked neutral puts the");
  Serial.println("# same error into the test and every flight after");
  Serial.println("#");
  Serial.println("# gimbal straight, key to continue");
  holdUntilKey();

  AxisProbe pp = probeServo(servoPitch, linkPitch, "PITCH");
  AxisProbe yp = probeServo(servoYaw,   linkYaw,   "YAW");

  // ---- interpret the two probe vectors -------------------------------------
  //each probe gives a vector in (pitch, yaw) space: direction the nose moved for that servo
  //checks whether the two vectors are perpendicular: a genuine two-axis gimbal, just rotated between frames
  const float pMag = sqrtf(pp.dPitch*pp.dPitch + pp.dYaw*pp.dYaw);
  const float yMag = sqrtf(yp.dPitch*yp.dPitch + yp.dYaw*yp.dYaw);

  Serial.println("#");
  Serial.println("# ===== RESULT =====");
  if (pMag < 8.0f || yMag < 8.0f) {
    Serial.printf("# tilts too small: %.1f and %.1f deg. re-run\n",
                  pMag, yMag);
    Serial.println("# tilt 20-30 deg each probe");
    centreGimbal();
    return;
  }

  const float pAng = atan2f(pp.dYaw, pp.dPitch) * 180.0f / PI;
  const float yAng = atan2f(yp.dYaw, yp.dPitch) * 180.0f / PI;
  float sep = yAng - pAng;
  while (sep >  180.0f) sep -= 360.0f;
  while (sep < -180.0f) sep += 360.0f;

  Serial.printf("# pitch servo -> %5.1f deg of tilt, bearing %+7.1f\n", pMag, pAng);
  Serial.printf("# yaw   servo -> %5.1f deg of tilt, bearing %+7.1f\n", yMag, yAng);
  Serial.printf("# separation %+.1f deg (90 or -90 is a clean two-axis gimbal)\n", sep);
  Serial.printf("# tilt sizes %.0f and %.0f deg (hand tilt, not gimbal)\n"
                "#             bearings above carry the information\n", pMag, yMag);

  if (fabsf(fabsf(sep) - 90.0f) > 30.0f) {
    Serial.println("#");
    Serial.println("# WARN axes not perpendicular. either both tilts were in");
    Serial.println("# similar directions, or there is a mechanical fault.");
    Serial.println("# re-run before trusting any value below");
    centreGimbal();
    return;
  }

  //orthogonalises before inverting: separation error from 90 deg is attributed to hand-tilt, split evenly between bearings
  //result is orthogonal, so its inverse is its transpose
  const float sgn  = (sep > 0.0f) ? 1.0f : -1.0f;
  const float derr = sep - sgn * 90.0f;
  const float pC = (pAng + derr * 0.5f) * PI / 180.0f;
  const float yC = (yAng - derr * 0.5f) * PI / 180.0f;
  float m[2][2] = {{ cosf(pC), sinf(pC) },
                   { cosf(yC), sinf(yC) }};
  const float det = m[0][0]*m[1][1] - m[0][1]*m[1][0];
  Serial.printf("# squared up by %+.1f deg; matrix determinant %+.3f\n", derr, det);

  Serial.println("#");
  Serial.printf("# imu rotation about nose: %+.0f deg relative\n", pAng);
  Serial.printf("# to gimbal%s. not a fault, undone in software\n",
                det < 0 ? ", handedness flipped" : "");
  Serial.println("# paste into main.cpp, next to the PID:");
  Serial.println("#");
  Serial.printf("#     mixPP = %+.4ff;  mixPY = %+.4ff;\n", m[0][0], m[0][1]);
  Serial.printf("#     mixYP = %+.4ff;  mixYY = %+.4ff;\n", m[1][0], m[1][1]);
  Serial.println("#");
  Serial.println("# keep linkPitch.dir and linkYaw.dir at +1; a reversed axis shows");
  Serial.println("# as a negative diagonal term above, matrix handles it");
  Serial.println("#");
  // ---- how is the currently compiled matrix doing? --------------------------
  //drives servos directly, measuring raw hardware rotation; pidControl() is what applies the mix, not this test
  //compiled matrix applied to these probe vectors to see how much energy lands on the wrong axis
  Serial.println("#");
  Serial.println("# ----- compiled matrix performance -----");
  float worst = 0.0f;
  const float vec[2][2] = {{ pp.dPitch, pp.dYaw }, { yp.dPitch, yp.dYaw }};
  const char *vn[2] = { "pitch probe", "yaw probe  " };
  for (int i = 0; i < 2; i++) {
    float mp = mixPP * vec[i][0] + mixPY * vec[i][1];
    float my = mixYP * vec[i][0] + mixYY * vec[i][1];
    float mm = sqrtf(mp*mp + my*my);
    float leak = (mm > 1e-6f) ? 100.0f * fminf(fabsf(mp), fabsf(my)) / mm : 0.0f;
    if (leak > worst) worst = leak;
    Serial.printf("#   %s -> (%+6.1f, %+6.1f)   cross-axis leak %4.1f%%\n",
                  vn[i], mp, my, leak);
  }
  Serial.println("#");
  if (worst < 12.0f) {
    Serial.printf("# worst leak %.1f%%: flight-acceptable, within hand-tilt spread\n", worst);
    Serial.println("# further tuning fits noise");
    Serial.println("# next test 8: gimbal must push back on hand tilt, not");
    Serial.println("# slam to a stop and hold");
  } else {
    Serial.printf("# worst leak %.1f%%: paste matrix above, reflash, re-run\n", worst);
  }
  centreGimbal();
}

// ---------------------------------------------------------------------------
static float readNumber() {
  char buf[16];
  int n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 2000 && n < 15) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') break;
      buf[n++] = c;
      t0 = millis();
    }
  }
  buf[n] = 0;
  return atof(buf);
}

//reads up to two numbers separated by comma or space; "5" -> 5,5; returns count found
static int readTwoNumbers(float &a, float &b) {
  char buf[32];
  int n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 2000 && n < 31) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') break;
      buf[n++] = c;
      t0 = millis();
    }
  }
  buf[n] = 0;
  //drains CRLF so a leftover newline isn't read as a keypress later
  {
    unsigned long td = millis();
    while (millis() - td < 30) {
      if (Serial.available() > 0) {
        int pk = Serial.peek();
        if (pk == '\n' || pk == '\r') { Serial.read(); td = millis(); }
        else break;
      } else {
        delay(1);
      }
    }
  }
  a = b = 0.0f;
  if (n == 0) return 0;
  char *sep = strpbrk(buf, ", ");
  if (sep) {
    *sep = 0;
    a = atof(buf);
    b = atof(sep + 1);
    return 2;
  }
  a = b = atof(buf);
  return 1;
}

//drives one axis and reports; shared by G and the envelope walk
static bool gimbalStep(Linkage &k, Servo &sv, const char *name, float g, bool quiet) {
  float s2, r;
  if (!linkageReachable(k, g)) {
    if (!quiet) Serial.printf("#   %s  UNREACHABLE, locks up before this angle\n", name);
    return false;
  }
  if (!linkageServoAngle(k, g, s2)) {
    if (!quiet) Serial.printf("#   %s  LOCKUP\n", name);
    return false;
  }
  sv.write(constrain(s2, 0.0f, 180.0f));
  if (!quiet) {
    Serial.printf("#   %s  servo %6.1f", name, s2);
    if (linkageRatio(k, g, r)) Serial.printf("   ratio %.3f", r);
    if (s2 < linkageServoMin(k) || s2 > linkageServoMax(k))
      Serial.print("   *** OUTSIDE SERVO WINDOW ***");
    Serial.println();
  }
  return true;
}

void benchRun() {
  //claims the vehicle for the duration; cleared on every exit path below
  benchActive = true;
  armed = false;
  banner();
  while (true) {
    if (Serial.available() <= 0) { delay(20); continue; }
    char c = Serial.read();

    //swallows the line ending before dispatch, so it isn't read as a keypress by an abort check
    if (c != 'G' && c != 'W' && c != 'S' && c != 'L' && c != 'E' && c != 'T' &&
        c != 'g' && c != 'w' && c != 's' && c != 'l' && c != 'e' &&
        c != 'p' && c != 'i' && c != 'd') {
      uint32_t td = millis();
      while (millis() - td < 25) {
        if (Serial.available() > 0) {
          int pk = Serial.peek();
          if (pk == '\n' || pk == '\r') { Serial.read(); td = millis(); }
          else break;
        } else delay(1);
      }
    }

    switch (c) {
      case '1': testAxisMapping();        break;
      case '2': testServoTravel();        break;
      case '3': testImuHealth();          break;
      case '4': filterNoiseRun(30, false); break;
      case '5': filterNoiseRun(10, true);  break;
      case '6': testGyroOnlyDrift();      break;
      case '7': testGyroOnlyReturn();     break;
      case '8': testClosedLoop();         break;
      case 'c':
      case 'C': testCsvLog();             break;
      case '9': Serial.println("# 9 removed. tests 1-8, in run order. ? menu");
                break;
      //G takes degrees of thrust deflection, not servo degrees; four-bar solved per axis
      //G<num> sets both axes to the same angle; G<pitch>,<yaw> sets them independently
      //unclamped by MAX_GIMBAL_DEG on purpose, to find where the hardware stops
      case 'G': {
        float gp, gy;
        readTwoNumbers(gp, gy);
        float mag = sqrtf(gp * gp + gy * gy);
        Serial.printf("# gimbal  pitch %+.1f  yaw %+.1f   combined tilt %.1f deg\n",
                      gp, gy, mag);
        if (mag > MAX_GIMBAL_DEG + 0.05f)
          Serial.printf("#   note: past %.0f deg flight limit, ok for probing\n"
                        "#         PID never commands this much\n",
                        MAX_GIMBAL_DEG);
        gimbalStep(linkPitch, servoPitch, "pitch", gp, false);
        gimbalStep(linkYaw,   servoYaw,   "yaw  ", gy, false);
        break;
      }

      //W<radius>: steps once around a circle of that radius in 30 deg increments, pausing at each
      case 'w':
      case 'W': {
        float rad, dummy;
        if (readTwoNumbers(rad, dummy) == 0 || rad <= 0.0f) rad = MAX_GIMBAL_DEG;
        Serial.printf("# envelope walk %.1f deg, 12 points, 1 s each. SERVOS MOVE. "
                      "hands clear. any key aborts\n", rad);
        Serial.println("#  bearing   pitch     yaw    reachable");
        bool aborted = false;
        for (int deg = 0; deg < 360 && !aborted; deg += 30) {
          float a = deg * PI / 180.0f;
          float gp = rad * cosf(a), gy = rad * sinf(a);
          bool okP = gimbalStep(linkPitch, servoPitch, "pitch", gp, true);
          bool okY = gimbalStep(linkYaw,   servoYaw,   "yaw  ", gy, true);
          Serial.printf("#   %4d   %+6.1f  %+6.1f    %s%s\n", deg, gp, gy,
                        okP ? "pitch ok " : "PITCH NO ",
                        okY ? "yaw ok"    : "YAW NO");
          unsigned long t0 = millis();
          while (millis() - t0 < 1000) {
            if (Serial.available() > 0) {
              char k = Serial.read();
              if (k != '\n' && k != '\r') { aborted = true; break; }
            }
            delay(10);
          }
        }
        servoPitch.write(linkPitch.servoTrim);
        servoYaw.write(linkYaw.servoTrim);
        Serial.println(aborted ? "# walk aborted, centred."
                               : "# walk complete, centred.");
        break;
      }
      case 'l':
      case 'L': { float v = readNumber(); logDump(v > 0 ? (int)v : -1); break; }
      case 'f':
      case 'F': logList(); break;

      //saves the RAM ring to flash now; flash otherwise only gets written after boost+coast latches
      //a clamped static fire never leaves the pad state, since thrust and clamp cancel and accel reads 1 g
      case 'm':
      case 'M': {
        int sl = logSaveNow(true);
        if (sl >= 0)
          Serial.printf("# log: saved to slot %d, %lu samples "
                        "(last %.1f s). reboot safe\n",
                        sl, (unsigned long)logCount, logCount / 200.0f);
        break;
      }
      case 'e':
      case 'E': { float v = readNumber(); logErase(v > 0 ? (int)v : -1); break; }
      case 'T': { float v = readNumber(); if (v > 0) logSetEpoch((uint32_t)v); break; }
      case 'g':
        benchForceGyroOnly = !benchForceGyroOnly;
        Serial.printf("# %s\n", benchForceGyroOnly ? "GYRO ONLY" : "accel + gyro");
        break;
      case 'p': benchKp = readNumber(); Serial.printf("# kP %.3f\n", benchKp); break;
      case 'i': benchKi = readNumber(); Serial.printf("# kI %.3f\n", benchKi); break;
      case 'd': benchKd = readNumber(); Serial.printf("# kD %.3f\n", benchKd); break;
      case 'q':
        benchActive = false;
        benchForceGyroOnly = false;
        servoPitch.write(linkPitch.servoTrim);
        servoYaw.write(linkYaw.servoTrim);
        Serial.println("# bench exit");
        return;
      case '\n': case '\r': continue;
      case 'z':
      case 'Z': banner(); continue;
      case '?': banner(); continue;
      case 't': continue;   //readlog.py sends 't' to enter bench; ignore it here

      //S<num> or S<pitch>,<yaw>: raw servo degrees, bypassing the linkage model
      //bounded so a typo cannot slam a servo into a hard stop
      case 's':
      case 'S': {
        float sp, sy;
        readTwoNumbers(sp, sy);
        //wider than the flight window on purpose; still bounded against a typo
        sp = constrain(sp, SERVO_CENTER - 50.0f, SERVO_CENTER + 50.0f);
        sy = constrain(sy, SERVO_CENTER - 50.0f, SERVO_CENTER + 50.0f);
        servoPitch.write(sp);
        servoYaw.write(sy);
        Serial.printf("# RAW servo  pitch %.1f  yaw %.1f   (no geometry applied)\n", sp, sy);

        //walks the model backwards to find which deflection it maps this servo angle to
        for (int i = 0; i < 2; i++) {
          Linkage &k = i ? linkYaw : linkPitch;
          float want = i ? sy : sp;
          const char *nm = i ? "yaw  " : "pitch";
          float bestD = 0, bestErr = 1e9f, sv;
          //0.01 deg steps: near a toggle point a coarser grid would misreport reachable angles as UNREACHABLE
          for (float d = -25.0f; d <= 25.0f; d += 0.01f) {
            if (!linkageReachable(k, d)) continue;
            if (!linkageServoAngle(k, d, sv)) continue;
            if (fabsf(sv - want) < bestErr) { bestErr = fabsf(sv - want); bestD = d; }
          }
          if (bestErr < 0.3f)
            Serial.printf("#   %s  model: %+.1f deg of nozzle\n", nm, bestD);
          else
            Serial.printf("#   %s  model: UNREACHABLE\n", nm);
        }
        Serial.println("#   nozzle_deg: measure both axes");
        break;
      }
      default:
        Serial.printf("# unknown command '%c'. ? menu\n", c);
        continue;
    }
    //no banner() here: reprinting the menu after every command would bury a stepped sweep like G-2 G-4 G-6
  }
}
