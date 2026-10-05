//redefines Serial as `con`, routing output over USB and the wireless handset; must precede flightlog.h
#define TVC_USE_CONSOLE
#include "console.h"

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Preferences.h>
#include "linkage.h"
#include "flightlog.h"

//the S3 variant header already defines this; ours is the board's actual LED pin
#undef LED_BUILTIN
#define LED_BUILTIN 2
#define SERVOPITCH_PIN 5
#define SERVOYAW_PIN 6
#define MPU9250_ADDR 0x68
#define SDA_PIN 16
#define SCL_PIN 15

#define IMU_PERIOD_US    5000   //200 Hz filter update
#define PRINT_PERIOD_US 20000   //50 Hz serial stream

Servo servoPitch;
Servo servoYaw;

void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_BUILTIN, LOW);
    delay(delayMs);
  }
}

int16_t accRaw[3], gyrRaw[3];          //x, y, z as the chip reports them
float   gyroBiasRaw[3] = {0, 0, 0};    //LSB, measured while still

//remaps sensor axes into a canonical frame (Z along the nose), built in calibrateOrientation()
int8_t mapIdx[3]  = {0, 1, 2};
int8_t mapSign[3] = {1, 1, 1};
float  A[3];   //canonical accel, raw LSB
float  G[3];   //canonical gyro rate, deg/s, bias removed

float kalAnglePitch = 0, kalAngleYaw = 0;
float gyroBiasPitch = 0, gyroBiasYaw = 0;
float P_pitch[2][2] = {{1, 0}, {0, 1}};
float P_yaw[2][2]   = {{1, 0}, {0, 1}};
float pitchOffset = 0, yawOffset = 0;
unsigned long lastKalUs = 0;

//wide sensor ranges in flight: the narrow ranges rail under boost (5-11 g, easily >250 dps), losing the gyro attitude data too
#define FLIGHT_MODE 1

#if FLIGHT_MODE
  #define ACCEL_FS_SEL 0x18                //+/-16 g
  #define GYRO_FS_SEL  0x10                //+/-1000 dps
  float ACC_LSB_PER_G    = 2048.0f;
  float GYRO_SENSITIVITY = 32.8f;
#else
  #define ACCEL_FS_SEL 0x00                //+/-2 g
  #define GYRO_FS_SEL  0x00                //+/-250 dps
  float ACC_LSB_PER_G    = 16384.0f;
  float GYRO_SENSITIVITY = 131.0f;
#endif

//accelTrusted is true only in STATE_PAD; latched, so a mid-flight tumble through 1 g cannot reset it
enum FlightState { STATE_PAD, STATE_BOOST, STATE_COAST };
FlightState flightState = STATE_PAD;
unsigned long boostStartMs = 0;
unsigned long coastAtMs = 0;
//two-stage flush: stage 1 at 300 ms post-burnout, stage 2 adds the coast and deletes stage 1
uint32_t LOG_FLUSH1_MS = 300;
uint32_t LOG_FLUSH2_MS = 8000;
int      logStage1Slot = -1;
bool     stage1Tried = false;
uint32_t logStage1Uptime = 0;

//thresholds sized for a 495 g airframe; recompute for a different mass or motor
float    BOOST_G         = 1.8f;
float    BURNOUT_G       = 1.0f;
uint16_t BOOST_CONFIRM   = 8;       //40 ms at 200 Hz
uint16_t BURNOUT_CONFIRM = 20;      //100 ms
uint32_t MAX_BURN_MS     = 1500;    //backstop, a C11 burns about 800 ms

bool accelTrusted       = true;
bool benchForceGyroOnly = false;

bool benchActive = false;   //true only while benchRun() is executing

bool  tvcEnabled = true;
float flightKp = 2.0f, flightKi = 0.10f, flightKd = 0.20f;

void pidReset();

//linkage.h solves the four-bar geometry from PID gimbal degrees to a servo angle
//measured by hand, millimetres, each axis in its own servo frame
Linkage linkPitch, linkYaw;
float pitchDeflMin = 0, pitchDeflMax = 0;
float yawDeflMin   = 0, yawDeflMax   = 0;

//manual trim, clamped to +/-25 deg, persisted to flash via O<pitch>,<yaw>
#define TRIM_LIMIT_DEG 25.0f
static Preferences trimNvs;

static void trimLoad() {
  trimNvs.begin("tvctrim", true);
  linkPitch.servoOffset = trimNvs.getFloat("offP", linkPitch.servoOffset);
  linkYaw.servoOffset   = trimNvs.getFloat("offY", linkYaw.servoOffset);
  bool stored = trimNvs.isKey("offP");
  trimNvs.end();
  if (stored)
    Serial.printf("# trim loaded from flash: pitch %+.2f  yaw %+.2f\n",
                  linkPitch.servoOffset, linkYaw.servoOffset);
}

static void trimSave() {
  trimNvs.begin("tvctrim", false);
  trimNvs.putFloat("offP", linkPitch.servoOffset);
  trimNvs.putFloat("offY", linkYaw.servoOffset);
  trimNvs.end();
}

void setupLinkages() {
  linkPitch.hornR = 18.4f;   linkPitch.rodL = 18.4f;   linkPitch.armR = 33.5f;
  linkPitch.pivotX = 19.5f;  linkPitch.pivotY = -19.9f;
  linkPitch.armNeutral = 103.0f; linkPitch.servoOffset = +10.0f;
  linkPitch.branchUp = true;     linkPitch.dir = +1.0f;        //confirm with test 1

  linkYaw.hornR = 18.1f;     linkYaw.rodL = 26.2f;     linkYaw.armR = 35.0f;
  linkYaw.pivotX = 25.6f;    linkYaw.pivotY = 33.8f;
  linkYaw.armNeutral = 252.0f;   linkYaw.servoOffset = +10.0f;
  linkYaw.branchUp = false;      linkYaw.dir = +1.0f;          //confirm with test 1

  trimLoad();   //overrides the compiled values above

  bool okP = linkageInit(linkPitch);
  bool okY = linkageInit(linkYaw);
  if (okP) linkageRange(linkPitch, linkageServoMin(linkPitch), linkageServoMax(linkPitch),
                        pitchDeflMin, pitchDeflMax, MAX_GIMBAL_DEG);
  if (okY) linkageRange(linkYaw,   linkageServoMin(linkYaw),   linkageServoMax(linkYaw),
                        yawDeflMin,   yawDeflMax,   MAX_GIMBAL_DEG);

  Serial.printf("# Gimbal cap %.1f deg. Centre is %.0f; "
                "pitch neutral %.1f (%+.1f), yaw neutral %.1f (%+.1f)\n",
                MAX_GIMBAL_DEG, SERVO_CENTER,
                linkPitch.servoTrim, linkPitch.servoOffset,
                linkYaw.servoTrim,   linkYaw.servoOffset);
  Serial.printf("# Linkage pitch: %s, travel %.1f to %.1f deg of gimbal\n",
                okP ? "ok" : "DOES NOT CLOSE", pitchDeflMin, pitchDeflMax);
  Serial.printf("# Linkage yaw:   %s, travel %.1f to %.1f deg of gimbal\n",
                okY ? "ok" : "DOES NOT CLOSE", yawDeflMin, yawDeflMax);
  if (!okP || !okY)
    Serial.println("# WARNING: falling back to raw servo degrees. Check the dimensions.");
  if (okP && (pitchDeflMax < MAX_GIMBAL_DEG - 0.15f || -pitchDeflMin < MAX_GIMBAL_DEG - 0.15f))
    Serial.println("# NOTE: pitch cannot reach the gimbal cap. Linkage or servo limit binds first.");
  if (okY && (yawDeflMax < MAX_GIMBAL_DEG - 0.15f || -yawDeflMin < MAX_GIMBAL_DEG - 0.15f))
    Serial.println("# NOTE: yaw cannot reach the gimbal cap. Linkage or servo limit binds first.");
}

static inline void centreGimbal() {
  servoPitch.write(linkPitch.servoTrim);
  servoYaw.write(linkYaw.servoTrim);
}

float Q_angle   = 0.001;
float Q_bias    = 0.003;
float R_measure = 0.03;

float rawPitch = 0, rawYaw = 0;
char  displayMode = 'b';   //'r' raw, 'f' filtered, 'b' both

//gravity-vector ("arrow") estimator: each filter tracks one leg of a vector pointing at the floor, so it has no pole and works upside down too
const float VEC_SCALE = (PI / 180.0f) * (PI / 180.0f);

float V[3]     = {0, 0, 1};   //the arrow, length 1
float vBias[3] = {0, 0, 0};
float P_v[3][2][2] = {{{1, 0}, {0, 1}}, {{1, 0}, {0, 1}}, {{1, 0}, {0, 1}}};

float wrapAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

void setupMPU9250() {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x6B); Wire.write(0x00);      //wake up
  Wire.endTransmission(true);
  delay(100);

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1A); Wire.write(0x03);      //DLPF ~44 Hz, reduces servo buzz
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1B); Wire.write(GYRO_FS_SEL);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1C); Wire.write(ACCEL_FS_SEL);
  Wire.endTransmission(true);
  delay(50);

  Serial.printf("# Ranges: accel %.0f LSB/g, gyro %.1f LSB per deg/s (%s)\n",
                ACC_LSB_PER_G, GYRO_SENSITIVITY,
                FLIGHT_MODE ? "FLIGHT" : "BENCH");
}

//reads all 14 bytes before parsing; byte-by-byte inline reads have unspecified evaluation order before C++17
bool readRawIMU() {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)14, (uint8_t)true) != 14) return false;

  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();

  accRaw[0] = (int16_t)((b[0]  << 8) | b[1]);
  accRaw[1] = (int16_t)((b[2]  << 8) | b[3]);
  accRaw[2] = (int16_t)((b[4]  << 8) | b[5]);
  //b[6], b[7] = temperature
  gyrRaw[0] = (int16_t)((b[8]  << 8) | b[9]);
  gyrRaw[1] = (int16_t)((b[10] << 8) | b[11]);
  gyrRaw[2] = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

void applyAxisMap() {
  for (int k = 0; k < 3; k++) {
    A[k] = mapSign[k] * (float)accRaw[mapIdx[k]];
    G[k] = mapSign[k] * ((float)gyrRaw[mapIdx[k]] - gyroBiasRaw[mapIdx[k]]) / GYRO_SENSITIVITY;
  }
}

void buildAxisMap(int vertIdx, int vertSign) {
  int a = (vertIdx + 1) % 3;
  int b = (vertIdx + 2) % 3;
  mapIdx[0] = a;       mapSign[0] = 1;
  mapIdx[1] = b;       mapSign[1] = vertSign;
  mapIdx[2] = vertIdx; mapSign[2] = vertSign;
}

void calibrateGyro() {
  Serial.println("# Calibrating gyro, keep the rocket still...");
  double s[3] = {0, 0, 0};
  int n = 0;
  for (int i = 0; i < 500; i++) {
    if (readRawIMU()) { s[0] += gyrRaw[0]; s[1] += gyrRaw[1]; s[2] += gyrRaw[2]; n++; }
    delay(3);
  }
  if (n > 0) for (int k = 0; k < 3; k++) gyroBiasRaw[k] = s[k] / n;
  Serial.printf("# Gyro bias LSB: %.1f %.1f %.1f (n=%d)\n",
                gyroBiasRaw[0], gyroBiasRaw[1], gyroBiasRaw[2], n);
}

void calibrateOrientation() {
  Serial.println("# Calibrating orientation, hold the launch position...");
  double s[3] = {0, 0, 0};
  int n = 0;
  for (int i = 0; i < 300; i++) {
    if (readRawIMU()) { s[0] += accRaw[0]; s[1] += accRaw[1]; s[2] += accRaw[2]; n++; }
    delay(5);
  }
  if (n == 0) { Serial.println("# IMU not responding, check wiring."); return; }

  float avg[3];
  for (int k = 0; k < 3; k++) avg[k] = s[k] / n;

  int vertIdx = 0;
  for (int k = 1; k < 3; k++) if (fabs(avg[k]) > fabs(avg[vertIdx])) vertIdx = k;
  int vertSign = (avg[vertIdx] >= 0) ? 1 : -1;
  buildAxisMap(vertIdx, vertSign);

  float Ac[3];
  for (int k = 0; k < 3; k++) Ac[k] = mapSign[k] * avg[mapIdx[k]];

  pitchOffset = atan2(-Ac[0], hypotf(Ac[1], Ac[2])) * 180.0 / PI;
  yawOffset   = atan2( Ac[1], Ac[2]) * 180.0 / PI;

  kalAnglePitch = 0;
  kalAngleYaw   = 0;
  gyroBiasPitch = 0;
  gyroBiasYaw   = 0;

  float mag = sqrtf(Ac[0] * Ac[0] + Ac[1] * Ac[1] + Ac[2] * Ac[2]);
  if (mag > 1.0f) { V[0] = Ac[0] / mag; V[1] = Ac[1] / mag; V[2] = Ac[2] / mag; }
  vBias[0] = vBias[1] = vBias[2] = 0;

  Serial.printf("# Vertical axis: %c%c   offsets pitch %.2f  yaw %.2f\n",
                vertSign > 0 ? '+' : '-', 'x' + vertIdx, pitchOffset, yawOffset);

  //a large pitch offset means the rocket was not vertical during calibration
  if (fabsf(pitchOffset) > 20.0f || fabsf(yawOffset) > 20.0f) {
    Serial.println("# ***********************************************************");
    Serial.println("# WARNING: large orientation offset. If the rocket was not in");
    Serial.println("#          its LAUNCH ATTITUDE just now, the nose axis above");
    Serial.println("#          is wrong and every attitude number is referenced to");
    Serial.println("#          the wrong frame.");
    Serial.println("#          Linkage work (G, W) is unaffected: it never touches");
    Serial.println("#          the IMU. Tests 2,3,4,5,7,8,9 ARE affected. Stand the");
    Serial.println("#          rocket vertical and power cycle before running them.");
    Serial.println("# ***********************************************************");
  }
}

float kalmanFilter1D(float newAngle, float newRate, float dt,
                     float &angle, float &bias, float P[2][2],
                     float Q_angle, float Q_bias, float R_measure) {
  float rate = newRate - bias;
  angle = wrapAngle(angle + dt * rate);

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float y = wrapAngle(newAngle - angle);
  float S = P[0][0] + R_measure;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  angle = wrapAngle(angle + K0 * y);
  bias  += K1 * y;

  float P00 = P[0][0];
  float P01 = P[0][1];
  P[0][0] -= K0 * P00;
  P[0][1] -= K0 * P01;
  P[1][0] -= K1 * P00;
  P[1][1] -= K1 * P01;

  return angle;
}

//predict only, no accelerometer correction; used whenever the accelerometer is not measuring gravity alone
float kalmanPredictOnly(float newRate, float dt, float &angle, float &bias,
                        float P[2][2], float Q_angle, float Q_bias) {
  angle = wrapAngle(angle + dt * (newRate - bias));
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
  return angle;
}

// ---- Estimator 1: two angles ----------------------------------------------
//pitch/yaw are tilt off vertical, not roll; roll is uncontrollable by a single gimballed nozzle and is not estimated
void applyTrim(float offP, float offY) {
  linkPitch.servoOffset = constrain(offP, -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);
  linkYaw.servoOffset   = constrain(offY, -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);

  bool okP = linkageInit(linkPitch);
  bool okY = linkageInit(linkYaw);
  if (okP) linkageRange(linkPitch, linkageServoMin(linkPitch), linkageServoMax(linkPitch),
                        pitchDeflMin, pitchDeflMax, MAX_GIMBAL_DEG);
  if (okY) linkageRange(linkYaw,   linkageServoMin(linkYaw),   linkageServoMax(linkYaw),
                        yawDeflMin,   yawDeflMax,   MAX_GIMBAL_DEG);
  centreGimbal();
  trimSave();

  Serial.printf("# TRIM pitch %+.2f (servo %.1f)  yaw %+.2f (servo %.1f)  saved\n",
                linkPitch.servoOffset, linkPitch.servoTrim,
                linkYaw.servoOffset,   linkYaw.servoTrim);
  Serial.printf("# reach now  pitch %+.1f to %+.1f   yaw %+.1f to %+.1f\n",
                pitchDeflMin, pitchDeflMax, yawDeflMin, yawDeflMax);
}

// ---------------------------------------------------------------------------
//AXIS FRAMES: the IMU frame (kalAnglePitch/Yaw) and the gimbal frame (gimbalPitch/Yaw) differ by a ~47 deg rotation with flipped handedness, measured in bench test 1 and confirmed by test 8
float mixPP = +0.6697f, mixPY = -0.7426f;
float mixYP = -0.7426f, mixYY = -0.6697f;

float gimbalPitch = 0, gimbalYaw = 0;

void updateAngles(float dt) {
  rawYaw   = wrapAngle(atan2( A[1], A[2]) * 180.0 / PI - yawOffset);
  rawPitch = wrapAngle(atan2(-A[0], hypotf(A[1], A[2])) * 180.0 / PI - pitchOffset);

  float rollRad  = kalAngleYaw   * PI / 180.0f;
  float pitchRad = kalAnglePitch * PI / 180.0f;
  float sr = sinf(rollRad), cr = cosf(rollRad);
  float tp = constrain(tanf(pitchRad), -5.0f, 5.0f);   //clamp near the +/-90 pole

  float pitchRate = G[1] * cr - G[2] * sr;
  float rollRate  = G[0] + tp * (G[1] * sr + G[2] * cr);

  if (accelTrusted) {
    kalAnglePitch = kalmanFilter1D(rawPitch, pitchRate, dt,
                                   kalAnglePitch, gyroBiasPitch, P_pitch,
                                   Q_angle, Q_bias, R_measure);
    kalAngleYaw   = kalmanFilter1D(rawYaw, rollRate, dt,
                                   kalAngleYaw, gyroBiasYaw, P_yaw,
                                   Q_angle, Q_bias, R_measure);
  } else {
    kalAnglePitch = kalmanPredictOnly(pitchRate, dt, kalAnglePitch,
                                      gyroBiasPitch, P_pitch, Q_angle, Q_bias);
    kalAngleYaw   = kalmanPredictOnly(rollRate, dt, kalAngleYaw,
                                      gyroBiasYaw, P_yaw, Q_angle, Q_bias);
  }

  gimbalPitch = mixPP * kalAnglePitch + mixPY * kalAngleYaw;
  gimbalYaw   = mixYP * kalAnglePitch + mixYY * kalAngleYaw;
}

// ---- Estimator 2: the arrow -----------------------------------------------
void updateGravityVector(float dt) {
  float wx = G[0] * PI / 180.0f;
  float wy = G[1] * PI / 180.0f;
  float wz = G[2] * PI / 180.0f;

  float rate[3];
  rate[0] = -(wy * V[2] - wz * V[1]);
  rate[1] = -(wz * V[0] - wx * V[2]);
  rate[2] = -(wx * V[1] - wy * V[0]);

  float mag = sqrtf(A[0] * A[0] + A[1] * A[1] + A[2] * A[2]);

  if (!accelTrusted || mag < 0.15f * ACC_LSB_PER_G) {
    for (int k = 0; k < 3; k++) {
      V[k] = kalmanPredictOnly(rate[k], dt, V[k], vBias[k], P_v[k],
                               Q_angle * VEC_SCALE, Q_bias * VEC_SCALE);
    }
  } else {
    for (int k = 0; k < 3; k++) {
      V[k] = kalmanFilter1D(A[k] / mag, rate[k], dt, V[k], vBias[k], P_v[k],
                            Q_angle * VEC_SCALE, Q_bias * VEC_SCALE,
                            R_measure * VEC_SCALE);
    }
  }

  float n = sqrtf(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]);
  if (n > 1e-6f) { V[0] /= n; V[1] /= n; V[2] /= n; }
}

// ---------------- Pad re-zero and arming ----------------
#define USE_IR_ARM 0          //set to 1 once a 38 kHz receiver is fitted
#define IR_PIN 4              //TSOP38238-style output, low while it sees carrier

float    STILL_GYRO_DPS    = 2.0f;
float    STILL_ACC_TOL     = 0.05f;
uint32_t REZERO_STILL_MS   = 2000;
uint32_t REZERO_PERIOD_MS  = 5000;
uint32_t AUTO_ARM_STILL_MS = 20000;

bool     armed        = false;
bool     hasZeroed    = false;
bool     armInhibit   = false;   //set by manual disarm, cleared by manual arm or reboot
bool     forceZero    = false;   //set by K, forces the next stillness window to re-zero
bool     landed       = false;   //without this accelTrusted stays false after coast forever
uint32_t landStillMs  = 0;

//STATIC FIRE mode, armed with B; drives an open-loop sweep on boost instead of the PID
bool     sfMode       = false;
float    sfRadius     = 0;       //measured at arm time, degrees of deflection
float    sfPitchMin = 0, sfPitchMax = 0, sfYawMin = 0, sfYawMax = 0;
float    SF_CIRCLE_HZ = 2.0f;    //bounded by servo slew rate; higher collapses the circle inward
uint32_t sfLastLoudMs = 0;
bool     sfLogFast = false;      //short-circuits LOG_FLUSH2_MS so the burn isn't pushed out of the ring buffer

//STATIC FIRE DETECTION: a clamped vehicle stays near 1 g through the burn, so triggering/settling use deviation from 1 g instead of the flight thresholds
float    SF_TRIG_G    = 0.30f;
uint16_t SF_TRIG_CONF = 3;       //15 ms at 200 Hz
float    SF_QUIET_G   = 0.50f;
uint32_t SF_RUN_MS    = 6000;
uint32_t SF_SETTLE_MS = 1800;
uint32_t SF_MAX_MS    = 9000;    //bounded by the 12.5 s RAM ring buffer
uint32_t SF_ARM_TIMEOUT_MS = 600000;
uint32_t sfArmedAtMs = 0;
bool     bootedInFlt  = false;
uint32_t stillSinceMs = 0;
uint32_t motionSinceMs = 0;
uint32_t lastRezeroMs = 0;
double   zsAcc[3] = {0,0,0}, zsGyr[3] = {0,0,0};
int      zsN = 0;
float    lastCmdP = 0, lastCmdY = 0;
unsigned long lastAttitudeMs = 0;

static void zsReset() {
  zsN = 0;
  zsAcc[0] = zsAcc[1] = zsAcc[2] = 0;
  zsGyr[0] = zsGyr[1] = zsGyr[2] = 0;
}

static void applyRezero() {
  if (zsN < 50) return;
  float avgA[3], avgG[3];
  for (int k = 0; k < 3; k++) { avgA[k] = zsAcc[k] / zsN; avgG[k] = zsGyr[k] / zsN; }

  for (int k = 0; k < 3; k++) gyroBiasRaw[k] = avgG[k];

  int vertIdx = 0;
  for (int k = 1; k < 3; k++) if (fabsf(avgA[k]) > fabsf(avgA[vertIdx])) vertIdx = k;
  int vertSign = (avgA[vertIdx] >= 0) ? 1 : -1;
  buildAxisMap(vertIdx, vertSign);

  float Ac[3];
  for (int k = 0; k < 3; k++) Ac[k] = mapSign[k] * avgA[mapIdx[k]];
  pitchOffset = atan2(-Ac[0], hypotf(Ac[1], Ac[2])) * 180.0 / PI;
  yawOffset   = atan2( Ac[1], Ac[2]) * 180.0 / PI;

  kalAnglePitch = kalAngleYaw = 0;
  gyroBiasPitch = gyroBiasYaw = 0;
  P_pitch[0][0] = P_pitch[1][1] = 1; P_pitch[0][1] = P_pitch[1][0] = 0;
  P_yaw[0][0]   = P_yaw[1][1]   = 1; P_yaw[0][1]   = P_yaw[1][0]   = 0;

  float mag = sqrtf(Ac[0]*Ac[0] + Ac[1]*Ac[1] + Ac[2]*Ac[2]);
  if (mag > 1.0f) { V[0] = Ac[0]/mag; V[1] = Ac[1]/mag; V[2] = Ac[2]/mag; }
  vBias[0] = vBias[1] = vBias[2] = 0;
  for (int k = 0; k < 3; k++) {
    P_v[k][0][0] = P_v[k][1][1] = 1; P_v[k][0][1] = P_v[k][1][0] = 0;
  }

  hasZeroed = true;
  lastRezeroMs = millis();
  Serial.printf("# EVENT rezero  vert %c%c  gyro bias %.1f %.1f %.1f LSB  n=%d\n",
                vertSign > 0 ? '+' : '-', 'x' + vertIdx,
                gyroBiasRaw[0], gyroBiasRaw[1], gyroBiasRaw[2], zsN);
}

#if USE_IR_ARM
//looks for a long LOW burst (500 ms window, twice running) rather than decoding a protocol
static bool irArmSignal() {
  static uint32_t winStart = 0, lowMs = 0, lastMs = 0, hits = 0;
  uint32_t now = millis();
  if (lastMs == 0) lastMs = now;
  if (digitalRead(IR_PIN) == LOW) lowMs += (now - lastMs);
  lastMs = now;
  if (now - winStart >= 500) {
    hits = (lowMs > 300) ? hits + 1 : 0;
    winStart = now; lowMs = 0;
    if (hits >= 2) { hits = 0; return true; }
  }
  return false;
}
#endif

static void updateStillnessAndArming(float magG) {
  uint32_t now = millis();
  float gm = fmaxf(fabsf(G[0]), fmaxf(fabsf(G[1]), fabsf(G[2])));
  bool still = (gm < STILL_GYRO_DPS) && (fabsf(magG - 1.0f) < STILL_ACC_TOL);

  if (flightState != STATE_PAD || bootedInFlt) return;
  if (benchActive) { armed = false; sfMode = false; return; }

  //gross motion disarms; a static fire is exempt, since a clamped stand will not reliably stay still outdoors
  if (!still) {
    if (motionSinceMs == 0) motionSinceMs = now;
    stillSinceMs = 0;
    zsReset();
    if (armed && !sfMode && now - motionSinceMs > 1000) {
      armed = false;
      sfMode = false;
      Serial.println("# EVENT disarmed, rocket moved");
    }
    return;
  }
  motionSinceMs = 0;

  if (stillSinceMs == 0) stillSinceMs = now;
  for (int k = 0; k < 3; k++) { zsAcc[k] += accRaw[k]; zsGyr[k] += gyrRaw[k]; }
  zsN++;

  if (now - stillSinceMs >= REZERO_STILL_MS &&
      (forceZero || now - lastRezeroMs >= REZERO_PERIOD_MS)) {
    if (zsN >= 50) {
      applyRezero();
      zsReset();
      if (forceZero) {
        forceZero = false;
        Serial.println("# EVENT forced calibration complete");
      }
    } else {
      static uint32_t lastMoan = 0;
      if (now - lastMoan > 5000) {
        lastMoan = now;
        Serial.printf("# LOG: re-zero starved, only %d IMU samples in the "
                      "window. Check the I2C wiring.\n", zsN);
      }
    }
  }

  if (!armed && hasZeroed && !armInhibit) {
#if USE_IR_ARM
    if (irArmSignal()) { armed = true; Serial.println("# EVENT armed by IR"); return; }
#endif
    if (now - stillSinceMs >= AUTO_ARM_STILL_MS) {
      armed = true;
      Serial.println("# EVENT armed on stillness timeout");
    }
  }
}

void updateFlightState(float magG) {
  static uint16_t above = 0, below = 0;
  switch (flightState) {
    case STATE_PAD:
      if (benchActive) { above = 0; break; }
      if (sfMode) {
        if (armed && fabsf(magG - 1.0f) > SF_TRIG_G) {
          if (++above >= SF_TRIG_CONF) {
            flightState = STATE_BOOST;
            boostStartMs = millis();
            sfLastLoudMs = millis();
            above = 0;
            Serial.printf("# EVENT static fire ignition (|a-1g| %.2f). Sweeping "
                          "at %.1f deg for at least %.1f s. Circle first.\n",
                          fabsf(magG - 1.0f), sfRadius, SF_RUN_MS / 1000.0f);
            linkSetQuiet(true);
          }
        } else above = 0;
        break;
      }
      if (armed && magG > BOOST_G) {
        if (++above >= BOOST_CONFIRM) {
          flightState = STATE_BOOST;
          boostStartMs = millis();
          above = 0;
          pidReset();
          Serial.println("# EVENT boost");
          if (!sfMode) linkSetQuiet(true);   //muted during boost to protect the control loop from radio interrupt jitter
        }
      } else above = 0;
      break;

    case STATE_BOOST:
      if (sfMode) {
        if (fabsf(magG - 1.0f) > SF_QUIET_G) sfLastLoudMs = millis();
        const uint32_t t = millis() - boostStartMs;
        const bool minDone   = t > SF_RUN_MS;
        const bool quietLong = millis() - sfLastLoudMs > SF_SETTLE_MS;
        if ((minDone && quietLong) || t > SF_MAX_MS) {
          flightState = STATE_COAST;
        }
      } else {
        if (magG < BURNOUT_G) {
          if (++below >= BURNOUT_CONFIRM) flightState = STATE_COAST;
        } else below = 0;
        if (millis() - boostStartMs > MAX_BURN_MS) flightState = STATE_COAST;   //flight-only backstop
      }
      if (flightState == STATE_COAST) {
        centreGimbal();
        pidReset();
        Serial.printf("# EVENT burnout at %lu ms\n", millis() - boostStartMs);
        if (sfMode) {
          sfMode = false;
          sfLogFast = true;
          Serial.println("# EVENT static fire sweep complete. Log writes in "
                         "about 1.5 s, or press SAVE now.");
        }
        coastAtMs = millis();
        linkSetQuiet(false);
      }
      break;

    case STATE_COAST:
      //LANDED: requires a steady 1 g and quiet gyro, which holds under neither a chute nor tumbling
      if (!landed) {
        bool quietNow = fabsf(magG - 1.0f) < 0.12f &&
                        fabsf(G[0]) < 15.0f && fabsf(G[1]) < 15.0f && fabsf(G[2]) < 15.0f;
        if (!quietNow) landStillMs = 0;
        else {
          if (landStillMs == 0) landStillMs = millis();
          else if (millis() - landStillMs > 1500) {
            landed = true;
            Serial.println("# EVENT landed. Accelerometer trusted again, "
                           "attitude will re-converge over a few seconds.");
          }
        }
      }

      if (landed && flightState == STATE_COAST) {
        uint32_t flushBy = (sfLogFast ? 1500UL : LOG_FLUSH2_MS) + 3000UL;
        bool flushDone = !logRunning || (millis() - coastAtMs > flushBy);
        if (flushDone) {
          flightState = STATE_PAD;
          armed       = false;
          sfMode      = false;
          landed      = false;
          landStillMs = 0;
          coastAtMs   = 0;
          above = below = 0;
          stage1Tried = false;
          logStage1Slot = -1;
          sfLogFast = false;
          logStart();
          armInhibit = true;   //next arm must be deliberate
          Serial.println("# EVENT back on the pad. Disarmed, bench available, "
                         "auto-arm blocked until you press ARM or SF ARM.");
        }
      }
      break;
  }
  accelTrusted = (flightState == STATE_PAD || landed) && !benchForceGyroOnly;
}

void updateAttitude() {
  if (!readRawIMU()) return;
  applyAxisMap();

  float magG = sqrtf(A[0] * A[0] + A[1] * A[1] + A[2] * A[2]) / ACC_LSB_PER_G;
  updateStillnessAndArming(magG);
  updateFlightState(magG);
  lastAttitudeMs = millis();

  unsigned long now = micros();
  float dt = (now - lastKalUs) / 1000000.0f;
  lastKalUs = now;
  if (dt <= 0 || dt > 0.5f) dt = IMU_PERIOD_US / 1000000.0f;

  updateAngles(dt);
  updateGravityVector(dt);

  uint8_t flags = (accelTrusted ? 1 : 0) | (armed ? 2 : 0)
                | ((tvcEnabled && flightState == STATE_BOOST) ? 4 : 0);
  logTick((uint8_t)flightState, flags, gimbalPitch, gimbalYaw,
          V, magG, G, lastCmdP, lastCmdY);
}

// ---------------- PID ----------------
float integralPitch = 0, integralYaw = 0;
float lastErrorPitch = 0, lastErrorYaw = 0;
unsigned long lastPidUs = 0;

//seeds lastError with the current error, not zero, since pidReset() runs at the BOOST latch and pidControl() runs microseconds later, so a zeroed lastError would spike the derivative term at ignition
void pidReset() {
  integralPitch = integralYaw = 0;
  lastErrorPitch = -gimbalPitch;
  lastErrorYaw   = -gimbalYaw;
  lastPidUs = micros();
}

void driveGimbal(float cPitch, float cYaw);   //defined below, after pidControl

void pidControl(float targetPitch, float targetYaw, float kP, float kI, float kD) {
  float errorPitch = targetPitch - gimbalPitch;
  float errorYaw   = targetYaw   - gimbalYaw;

  //micros(), not millis(): at a 5 ms control period millis() truncation swings dt +/-20%, which shows up as servo dither in the derivative term
  unsigned long t = micros();
  float dt = (t - lastPidUs) / 1000000.0f;
  lastPidUs = t;
  if (dt <= 0 || dt > 1) dt = IMU_PERIOD_US / 1000000.0f;

  //anti-windup sized so the integral term alone cannot exceed the linkage's travel
  float travelP = fmaxf(fabsf(pitchDeflMin), fabsf(pitchDeflMax));
  float travelY = fmaxf(fabsf(yawDeflMin),   fabsf(yawDeflMax));
  if (travelP < 1e-3f) travelP = 10.0f;
  if (travelY < 1e-3f) travelY = 10.0f;
  float iLimP = travelP / fmaxf(kI, 1e-3f);
  float iLimY = travelY / fmaxf(kI, 1e-3f);
  integralPitch = constrain(integralPitch + errorPitch * dt, -iLimP, iLimP);
  integralYaw   = constrain(integralYaw   + errorYaw   * dt, -iLimY, iLimY);

  float dPitch = (errorPitch - lastErrorPitch) / dt;
  float dYaw   = (errorYaw   - lastErrorYaw)   / dt;
  lastErrorPitch = errorPitch;
  lastErrorYaw   = errorYaw;

  float cPitch = errorPitch * kP + integralPitch * kI + dPitch * kD;   //gimbal deg
  float cYaw   = errorYaw   * kP + integralYaw   * kI + dYaw   * kD;   //gimbal deg

  driveGimbal(cPitch, cYaw);
}

//STATIC FIRE SWEEP: open loop, purely a function of time since ignition, one circle at full radius for the run
void staticFireSweep(uint32_t tMs) {
  const float a = (tMs / 1000.0f) * SF_CIRCLE_HZ * 2.0f * PI;
  driveGimbal(sfRadius * cosf(a), sfRadius * sinf(a));
}

//everything that moves the gimbal goes through here, sharing one round limit, one clamp, one fallback
void driveGimbal(float cPitch, float cYaw) {
  {
    //round limit on the combined deflection, not per axis: clamping each axis independently would permit MAX*sqrt(2) on the diagonal, exactly where the gimbal fouls
    const float cap = sfMode ? fmaxf(sfRadius, MAX_GIMBAL_DEG) : MAX_GIMBAL_DEG;
    float mag = sqrtf(cPitch * cPitch + cYaw * cYaw);
    if (mag > cap && mag > 1e-6f) {
      float k = cap / mag;
      cPitch *= k;
      cYaw   *= k;
    }
  }

  lastCmdP = cPitch; lastCmdY = cYaw;

  if (linkPitch.ready && linkYaw.ready) {
    const float pLo = sfMode ? sfPitchMin : pitchDeflMin;
    const float pHi = sfMode ? sfPitchMax : pitchDeflMax;
    const float yLo = sfMode ? sfYawMin   : yawDeflMin;
    const float yHi = sfMode ? sfYawMax   : yawDeflMax;
    servoPitch.write(linkageServoClamped(linkPitch, cPitch, pLo, pHi));
    servoYaw.write(  linkageServoClamped(linkYaw,   cYaw,   yLo, yHi));
  } else {
    //fallback if the linkage is unmeasured or invalid; dir must be applied here too or an axis with dir = -1 runs as positive feedback
    servoPitch.write(constrain(linkPitch.servoTrim + linkPitch.dir * cPitch,
                               linkageServoMin(linkPitch), linkageServoMax(linkPitch)));
    servoYaw.write(constrain(linkYaw.servoTrim + linkYaw.dir * cYaw,
                             linkageServoMin(linkYaw), linkageServoMax(linkYaw)));
  }
}

static void buildTlm(LinkTlm &t) {
  t.ms     = millis();
  t.pitch  = gimbalPitch;
  t.yaw    = gimbalYaw;
  t.tilt   = acosf(constrain(V[2], -1.0f, 1.0f)) * 180.0f / PI;
  t.cmdP   = lastCmdP;
  t.cmdY   = lastCmdY;
  //via locals: LinkTlm is packed, so binding a float& directly to a field is a compile error
  float sp, sy;
  if (!linkageServoAngle(linkPitch, lastCmdP, sp)) sp = linkPitch.servoTrim;
  if (!linkageServoAngle(linkYaw,   lastCmdY, sy)) sy = linkYaw.servoTrim;
  t.svP = sp;
  t.svY = sy;
  t.accelG = sqrtf(A[0]*A[0] + A[1]*A[1] + A[2]*A[2]) / ACC_LSB_PER_G;
  t.offP   = linkPitch.servoOffset;
  t.offY   = linkYaw.servoOffset;
  t.state  = (uint8_t)flightState;
  t.flags  = (armed              ? LF_ARMED    : 0)
           | (benchActive        ? LF_BENCH    : 0)
           | (hasZeroed          ? LF_ZEROED   : 0)
           | (benchForceGyroOnly ? LF_GYROONLY : 0)
           | (tvcEnabled         ? LF_TVCON    : 0)
           | (armInhibit         ? LF_INHIBIT  : 0);
}

void benchRun();   //src/bench.cpp

void checkSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r' || c == 'f' || c == 'b') displayMode = c;

    else if (c == 'A' || c == 'a') {
      if (flightState != STATE_PAD) Serial.println("# arm refused: not on the pad.");
      else if (!hasZeroed)          Serial.println("# arm refused: never zeroed. Hold it still, or send K.");
      else if (armed)               Serial.println("# already armed.");
      else {
        armed = true;
        armInhibit = false;
        Serial.println("# EVENT ARMED manually");
      }
    }

    else if (c == 'D' || c == 'd') {
      armed = false;
      armInhibit = true;
      sfMode = false;
      sfArmedAtMs = 0;
      centreGimbal();
      Serial.println("# EVENT DISARMED manually. Auto-arm is now blocked until "
                     "you send A or reboot.");
    }

    else if (c == 'K' || c == 'k') {
      if (armed) Serial.println("# calibrate refused: disarm first (D).");
      else {
        zsReset();
        stillSinceMs = 0;
        lastRezeroMs = 0;
        forceZero = true;
        Serial.printf("# calibrating. HOLD STILL in flight attitude for %lu ms.\n",
                      (unsigned long)REZERO_STILL_MS);
      }
    }

    else if (c == 'O' || c == 'o') {
      if (armed) Serial.println("# trim refused: disarm first (D).");
      else {
        unsigned long keep = Serial.getTimeout();
        Serial.setTimeout(150);
        float op = Serial.parseFloat();
        float oy = Serial.parseFloat();
        Serial.setTimeout(keep);
        applyTrim(op, oy);
      }
    }

    //STATIC FIRE ARM; uppercase only, since lowercase 'b' is the display-mode selector
    else if (c == 'B') {
      if (flightState != STATE_PAD) Serial.println("# refused: not on the pad.");
      else if (!linkPitch.ready || !linkYaw.ready)
        Serial.println("# refused: linkage does not close. Fix the geometry first.");
      else {
        linkageRange(linkPitch, linkageServoMin(linkPitch), linkageServoMax(linkPitch),
                     sfPitchMin, sfPitchMax, 25.0f);
        linkageRange(linkYaw, linkageServoMin(linkYaw), linkageServoMax(linkYaw),
                     sfYawMin, sfYawMax, 25.0f);
        sfRadius = fminf(fminf(fabsf(sfPitchMin), sfPitchMax),
                         fminf(fabsf(sfYawMin),   sfYawMax));
        sfRadius = fminf(sfRadius, 15.0f);
        if (sfRadius < 3.0f) {
          Serial.printf("# refused: only %.1f deg reachable on every bearing.\n", sfRadius);
        } else {
          armed = true;
          sfMode = true;
          armInhibit = false;
          sfArmedAtMs = millis();
          Serial.printf("# EVENT ARMED FOR STATIC FIRE. Sweep radius %.1f deg "
                        "(flight cap is %.1f).\n", sfRadius, (float)MAX_GIMBAL_DEG);
          Serial.printf("# On boost: one continuous %.1f Hz circle at full radius, "
                        "from the first millisecond.\n", SF_CIRCLE_HZ);
          Serial.println("# THE GIMBAL WILL MOVE HARD. Keep hands clear.");
          Serial.printf("# Stays armed through knocks and wind. Only DISARM or "
                        "%lu minutes ends it.\n",
                        (unsigned long)(SF_ARM_TIMEOUT_MS / 60000));
        }
      }
    }

    else if (c == 't') {
      if (!armed && flightState == STATE_PAD) benchRun();
      else Serial.println("# bench refused: armed or in flight.");
    }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  { uint32_t t0 = millis(); while (!Serial && millis() - t0 < 2000) delay(10); }   //up to 2 s for host enumeration
  delay(150);
  delay(1500);

#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);   //discard rather than block the loop if no host attaches
#endif

  linkBegin();
  linkSetTlmSource(buildTlm);

  Serial.println("# === TVC Servo + IMU + PID ===");
  blinkLED(3, 200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoPitch.setPeriodHertz(50);
  servoYaw.setPeriodHertz(50);
  servoPitch.attach(SERVOPITCH_PIN, 500, 2400);
  servoYaw.attach(SERVOYAW_PIN, 500, 2400);

  setupLinkages();   //must run before the first write, since servoTrim is set here
  centreGimbal();
  delay(500);

  setupMPU9250();

  //brownout guard: not reading about 1 g means the boot did not happen on the pad
  {
    double sum = 0; int n = 0;
    for (int i = 0; i < 40; i++) {
      if (readRawIMU()) {
        sum += sqrtf((float)accRaw[0]*accRaw[0] + (float)accRaw[1]*accRaw[1]
                   + (float)accRaw[2]*accRaw[2]) / ACC_LSB_PER_G;
        n++;
      }
      delay(5);
    }
    float m = n ? (float)(sum / n) : 1.0f;
    bootedInFlt = (n > 0) && (fabsf(m - 1.0f) > 0.35f);
    if (bootedInFlt) {
      flightState = STATE_COAST;
      armed = false;
      coastAtMs = millis();
      linkSetQuiet(false);
      centreGimbal();
      Serial.printf("# BOOTED IN FLIGHT (%.2f g). Calibration skipped, "
                    "gimbal centred, control inhibited.\n", m);
    }
  }

  if (!bootedInFlt) {
    calibrateGyro();
    calibrateOrientation();
  }

  lastKalUs   = micros();
  lastPidUs = micros();

  Serial.println("# Send 'r', 'f' or 'b' to choose the stream, or 't' for the bench tests.");
  Serial.println("# Defaulting to 'b' in 2 s.");
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    if (Serial.available() > 0) { checkSerialInput(); break; }
    delay(10);
  }
  Serial.printf("# Display mode: %c\n", displayMode);
  logInit();
  logStart();
  Serial.printf("# Recorder armed: %d slots, %.1f s at 200 Hz, %.0f KB of RAM\n",
                LOG_SLOTS, LOG_SLOTS / 200.0, LOG_SLOTS * sizeof(LogRec) / 1024.0);
  Serial.printf("# Arming: needs a re-zero, then %s.\n",
                USE_IR_ARM ? "an IR signal" : "20 s of stillness");
  Serial.println("# Stand the rocket vertical and leave it alone to re-zero.");
  Serial.println("# READY");
}
void loop() {
  static unsigned long lastImu = 0, lastPrint = 0;
  checkSerialInput();

  unsigned long now = micros();

  if (now - lastImu >= IMU_PERIOD_US) {
    lastImu = now;
    updateAttitude();

    if (sfMode && flightState == STATE_BOOST) {
      staticFireSweep(millis() - boostStartMs);
    } else if (tvcEnabled && flightState == STATE_BOOST) {
      pidControl(0, 0, flightKp, flightKi, flightKd);
    }
  }

  if (sfMode && armed && flightState == STATE_PAD && sfArmedAtMs &&
      millis() - sfArmedAtMs > SF_ARM_TIMEOUT_MS) {
    armed = false;
    sfMode = false;
    sfArmedAtMs = 0;
    centreGimbal();
    Serial.println("# EVENT static fire arm expired. Disarmed. Press SF ARM again.");
  }

  //stall failsafe: centres the gimbal if updateAttitude() stops running (e.g. an I2C bus lockup)
  if (tvcEnabled && lastAttitudeMs && millis() - lastAttitudeMs > 250) {
    centreGimbal();
  }

  if (flightState == STATE_COAST && logRunning && coastAtMs) {
    uint32_t since = millis() - coastAtMs;

    if (!stage1Tried && logStage1Slot < 0 && since > LOG_FLUSH1_MS) {
      stage1Tried = true;
      logStage1Slot = logSaveNow();
      if (logStage1Slot >= 0) logStage1Uptime = logSlotUptime(logStage1Slot);
    }

    if (since > (sfLogFast ? 1500UL : LOG_FLUSH2_MS)) {
      logStop();
      int s2 = logSaveNow();
      if (s2 >= 0) {
        if (logStage1Slot >= 0 && s2 != logStage1Slot &&
            logSlotUptime(logStage1Slot) == logStage1Uptime) {
          logErase(logStage1Slot);
        }
        logSaved = true;
      }
      logStage1Slot = -1;
    }
  }

  if (now - lastPrint >= PRINT_PERIOD_US) {
    lastPrint = now;
    if (displayMode == 'r') {
      Serial.printf("%.2f,%.2f\n", rawPitch, rawYaw);
    } else if (displayMode == 'f') {
      Serial.printf("%.2f,%.2f\n", gimbalPitch, gimbalYaw);
    } else {
      //rawPitch, rawYaw, gimbalPitch, gimbalYaw, Vx, Vy, Vz, state, cmdP, cmdY
      int stateField = (int)flightState + (armed ? 10 : 0);
      Serial.printf("%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%d,%.2f,%.2f\n",
                    rawPitch, rawYaw, gimbalPitch, gimbalYaw,
                    V[0], V[1], V[2], stateField, lastCmdP, lastCmdY);
    }
  }

}
