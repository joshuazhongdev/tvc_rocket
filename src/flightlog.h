// ---------------------------------------------------------------------------
//flightlog.h: onboard flight recorder
//
//no serial cable in flight, so anything not written here is lost; records
//full state at the IMU rate
//
//WHY IT BUFFERS IN RAM
//
//a flash page write blocks for tens of ms, which would stall the 200 Hz
//control loop during the burn; the whole flight fits in RAM (28 bytes per
//sample at 200 Hz, so 12.5 s is 68 KB against the S3's 512 KB), so it is
//buffered and written once, after the flight ends
//
//the buffer is circular, so it always holds the most recent LOG_SLOTS
//samples, including the seconds before ignition
//
//WHY EVICTION IS BY QUALITY, NOT BY AGE
//
//a pothole, or a re-arm with the battery still connected, can trigger a
//save that looks like a flight; every file's header carries its peak
//acceleration and boost duration, and when space runs out the least
//flight-like file goes, so a junk record can never evict a real one
// ---------------------------------------------------------------------------
#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include <string.h>

#define LOG_SLOTS      2500      //12.5 s at 200 Hz, 68 KB of RAM
#define LOG_PREFIX     "/flt"
//only ~22 of 24 slots fit: a 12.5 s file is 68.4 KB against ~1.5 MB of LittleFS; logSaveNow() also checks free space, so the limit binds on bytes, not this count
#define LOG_MAX_FILES  24
#define LOG_BOOTCNT    "/boot.cnt"
#define LOG_MAGIC      0x31435654UL   //"TVC1"

//what counts as a real flight rather than a bump
#define LOG_REAL_BOOST_MS  300
#define LOG_REAL_PEAK_MG   3000

struct __attribute__((packed)) LogHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t recSize;
  uint32_t bootId;         //increments every power-on, so files always order
  uint32_t savedUptimeMs;  //millis() at the moment of saving
  uint32_t epochAtSave;    //unix seconds, 0 if the clock was never set
  uint32_t sampleCount;
  uint16_t peakAccMg;      //peak |a| in milli-g
  uint16_t boostMs;        //how long STATE_BOOST lasted
  uint16_t maxTiltCdeg;    //peak tilt off vertical, centi-degrees
  uint16_t reserved;
};

//28 bytes; scaled integers rather than floats, half the size and finer resolution than the sensors have anyway
struct __attribute__((packed)) LogRec {
  uint32_t tMs;
  uint8_t  state;        //0 pad, 1 boost, 2 coast
  uint8_t  flags;        //bit0 accelTrusted, bit1 armed, bit2 tvc active
  int16_t  pitch;        //deg * 100
  int16_t  yaw;         //deg * 100
  int16_t  vx, vy, vz;   //arrow components * 10000
  int16_t  accMag;       //g * 1000
  int16_t  gx, gy, gz;   //deg/s * 10
  int16_t  cmdP, cmdY;   //commanded gimbal deg * 100
};

//state lives in flightlog.cpp, not here: a `static` at namespace scope in this header would give every including TU its own copy, and main.cpp and bench.cpp both include it, risking logTick() filling one buffer while logSaveNow() reads another
extern LogRec   logBuf[LOG_SLOTS];
extern uint32_t logHead, logCount;
extern bool     logRunning, logSaved;
extern uint32_t logBootId;
extern uint32_t logEpoch0;              //unix seconds at uptime zero, 0 = unset
extern uint16_t logPeakMg;
extern uint32_t logBoostFirstMs, logBoostLastMs;
extern float    logMinVz;
extern bool     logReady;               //false = no filesystem, saving is a no-op

inline int16_t logQ(float v, float scale) {
  float x = v * scale;
  if (x >  32767.0f) x =  32767.0f;
  if (x < -32768.0f) x = -32768.0f;
  return (int16_t)x;
}

inline void logPath(char *out, size_t n, int slot) {
  snprintf(out, n, LOG_PREFIX "%03d.bin", slot);
}

//bumps the power-on counter so files can always be ordered even with no clock
//a blank partition fails its first mount with a scary esp_littlefs error; mounting plain first lets that be told apart from a real failure
inline void logInit() {
  logReady = LittleFS.begin(false);
  if (!logReady) {
    Serial.println("# LOG: no filesystem found. The esp_littlefs errors above are");
    Serial.println("#      what a blank partition looks like. Formatting once...");
    logReady = LittleFS.begin(true);
    if (!logReady) {
      Serial.println("# LOG: FORMAT FAILED. No flight logging this session.");
      Serial.println("#      Check that the partition table has a 'spiffs' partition.");
      return;
    }
    Serial.println("# LOG: formatted and mounted.");
    Serial.println("#      If this was NOT the first boot, every stored flight");
    Serial.println("#      has just been erased. A blank partition and a damaged");
    Serial.println("#      one look the same from here.");
  }
  uint32_t n = 0;
  File f = LittleFS.open(LOG_BOOTCNT, "r");
  if (f) { f.read((uint8_t *)&n, sizeof(n)); f.close(); }
  n++;
  f = LittleFS.open(LOG_BOOTCNT, "w");
  if (f) { f.write((const uint8_t *)&n, sizeof(n)); f.close(); }
  logBootId = n;
  Serial.printf("# LOG: boot #%lu\n", (unsigned long)n);
}

//sets a wall clock reference; pass unix seconds for right now
inline void logSetEpoch(uint32_t unixNow) {
  logEpoch0 = unixNow - (millis() / 1000UL);
  Serial.printf("# LOG: clock set, epoch at boot = %lu\n", (unsigned long)logEpoch0);
}

inline uint32_t logNowEpoch() {
  return logEpoch0 ? (logEpoch0 + millis() / 1000UL) : 0;
}

inline void logStart() {
  if (!logReady) return;
  logHead = logCount = 0;
  logRunning = true; logSaved = false;
  logPeakMg = 0; logBoostFirstMs = logBoostLastMs = 0; logMinVz = 1.0f;
}
inline void logStop() { logRunning = false; }

inline void logTick(uint8_t state, uint8_t flags, float pitch, float yaw,
                    const float *V, float accMagG, const float *G,
                    float cmdP, float cmdY) {
  if (!logRunning) return;
  LogRec &r = logBuf[logHead];
  r.tMs   = millis();
  r.state = state;
  r.flags = flags;
  r.pitch = logQ(pitch, 100);
  r.yaw  = logQ(yaw, 100);
  r.vx    = logQ(V[0], 10000);
  r.vy    = logQ(V[1], 10000);
  r.vz    = logQ(V[2], 10000);
  r.accMag= logQ(accMagG, 1000);
  r.gx    = logQ(G[0], 10);
  r.gy    = logQ(G[1], 10);
  r.gz    = logQ(G[2], 10);
  r.cmdP  = logQ(cmdP, 100);
  r.cmdY  = logQ(cmdY, 100);
  logHead = (logHead + 1) % LOG_SLOTS;
  if (logCount < LOG_SLOTS) logCount++;

  //running summary, used to tell a flight from a pothole
  uint16_t mg = (uint16_t)constrain(accMagG * 1000.0f, 0.0f, 65535.0f);
  //peak and tilt fold in only once the motor is lit; from power-on, carrying the rocket to the pad would report a false 90 deg max tilt, and a knock could set a peak toward eviction score that appears in no sample
  if (state != 0) {                       // 0 = STATE_PAD
    if (mg > logPeakMg) logPeakMg = mg;
    if (V[2] < logMinVz) logMinVz = V[2];
  }
  if (state == 1) {
    if (logBoostFirstMs == 0) logBoostFirstMs = r.tMs;
    logBoostLastMs = r.tMs;
  }
}

//3 = looks like a real flight, 0 = looks like a bump
inline int logScoreOf(const LogHeader &h) {
  int s = 0;
  if (h.boostMs   >= LOG_REAL_BOOST_MS) s += 2;
  if (h.peakAccMg >= LOG_REAL_PEAK_MG)  s += 1;
  return s;
}

inline bool logReadHeader(int slot, LogHeader &h) {
  char p[24]; logPath(p, sizeof(p), slot);
  //callers scan logSlotMask() first, so this only opens files that exist; an exists() guard here would just be a second failed open
  File f = LittleFS.open(p, "r");
  if (!f) return false;
  //magic alone isn't enough: also checks record layout and that the file is long enough for its claimed sample count, or a power-loss-torn write would read back as a complete, high-scoring flight that can never be evicted
  uint32_t sz = f.size();
  bool ok = f.read((uint8_t *)&h, sizeof(h)) == sizeof(h)
            && h.magic   == LOG_MAGIC
            && h.version == 1
            && h.recSize == sizeof(LogRec)
            && sz >= sizeof(LogHeader) + (uint32_t)h.sampleCount * h.recSize;
  f.close();
  return ok;
}

//which slots exist, from one directory listing
//LittleFS.exists() is implemented as open(), so guarding a read with it just moves the failed open; one directory listing avoids probing each slot
//returns a bitmask, bit i set means slot i exists
inline uint32_t logSlotMask() {
  uint32_t mask = 0;
  File root = LittleFS.open("/");
  if (!root) return 0;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    const char *n = f.name();
    if (n) {
      if (*n == '/') n++;                       //core version dependent
      //expect "fltNNN.bin"
      if (strncmp(n, "flt", 3) == 0 && strlen(n) >= 10) {
        int slot = atoi(n + 3);
        if (slot >= 0 && slot < LOG_MAX_FILES) mask |= (1UL << slot);
      }
    }
    f.close();
  }
  root.close();
  return mask;
}

inline int logNextFree() {
  const uint32_t mask = logSlotMask();
  for (int i = 0; i < LOG_MAX_FILES; i++)
    if (!(mask & (1UL << i))) return i;
  return -1;
}

inline int logNewest() {
  LogHeader h; int best = -1; uint32_t bb = 0, bu = 0;
  const uint32_t mask = logSlotMask();
  for (int i = 0; i < LOG_MAX_FILES; i++) {
    if (!(mask & (1UL << i))) continue;
    if (!logReadHeader(i, h)) continue;
    if (best < 0 || h.bootId > bb || (h.bootId == bb && h.savedUptimeMs > bu)) {
      best = i; bb = h.bootId; bu = h.savedUptimeMs;
    }
  }
  return best;
}

//weakest stored file scoring no better than maxScore, oldest first on a tie; -1 if nothing is evictable, protecting a real flight from a pothole
inline int logWeakest(int maxScore) {
  LogHeader h; int pick = -1, ps = 99; uint32_t pb = 0, pu = 0;
  const uint32_t mask = logSlotMask();
  for (int i = 0; i < LOG_MAX_FILES; i++) {
    if (!(mask & (1UL << i))) continue;
    if (!logReadHeader(i, h)) continue;
    int sc = logScoreOf(h);
    if (sc > maxScore) continue;
    bool better = (pick < 0) || (sc < ps) ||
                  (sc == ps && (h.bootId < pb || (h.bootId == pb && h.savedUptimeMs < pu)));
    if (better) { pick = i; ps = sc; pb = h.bootId; pu = h.savedUptimeMs; }
  }
  return pick;
}

//raw write, returns the slot used or -1; ignores the logSaved latch, so the two-stage flush can call it twice
//force is for a person-requested save (M command): the eviction rules refuse to save anything scoring worse than what's stored, which is wrong for a deliberate save, since a clamped static fire never latches boost and would score as junk; force evicts the weakest file regardless of score
inline int logSaveNow(bool force = false) {
  if (logCount == 0) return -1;
  if (!LittleFS.begin(true)) { Serial.println("# LOG: mount failed"); return -1; }

  LogHeader h{};
  h.magic = LOG_MAGIC;
  h.version = 1;
  h.recSize = sizeof(LogRec);
  h.bootId = logBootId;
  h.savedUptimeMs = millis();
  h.epochAtSave = logNowEpoch();
  h.sampleCount = logCount;
  h.peakAccMg = logPeakMg;
  h.boostMs = (uint16_t)constrain((float)(logBoostLastMs - logBoostFirstMs), 0.0f, 65535.0f);
  float tilt = acosf(constrain(logMinVz, -1.0f, 1.0f)) * 180.0f / PI;
  h.maxTiltCdeg = (uint16_t)constrain(tilt * 100.0f, 0.0f, 65535.0f);

  const int myScore = logScoreOf(h);
  const uint32_t need = sizeof(LogHeader) + logCount * sizeof(LogRec) + 4096;

  //makes room, evicting the least flight-like file first
  int slot = logNextFree();
  while (slot < 0 || LittleFS.totalBytes() - LittleFS.usedBytes() < need) {
    int victim = logWeakest(force ? 99 : myScore);
    if (victim < 0) {
      if (force)
        Serial.println("# LOG: no room and no file to evict. Erase one with E.");
      else
        Serial.printf("# LOG: no room, and nothing worth evicting for a "
                      "score-%d record. NOT saved.\n", myScore);
      return -1;
    }
    char vp[24]; logPath(vp, sizeof(vp), victim);
    LogHeader vh; logReadHeader(victim, vh);
    Serial.printf("# LOG: evicting slot %d (score %d, boost %u ms, peak %.1f g)\n",
                  victim, logScoreOf(vh), vh.boostMs, vh.peakAccMg / 1000.0f);
    LittleFS.remove(vp);
    slot = logNextFree();
  }

  char p[24]; logPath(p, sizeof(p), slot);
  File f = LittleFS.open(p, "w");
  if (!f) { Serial.println("# LOG: open for write failed"); return -1; }
  //every write is checked: an unchecked short write would leave a file with a full header but partial records, reading back as a real flight that can never be evicted
  bool wrote = f.write((const uint8_t *)&h, sizeof(h)) == sizeof(h);
  uint32_t start = (logCount < LOG_SLOTS) ? 0 : logHead;
  for (uint32_t i = 0; i < logCount && wrote; i++) {
    const LogRec &r = logBuf[(start + i) % LOG_SLOTS];
    wrote = f.write((const uint8_t *)&r, sizeof(LogRec)) == sizeof(LogRec);
  }
  f.close();
  if (!wrote) {
    //removed rather than left: a surviving stub would occupy a slot and misrepresent its contents
    LittleFS.remove(p);
    Serial.println("# LOG: write failed partway, partial file removed.");
    return -1;
  }
  Serial.printf("# LOG: saved slot %d, %lu samples, score %d "
                "(boost %u ms, peak %.1f g, max tilt %.1f deg)\n",
                slot, (unsigned long)logCount, myScore,
                h.boostMs, h.peakAccMg / 1000.0f, h.maxTiltCdeg / 100.0f);
  return slot;
}

inline bool logSave() {
  if (logSaved) return false;
  int s = logSaveNow();
  if (s >= 0) logSaved = true;
  return s >= 0;
}

//uptime stamp of a slot; lets a two-stage flush confirm a file is still its own before deleting it
inline uint32_t logSlotUptime(int slot) {
  LogHeader h;
  return logReadHeader(slot, h) ? h.savedUptimeMs : 0;
}

inline void logStamp(char *out, size_t n, const LogHeader &h) {
  if (h.epochAtSave) snprintf(out, n, "epoch %lu", (unsigned long)h.epochAtSave);
  else snprintf(out, n, "boot #%lu +%.1fs", (unsigned long)h.bootId,
                h.savedUptimeMs / 1000.0f);
}

inline void logList() {
  if (!LittleFS.begin(true)) { Serial.println("# LOG: mount failed"); return; }
  LogHeader h; int n = 0; char stamp[40];
  Serial.println("# slot  when                 samples   secs  boost   peak    tilt   score");
  const uint32_t mask = logSlotMask();
  for (int i = 0; i < LOG_MAX_FILES; i++) {
    if (!(mask & (1UL << i))) continue;
    if (!logReadHeader(i, h)) continue;
    logStamp(stamp, sizeof(stamp), h);
    Serial.printf("#  %3d  %-19s %7lu  %5.2f  %4u ms  %4.1f g  %5.1f    %d%s\n",
                  i, stamp, (unsigned long)h.sampleCount, h.sampleCount / 200.0f,
                  h.boostMs, h.peakAccMg / 1000.0f, h.maxTiltCdeg / 100.0f,
                  logScoreOf(h), logScoreOf(h) >= 3 ? "  <- real flight" : "");
    n++;
  }
  if (!n) Serial.println("# LOG: flash is empty");
  Serial.printf("# LOG: %d file(s), %lu of %lu bytes used\n", n,
                (unsigned long)LittleFS.usedBytes(), (unsigned long)LittleFS.totalBytes());
}

//CSV to serial; negative slot means the newest
inline void logDump(int slot = -1) {
  if (!LittleFS.begin(true)) { Serial.println("# LOG: mount failed"); return; }
  if (slot < 0) slot = logNewest();
  if (slot < 0) { Serial.println("# LOG: no logs on flash"); return; }
  LogHeader h;
  if (!logReadHeader(slot, h)) { Serial.printf("# LOG: slot %d unreadable\n", slot); return; }
  char p[24]; logPath(p, sizeof(p), slot);
  File f = LittleFS.open(p, "r");
  if (!f) return;
  f.read((uint8_t *)&h, sizeof(h));            //skip the header

  char stamp[40]; logStamp(stamp, sizeof(stamp), h);
  const float durS = h.sampleCount / 200.0f;

  //everything needed to interpret the numbers goes at the top, as comment lines a CSV reader skips
  Serial.println("# ---------------------------------------------------------------");
  Serial.printf ("# TVC FLIGHT LOG   slot %d   %s\n", slot, stamp);
  Serial.printf ("# %lu samples at 200 Hz = %.2f seconds\n",
                 (unsigned long)h.sampleCount, durS);
  Serial.printf ("# boost lasted %u ms, peak |a| %.2f g, max tilt %.1f deg\n",
                 h.boostMs, h.peakAccMg / 1000.0f, h.maxTiltCdeg / 100.0f);
  Serial.printf ("# boot %lu, saved at uptime %lu ms\n",
                 (unsigned long)h.bootId, (unsigned long)h.savedUptimeMs);
  Serial.println("#");
  Serial.println("# The buffer is circular and holds the most recent 12.5 s, so t_ms");
  Serial.println("# is milliseconds since POWER ON, not since ignition. Subtract the");
  Serial.println("# first t_ms of STATE 1 to get time from ignition.");
  Serial.println("#");
  Serial.println("# state          0 pad, 1 boost, 2 coast");
  Serial.println("# accelTrusted   1 = accelerometer feeding the filter. 0 through");
  Serial.println("#                boost, when it measures thrust and not gravity.");
  Serial.println("# armed          1 = liftoff detection live");
  Serial.println("# tvc            1 = closed loop was driving the gimbal");
  Serial.println("# gimPitch/Yaw   attitude in GIMBAL axes, degrees. Already rotated");
  Serial.println("#                out of the IMU frame, so each one is the axis a");
  Serial.println("#                single servo controls.");
  Serial.println("# vx,vy,vz       gravity unit vector, body frame, the pole-free");
  Serial.println("#                estimator. vz near 1 is upright.");
  Serial.println("# accG           accelerometer magnitude, g. Clamped on a stand this");
  Serial.println("#                reads 1.00 however hard the motor pushes.");
  Serial.println("# gx,gy,gz       gyro rates, deg/s, body frame");
  Serial.println("# cmdP,cmdY      gimbal deflection COMMANDED, degrees of thrust");
  Serial.println("#                vector angle, not servo degrees.");
  Serial.println("# ---------------------------------------------------------------");
  //machine readable: readlog.py matches "# BEGIN" and parses the key=value tokens before capturing rows; changing this shape breaks downloads
  Serial.printf("# BEGIN slot=%d when=%s samples=%lu durS=%.2f boostMs=%u "
                "peakG=%.3f maxTilt=%.2f\n",
                slot, stamp, (unsigned long)h.sampleCount, durS, h.boostMs,
                h.peakAccMg / 1000.0f, h.maxTiltCdeg / 100.0f);
  Serial.println("t_ms,state,accelTrusted,armed,tvc,gimPitch_deg,gimYaw_deg,"
                 "vx,vy,vz,accG,gx_dps,gy_dps,gz_dps,cmdP_deg,cmdY_deg");
  LogRec r;
  while (f.read((uint8_t *)&r, sizeof(LogRec)) == sizeof(LogRec)) {
    Serial.printf("%lu,%u,%u,%u,%u,%.2f,%.2f,%.4f,%.4f,%.4f,%.3f,%.1f,%.1f,%.1f,%.2f,%.2f\n",
                  (unsigned long)r.tMs, r.state,
                  (r.flags >> 0) & 1, (r.flags >> 1) & 1, (r.flags >> 2) & 1,
                  r.pitch / 100.0, r.yaw / 100.0,
                  r.vx / 10000.0, r.vy / 10000.0, r.vz / 10000.0,
                  r.accMag / 1000.0, r.gx / 10.0, r.gy / 10.0, r.gz / 10.0,
                  r.cmdP / 100.0, r.cmdY / 100.0);
  }
  f.close();
  Serial.printf("# END slot %d, %.2f s\n", slot, durS);
}

inline void logErase(int slot = -1) {
  if (!LittleFS.begin(true)) return;
  char p[24];
  if (slot >= 0) {
    logPath(p, sizeof(p), slot);
    LittleFS.remove(p);
    Serial.printf("# LOG: erased slot %d\n", slot);
    return;
  }
  const uint32_t mask = logSlotMask();
  for (int i = 0; i < LOG_MAX_FILES; i++) {
    if (!(mask & (1UL << i))) continue;
    logPath(p, sizeof(p), i);
    LittleFS.remove(p);
  }
  Serial.println("# LOG: all slots erased");
}
