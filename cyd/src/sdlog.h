#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#define SD_SCK   18
#define SD_MISO  19
#define SD_MOSI  23
#define SD_CS     5

#define SDBUF_SZ    2048
#define SD_FLUSH_MS 1000

static SPIClass sdSPI(VSPI);
static bool     sdReady   = false;
static bool     sdRecording = false;
static char     sdPath[24] = "";
static char     sdBuf[SDBUF_SZ];
static uint16_t sdLen = 0;
static uint32_t sdLastFlush = 0;
static uint32_t sdBytes = 0;
static uint32_t sdDropped = 0;
static bool     sdFault = false;
static portMUX_TYPE sdMux = portMUX_INITIALIZER_UNLOCKED;

static bool sdBegin() {
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdReady = SD.begin(SD_CS, sdSPI, 20000000);
  return sdReady;
}

static void sdFlush() {
  if (!sdReady || !sdPath[0]) { sdLen = 0; return; }
  static char out[SDBUF_SZ];
  uint16_t n;
  portENTER_CRITICAL(&sdMux);
  n = sdLen;
  if (n) memcpy(out, sdBuf, n);
  sdLen = 0;
  portEXIT_CRITICAL(&sdMux);
  sdLastFlush = millis();
  if (!n) return;

  File f = SD.open(sdPath, FILE_APPEND);
  if (!f) { sdFault = true; return; }
  size_t w = f.write((const uint8_t *)out, n);
  f.close();
  if (w != n) sdFault = true;
  sdBytes += w;
}

static bool sdStart() {
  if (!sdReady && !sdBegin()) return false;
  for (int i = 1; i < 1000; i++) {
    snprintf(sdPath, sizeof(sdPath), "/tvc%03d.txt", i);
    if (!SD.exists(sdPath)) {
      File f = SD.open(sdPath, FILE_WRITE);
      if (!f) { sdPath[0] = 0; return false; }
      f.close();
      sdLen = 0; sdBytes = 0; sdDropped = 0; sdFault = false;
      sdRecording = true;
      sdLastFlush = millis();
      return true;
    }
  }
  sdPath[0] = 0;
  return false;
}

static void sdStop() {
  sdFlush();
  sdRecording = false;
}

static void sdWrite(const char *s, size_t n) {
  if (!sdRecording) return;
  portENTER_CRITICAL(&sdMux);
  for (size_t i = 0; i < n; i++) {
    if (sdLen >= SDBUF_SZ) { sdDropped++; break; }
    sdBuf[sdLen++] = s[i];
  }
  portEXIT_CRITICAL(&sdMux);
}

static void sdPoll() {
  if (!sdRecording) return;
  bool due = (sdLen >= SDBUF_SZ / 2) || (sdLen && millis() - sdLastFlush >= SD_FLUSH_MS);
  if (due) sdFlush();
}
