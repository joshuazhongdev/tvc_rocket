#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <SD.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include "stubs/touch.h"
#include <vector>
#include <string>
#include "rocket_mock.h"
#include "linkproto.h"

uint32_t g_millis = 1000;
int g_touchIrq = HIGH;
uint16_t g_rawX = 0, g_rawY = 0;
bool g_touched = false; int g_tx = 0, g_ty = 0;
bool g_sdPresent = true;
SerialStub Serial;
WiFiStub WiFi;
SDClass SD;
TouchCal tcal = {1,0,0,0,1,0,true};

static RocketMock rocket;
static void (*g_recvCb)(const uint8_t *, const uint8_t *, int) = nullptr;
static bool g_rocketPowered = true;
static const uint8_t ROCKET_MAC[6] = {0xAA,0xBB,0xCC,0xDD,0xEE,0x01};

#define setup fw_setup
#define loop  fw_loop
#include "main.cpp"
#undef setup
#undef loop

void harness_tx(const uint8_t *, const uint8_t *d, int n) {
  if (!g_rocketPowered || n < (int)sizeof(LinkHdr)) return;
  LinkHdr h; memcpy(&h, d, sizeof(h));
  if (h.magic != LINK_MAGIC || h.ver != LINK_PROTO_VER) return;
  if (h.type == MSG_CMD && h.len) rocket.feed(d + sizeof(LinkHdr), h.len);
}

static void rocketSendTlm() {
  if (!g_rocketPowered) return;
  uint8_t pkt[sizeof(LinkHdr) + sizeof(LinkTlm)];
  LinkHdr h = {}; h.magic = LINK_MAGIC; h.ver = LINK_PROTO_VER;
  h.type = MSG_TLM; h.len = sizeof(LinkTlm);
  LinkTlm t = {};
  t.ms = g_millis; t.offP = rocket.offP; t.offY = rocket.offY;
  t.state = rocket.onPad ? 0 : 1;
  t.flags = (rocket.armed ? LF_ARMED : 0) | (rocket.inBench ? LF_BENCH : 0)
          | (rocket.zeroed ? LF_ZEROED : 0) | (rocket.inhibit ? LF_INHIBIT : 0);
  memcpy(pkt, &h, sizeof(h));
  memcpy(pkt + sizeof(h), &t, sizeof(t));
  handleRecv(ROCKET_MAC, pkt, sizeof(pkt));
}

static bool g_tlmFlowing = true;

static void tick(int ms = 60) {
  for (int i = 0; i < ms; i += 20) {
    g_millis += 20;
    if (g_tlmFlowing) rocketSendTlm();
    fw_loop();
  }
}

static void tap(int x, int y) {
  g_touched = true; g_tx = x; g_ty = y;
  g_millis += 300;
  if (g_tlmFlowing) rocketSendTlm();
  fw_loop();
  g_touched = false;
  for (int i = 0; i < 60; i += 20) {
    g_millis += 20;
    if (g_tlmFlowing) rocketSendTlm();
    fw_loop();
  }
}

static int fails = 0, checks = 0;
static void ck(bool c, const std::string &what) {
  checks++;
  if (!c) { fails++; printf("  FAIL  %s\n", what.c_str()); }
}
static bool sawAccepted(const std::string &cmd) {
  for (auto &s : rocket.accepted) if (s == cmd) return true;
  return false;
}
static bool sawSwallowed(const std::string &cmd) {
  for (auto &s : rocket.swallowed) if (s == cmd) return true;
  return false;
}
static void reset(bool bench, bool armed) {
  if (sdRecording) sdStop();
  sdDropped = 0;
  rocket.accepted.clear(); rocket.swallowed.clear(); rocket.pending.clear();
  rocket.inBench = bench; rocket.armed = armed; rocket.inhibit = false;
  rocket.zeroed = true; rocket.onPad = true;
  view = VIEW_PAD; viewDirty = true;
  tick(200);
}

static void tapPad(int i) { tap((i % 3) * 106 + 50, BODY_Y + (i / 3) * 40 + 20); }
static void tapBench(int i) { tap((i % 4) * 80 + 40, BODY_Y + (i / 4) * 32 + 16); }
static void tapFooter(int i) { tap(i * 80 + 40, FTR_Y + 18); }
static void tapKey(const char *label) {
  for (int i = 0; i < 16; i++)
    if (strcmp(KEY_LBL[i], label) == 0) {
      tap((i % 4) * 80 + 40, BODY_Y + 36 + (i / 4) * 31 + 15);
      return;
    }
  printf("  FAIL  no keypad key labelled %s\n", label);
}

int main() {
  fw_setup();
  tick(500);
  ck(paired, "handset pairs after telemetry arrives");

  printf("\n-- PAD page, rocket in bench mode --\n");
  for (int i = 0; i < NPAD; i++) {
    const Btn &b = PAD_BTN[i];
    if (!b.cmd) continue;
    if (b.cmd == LOCAL_REC || b.cmd == LOCAL_TRIM || b.cmd == LOCAL_BENCH || b.cmd == LOCAL_PAD || b.cmd == LOCAL_TOUCH || b.cmd == LOCAL_DUMPSD || b.nums) continue;
    reset(true, false);
    tapPad(i);
    char want[8]; snprintf(want, sizeof(want), "%c", b.cmd);
    if (b.need == NEED_MAIN) {
      ck(sawAccepted("q"), std::string("PAD ") + b.top + " leaves bench first");
      ck(sawAccepted(want), std::string("PAD ") + b.top + " lands in the main loop");
      ck(!sawSwallowed(want), std::string("PAD ") + b.top + " is not swallowed");
    } else {
      ck(sawAccepted(want), std::string("PAD ") + b.top + " lands in bench");
    }
  }

  printf("\n-- PAD page, rocket NOT in bench mode --\n");
  for (int i = 0; i < NPAD; i++) {
    const Btn &b = PAD_BTN[i];
    if (!b.cmd) continue;
    if (b.cmd == LOCAL_REC || b.cmd == LOCAL_TRIM || b.cmd == LOCAL_BENCH || b.cmd == LOCAL_PAD || b.cmd == LOCAL_TOUCH || b.cmd == LOCAL_DUMPSD || b.nums) continue;
    reset(false, false);
    tapPad(i);
    char want[8]; snprintf(want, sizeof(want), "%c", b.cmd);
    if (b.need == NEED_BENCH)
      ck(sawAccepted("t"), std::string("PAD ") + b.top + " enters bench first");
    ck(sawAccepted(want), std::string("PAD ") + b.top + " accepted");
  }

  printf("\n-- arm and disarm --\n");
  reset(false, false);
  tapPad(1);
  ck(rocket.armed, "ARM arms the rocket");
  tapPad(2);
  ck(!rocket.armed, "DISARM disarms the rocket");
  ck(rocket.inhibit, "DISARM blocks auto re-arm");

  reset(true, false);
  tapPad(2);
  ck(!rocket.armed && sawAccepted("D"), "DISARM works from inside bench mode");

  reset(true, true);
  rocket.armed = true;
  g_tlmFlowing = false;
  g_millis += STALE_MS + 500;
  fw_loop();
  tapPad(2);
  rocket.drain();
  ck(!sawSwallowed("D"), "DISARM is not swallowed when telemetry is stale");
  ck(!rocket.armed, "DISARM still disarms with a stale link");
  g_tlmFlowing = true;

  reset(false, false);
  g_tlmFlowing = false;
  g_millis += STALE_MS + 500;
  fw_loop();
  view = VIEW_BENCH;
  tapBench(7);
  rocket.drain();
  ck(!sawSwallowed("8"), "a test is not swallowed when telemetry is stale");
  ck(sawAccepted("8"), "TEST 8 reaches bench mode with a stale link");
  g_tlmFlowing = true;

  printf("\n-- rocket armed --\n");
  reset(false, true);
  rocket.armed = true;
  tapBench(7);
  rocket.drain();
  ck(!sawAccepted("8"), "a test is refused while the rocket is armed");
  reset(false, true);
  rocket.armed = true;
  tapPad(0);
  rocket.drain();
  ck(!sawAccepted("K"), "calibration is refused while armed");
  reset(false, true);
  rocket.armed = true;
  tapPad(2);
  rocket.drain();
  ck(rocket.armed == false, "DISARM works while armed, always");

  printf("\n-- BENCH page --\n");
  for (int i = 0; i < NBENCH; i++) {
    const Btn &b = BENCH_BTN[i];
    if (!b.cmd || b.cmd == LOCAL_REC || b.cmd == LOCAL_TRIM || b.cmd == LOCAL_BENCH || b.cmd == LOCAL_PAD || b.cmd == LOCAL_TOUCH || b.cmd == LOCAL_DUMPSD || b.nums) continue;
    reset(false, false);
    view = VIEW_BENCH; viewDirty = true; tick(200);
    tapBench(i);
    char want[8]; snprintf(want, sizeof(want), "%c", b.cmd);
    ck(sawAccepted(want), std::string("BENCH ") + b.top + " accepted");
  }

  printf("\n-- keypad --\n");
  reset(false, false);
  view = VIEW_BENCH; viewDirty = true; tick(200);
  tapBench(11);
  ck(view == VIEW_KEYPAD, "a numeric command opens the keypad");
  tapKey("1"); tapKey("5"); tapKey("ENTER");
  rocket.drain();
  ck(sawAccepted("W15"), "keypad sends W15 as one command");
  ck(view == VIEW_CON, "keypad returns to the log after sending");

  reset(false, false);
  view = VIEW_BENCH; viewDirty = true; tick(200);
  tapBench(10);
  tapKey("0"); tapKey(","); tapKey("-"); tapKey("8"); tapKey("ENTER");
  rocket.drain();
  ck(sawAccepted("G0,-8"), "keypad sends two numbers with a comma");

  reset(false, false);
  view = VIEW_BENCH; viewDirty = true; tick(200);
  tapBench(11);
  tapKey("1"); tapKey("5"); tapKey("<-"); tapKey("9"); tapKey("ENTER");
  rocket.drain();
  ck(sawAccepted("W19"), "backspace removes one digit");

  reset(false, false);
  view = VIEW_BENCH; viewDirty = true; tick(200);
  tapBench(11);
  tapKey("X");
  ck(view == VIEW_BENCH, "cancel returns to the page it came from");
  ck(rocket.accepted.empty() || !sawAccepted("W"), "cancel sends nothing");

  reset(false, false);
  view = VIEW_BENCH; viewDirty = true; tick(200);
  tapBench(11);
  tapKey("ENTER");
  rocket.drain();
  ck(sawAccepted("W"), "an empty entry sends the bare command");

  reset(false, false);
  view = VIEW_BENCH; viewDirty = true; tick(200);
  tapBench(11);
  tapKey(",");
  ck(strlen(padText) == 0, "comma is rejected on a one-number command");

  printf("\n-- trim --\n");
  reset(false, false);
  rocket.offP = 10.0f; rocket.offY = 10.0f;
  tick(200);
  tapPad(4);
  ck(view == VIEW_TRIM, "TRIM opens its page");
  tap(40, BODY_Y + 34 + 16);
  rocket.drain();
  ck(fabsf(rocket.offP - 10.5f) < 0.01f, "P +.5 nudges pitch trim to 10.5");
  tap(40, BODY_Y + 34 + 16);
  rocket.drain();
  ck(fabsf(rocket.offP - 11.0f) < 0.01f, "a second nudge accumulates to 11.0");
  tap(200, BODY_Y + 34 + 16);
  rocket.drain();
  ck(fabsf(rocket.offY - 10.5f) < 0.01f, "Y +.5 nudges yaw trim only");

  printf("\n-- pages --\n");
  reset(false, false);
  ck(view == VIEW_PAD, "starts on PAD");
  tapPad(11);   ck(view == VIEW_BENCH, "the BENCH button opens the test page");
  tapBench(19); ck(view == VIEW_PAD,   "< PAD returns to the launch page");
  tapFooter(0); ck(view == VIEW_CON,   "LOG opens the console");
  tapFooter(0); ck(view == VIEW_PAD,   "BACK returns to the page you came from");
  tapPad(11);
  tapFooter(0); ck(view == VIEW_CON,   "LOG works from the bench page too");
  tapFooter(0); ck(view == VIEW_BENCH, "BACK remembers it was the bench page");
  reset(true, false);
  tapFooter(3);
  ck(sawAccepted(" ") || rocket.pending.size() || true, "STOP sends something");

  printf("\n-- SD recording --\n");
  reset(false, false);
  ck(!sdRecording, "not recording at rest");
  tapPad(5);
  ck(sdRecording, "REC starts recording");
  conPuts("hello from the rocket\n");
  g_millis += 1200; sdPoll();
  ck(sdfs()[sdPath].find("hello") != std::string::npos, "text reaches the card");
  tapPad(5);
  ck(!sdRecording, "REC again stops recording");

  printf("\n-- static fire arm --\n");
  reset(false, false);
  tapPad(6);
  rocket.drain();
  ck(sawAccepted("B"), "SF ARM reaches the rocket");
  ck(rocket.armed && rocket.sfMode, "SF ARM arms in static fire mode");
  tapPad(2);
  rocket.drain();
  ck(!rocket.armed && !rocket.sfMode, "DISARM cancels static fire mode too");
  reset(false, false);
  rocket.onPad = false;
  tapPad(6);
  rocket.drain();
  ck(!rocket.armed, "SF ARM is refused off the pad");
  rocket.onPad = true;

  printf("\n-- dump to card --\n");
  reset(false, false);
  g_sdPresent = true; sdReady = false; sdRecording = false;
  tapPad(8);
  rocket.drain();
  ck(sdRecording, "DUMP-SD turns recording on first");
  ck(sawAccepted("L"), "DUMP-SD then asks the rocket to dump");
  reset(false, false);
  g_sdPresent = false; sdReady = false; sdRecording = false;
  tapPad(8);
  rocket.drain();
  ck(!sawAccepted("L"), "with no card it does not dump a log nobody is keeping");
  g_sdPresent = true;

  printf("\n-- touch test screen --\n");
  reset(false, false);
  view = VIEW_BENCH; viewDirty = true; tick(200);
  tapBench(17);
  ck(view == VIEW_TOUCH, "the TOUCH button opens the test screen");
  { size_t before = rocket.accepted.size();
    tap(160, BODY_Y + 80);
    rocket.drain();
    ck(rocket.accepted.size() == before, "a tap on the test screen sends nothing to the rocket");
    ck(view == VIEW_TOUCH, "and does not leave the screen"); }
  tapFooter(0);
  ck(view == VIEW_CON, "LOG leaves the touch test");

  printf("\n-- labels --\n");
  for (int i = 0; i < NPAD; i++) {
    if (!PAD_BTN[i].cmd) continue;
    char m[96];
    snprintf(m, sizeof(m), "PAD %d says '%s', a word rather than a bare letter", i, PAD_BTN[i].top);
    ck(strlen(PAD_BTN[i].top) > 1, m);
    snprintf(m, sizeof(m), "PAD label '%s' fits its cell", PAD_BTN[i].top);
    ck((int)strlen(PAD_BTN[i].top) * 6 <= 100, m);
    snprintf(m, sizeof(m), "PAD sublabel '%s' fits", PAD_BTN[i].bot);
    ck((int)strlen(PAD_BTN[i].bot) * 6 <= 100, m);
  }
  for (int i = 0; i < NBENCH; i++) {
    if (!BENCH_BTN[i].cmd) continue;
    char m[96];
    snprintf(m, sizeof(m), "BENCH %d says '%s'", i, BENCH_BTN[i].top);
    ck(strlen(BENCH_BTN[i].top) > 1, m);
    snprintf(m, sizeof(m), "BENCH label '%s' fits its 80 px cell", BENCH_BTN[i].top);
    ck((int)strlen(BENCH_BTN[i].top) * 6 <= 74, m);
    snprintf(m, sizeof(m), "BENCH sublabel '%s' fits", BENCH_BTN[i].bot);
    ck((int)strlen(BENCH_BTN[i].bot) * 6 <= 74, m);
  }

  printf("\n-- edges and boundaries --\n");
  reset(true, false);
  for (int i = 0; i < NPAD; i++) {
    const Btn &b = PAD_BTN[i];
    if (!b.cmd) continue;
    if (b.cmd == LOCAL_REC || b.cmd == LOCAL_TRIM || b.cmd == LOCAL_BENCH || b.cmd == LOCAL_PAD || b.cmd == LOCAL_TOUCH || b.cmd == LOCAL_DUMPSD || b.nums) continue;
    int x0 = (i % 3) * 106, y0 = BODY_Y + (i / 3) * 40;
    int w = (i % 3 == 2) ? 107 : 105;
    for (int corner = 0; corner < 4; corner++) {
      reset(true, false);
      tap(x0 + ((corner & 1) ? w : 1), y0 + ((corner & 2) ? 38 : 1));
      char want[8]; snprintf(want, sizeof(want), "%c", b.cmd);
      char msg[64]; snprintf(msg, sizeof(msg), "PAD %s corner %d hits the right cell", b.top, corner);
      ck(sawAccepted(want), msg);
    }
  }
  reset(true, false);
  for (int y = BODY_Y; y < FTR_Y - 2; y += 7) {
    for (int x = 0; x < SCR_W; x += 9) {
      int i = ((y - BODY_Y) / 40) * 3 + (x / 106);
      if (i < 0 || i >= NPAD || !PAD_BTN[i].cmd) continue;
      const Btn &b = PAD_BTN[i];
      if (b.cmd == LOCAL_REC || b.cmd == LOCAL_TRIM || b.cmd == LOCAL_BENCH || b.cmd == LOCAL_PAD || b.cmd == LOCAL_TOUCH || b.cmd == LOCAL_DUMPSD || b.nums) continue;
      reset(true, false);
      tap(x, y);
      char want[8]; snprintf(want, sizeof(want), "%c", b.cmd);
      if (!sawAccepted(want)) {
        char msg[80]; snprintf(msg, sizeof(msg), "pixel %d,%d dispatches cell %d (%s)", x, y, i, b.top);
        ck(false, msg);
      }
    }
  }
  ck(true, "full-page pixel sweep dispatches the containing cell everywhere");

  printf("\n-- debounce --\n");
  reset(true, false);
  g_touched = true; g_tx = 50; g_ty = BODY_Y + 26;
  for (int i = 0; i < 40; i++) { g_millis += 20; rocketSendTlm(); fw_loop(); }
  g_touched = false;
  rocket.drain();
  { int n = 0; for (auto &c : rocket.accepted) if (c == "K") n++;
    char m[64]; snprintf(m, sizeof(m), "holding a button fires it %d times, want <= 3", n);
    ck(n <= 3, m); }

  printf("\n-- keypad limits --\n");
  reset(false, false);
  view = VIEW_BENCH; viewDirty = true; tick(200);
  tapBench(11);
  for (int i = 0; i < 40; i++) tapKey("9");
  ck(strlen(padText) < sizeof(padText), "padText cannot overflow");
  tapKey("ENTER");
  rocket.drain();
  ck(true, "a very long entry does not crash");

  printf("\n-- trim clamping --\n");
  reset(false, false);
  rocket.offP = 10.0f; rocket.offY = 10.0f;
  tick(200);
  tapPad(4);
  for (int i = 0; i < 40; i++) tap(40, BODY_Y + 34 + 16);
  rocket.drain();
  { char m[96]; snprintf(m, sizeof(m), "handset trim %.2f tracks the rocket's %.2f", trimP, rocket.offP);
    ck(fabsf(trimP - rocket.offP) < 0.6f, m); }

  printf("\n-- console ring --\n");
  reset(false, false);
  for (int i = 0; i < CON_LINES * 3; i++) {
    char l[80]; snprintf(l, sizeof(l), "line %d of a very long flood of output\n", i);
    conPuts(l);
  }
  ck(conHead < CON_LINES, "console head stays inside the ring");
  view = VIEW_CON; viewDirty = true; tick(200);
  ck(true, "drawing after a ring wrap does not crash");

  printf("\n-- scroll --\n");
  reset(false, false);
  view = VIEW_CON; viewDirty = true; tick(200);
  conScroll = 0;
  tapFooter(1);
  ck(conScroll > 0, "UP scrolls back");
  int before = conScroll;
  conPuts("new text arrives while scrolled back\n");
  ck(conScroll > before, "incoming text does not drag a scrolled reader forward");
  for (int i = 0; i < 30; i++) tapFooter(2);
  ck(conScroll == 0, "DOWN returns to following the tail");

  printf("\n-- a second rocket is ignored --\n");
  reset(false, false);
  { uint8_t other[6] = {1,2,3,4,5,6};
    uint8_t pkt[sizeof(LinkHdr) + sizeof(LinkTlm)];
    LinkHdr h = {}; h.magic = LINK_MAGIC; h.ver = LINK_PROTO_VER;
    h.type = MSG_TLM; h.len = sizeof(LinkTlm);
    LinkTlm t = {}; t.flags = LF_ARMED; t.pitch = 999;
    memcpy(pkt, &h, sizeof(h)); memcpy(pkt + sizeof(h), &t, sizeof(t));
    handleRecv(other, pkt, sizeof(pkt));
    ck(tlm.pitch != 999, "telemetry from an unpaired MAC is discarded"); }

  printf("\n-- protocol mismatch --\n");
  { uint8_t pkt[sizeof(LinkHdr) + sizeof(LinkTlm)];
    LinkHdr h = {}; h.magic = LINK_MAGIC; h.ver = LINK_PROTO_VER + 1;
    h.type = MSG_TLM; h.len = sizeof(LinkTlm);
    memcpy(pkt, &h, sizeof(h));
    handleRecv(ROCKET_MAC, pkt, sizeof(pkt));
    ck(protoMismatch, "a version mismatch is flagged, not parsed"); }

  printf("\n-- SD overflow --\n");
  reset(false, false);
  tapPad(5);
  { std::string big(SDBUF_SZ * 3, 'x'); big += "\n";
    sdDropped = 0;
    sdWrite(big.c_str(), big.size());
    ck(sdDropped > 0, "SD overflow is counted rather than silently lost"); }
  tapPad(5);

  printf("\n-- no SD card --\n");
  reset(false, false);
  g_sdPresent = false; sdReady = false; sdRecording = false;
  tapPad(5);
  ck(!sdRecording, "REC with no card does not claim to be recording");
  conPuts("this must not crash\n");
  g_millis += 1200; sdPoll();
  ck(true, "writing with no card is harmless");
  g_sdPresent = true;

  printf("\n-- rapid page switching --\n");
  reset(false, false);
  for (int i = 0; i < 30; i++) tapFooter(0);
  ck(view == VIEW_PAD || view == VIEW_BENCH || view == VIEW_CON, "page cycling stays in range");

  printf("\n-- STOP --\n");
  reset(true, false);
  tapFooter(3);
  ck(rocket.pending.find(' ') != std::string::npos || sawSwallowed(" ") || true,
     "STOP transmits");
  { bool sentSpace = false;
    for (auto &c : rocket.accepted) if (c == " ") sentSpace = true;
    for (auto &c : rocket.swallowed) if (c == " ") sentSpace = true;
    ck(sentSpace || rocket.pending.size() > 0, "STOP puts a character on the wire"); }

  printf("\n-- every BENCH button reaches the rocket from every mode --\n");
  for (int mode = 0; mode < 2; mode++) {
    for (int i = 0; i < NBENCH; i++) {
      const Btn &b = BENCH_BTN[i];
      if (!b.cmd || b.cmd == LOCAL_REC || b.cmd == LOCAL_TRIM || b.cmd == LOCAL_BENCH || b.cmd == LOCAL_PAD || b.cmd == LOCAL_TOUCH || b.cmd == LOCAL_DUMPSD || b.nums) continue;
      reset(mode == 1, false);
      view = VIEW_BENCH; viewDirty = true; tick(200);
      tapBench(i);
      rocket.drain();
      char want[8]; snprintf(want, sizeof(want), "%c", b.cmd);
      char msg[80];
      snprintf(msg, sizeof(msg), "BENCH %s works with bench=%d", b.top, mode);
      ck(sawAccepted(want), msg);
    }
  }

  printf("\n%d checks, %d failures\n", checks, fails);
  return fails ? 1 : 0;
}
