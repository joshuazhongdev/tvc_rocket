#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <string.h>
#include "display.h"
#include "touch.h"
#include "sdlog.h"
#include "linkproto.h"

static LGFX lcd;

static LGFX_Sprite hdrSpr(&lcd);
static bool hdrSprOk = false;

#define SCR_W   320
#define SCR_H   240
#define HDR_H    40
#define FTR_Y   204
#define FTR_H    36
#define BODY_Y   42
#define BODY_H  (FTR_Y - BODY_Y - 2)

#define COL_BG      0x0000
#define COL_HDR     0x18E3
#define COL_TEXT    0xFFFF
#define COL_DIM     0x8410
#define COL_OK      0x07E0
#define COL_WARN    0xFFE0
#define COL_BAD     0xF800
#define COL_BTN     0x2124
#define COL_BTN_HI  0x4A69
#define COL_EDGE    0x630C
#define COL_STOP    0x9000
#define COL_ARM     0x7800
#define COL_SAFE    0x0320

static const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static uint8_t  rocketMac[6]  = {0,0,0,0,0,0};
static bool     paired        = false;
static uint32_t lastTlmMs     = 0;
#define STALE_MS 3500
static LinkTlm  tlm = {};
static portMUX_TYPE tlmMux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t seqCmd = 0;
static bool     protoMismatch = false;

#define CON_COLS  53
#define CON_LINES 140
static char     conBuf[CON_LINES][CON_COLS + 1];
static uint16_t conHead = 0, conCol = 0;
static int16_t  conScroll = 0;
static volatile bool conDirty = true;
static volatile bool viewDirty = true;
static portMUX_TYPE conMux = portMUX_INITIALIZER_UNLOCKED;

static void conNewline() {
  conBuf[conHead][conCol] = 0;
  conHead = (conHead + 1) % CON_LINES;
  conCol  = 0;
  conBuf[conHead][0] = 0;
  if (conScroll > 0 && conScroll < CON_LINES - 1) conScroll++;
}

static void conPutc(char c) {
  if (c == '\r') return;
  if (c == '\n') { conNewline(); return; }
  if (c < 32 || c > 126) c = '.';
  if (conCol >= CON_COLS) conNewline();
  conBuf[conHead][conCol++] = c;
  conBuf[conHead][conCol]   = 0;
}

static void conPuts(const char *s) {
  size_t n = strlen(s);
  portENTER_CRITICAL(&conMux);
  for (size_t i = 0; i < n; i++) conPutc(s[i]);
  portEXIT_CRITICAL(&conMux);
  sdWrite(s, n);
  conDirty = true;
}

enum View { VIEW_PAD, VIEW_BENCH, VIEW_CON, VIEW_KEYPAD, VIEW_TRIM, VIEW_TOUCH };
static View view = VIEW_PAD;
static View pageReturn = VIEW_PAD;

#define NEED_ANY   0
#define NEED_BENCH 1
#define NEED_MAIN  2

#define LOCAL_REC  '\x01'
#define LOCAL_TRIM '\x02'
#define LOCAL_BENCH '\x03'
#define LOCAL_PAD   '\x04'
#define LOCAL_TOUCH '\x05'
#define LOCAL_DUMPSD '\x06'

struct Btn { const char *top; const char *bot; char cmd; uint8_t nums; uint8_t need; };

#define NPAD 12
static const Btn PAD_BTN[NPAD] = {
  {"CALIBRATE","zero now",  'K', 0, NEED_MAIN},
  {"ARM",      "flight",    'A', 0, NEED_MAIN},
  {"DISARM",   "",          'D', 0, NEED_MAIN},
  {"CENTRE",   "gimbal 0",  'G', 0, NEED_BENCH},
  {"TRIM",     "remembers", LOCAL_TRIM, 0, NEED_ANY},
  {"RECORD",   "SD on/off", LOCAL_REC,  0, NEED_ANY},
  {"SF ARM",   "sweep test",'B', 0, NEED_MAIN},
  {"SAVE",     "buffer now",'M', 0, NEED_BENCH},
  {"DUMP-SD",  "log to card",LOCAL_DUMPSD, 0, NEED_ANY},
  {"LOGS",     "list",      'F', 0, NEED_BENCH},
  {"",         "",           0,  0, NEED_ANY},
  {"BENCH",    "tests >",   LOCAL_BENCH, 0, NEED_ANY},
};

#define NBENCH 20
static const Btn BENCH_BTN[NBENCH] = {
  {"1 AXIS",   "map",     '1',0,NEED_BENCH}, {"2 SERVO", "travel", '2',0,NEED_BENCH},
  {"3 IMU",    "health",  '3',0,NEED_BENCH}, {"4 FILTER","noise",  '4',0,NEED_BENCH},
  {"5 NOISE",  "+ servo", '5',0,NEED_BENCH}, {"6 GYRO",  "drift",  '6',0,NEED_BENCH},
  {"7 GYRO",   "return",  '7',0,NEED_BENCH}, {"8 CLOSED","loop",   '8',0,NEED_BENCH},
  {"LIVE",     "csv",     'C',0,NEED_BENCH}, {"GYRO",    "only",   'g',0,NEED_BENCH},
  {"GIMBAL",   "degrees", 'G',2,NEED_BENCH}, {"ENVELOPE","walk",   'W',1,NEED_BENCH},
  {"RAW",      "servo",   'S',2,NEED_BENCH}, {"ERASE",   "logs",   'E',1,NEED_BENCH},
  {"GAIN",     "kP",      'p',1,NEED_BENCH}, {"GAIN",    "kI",     'i',1,NEED_BENCH},
  {"GAIN",     "kD",      'd',1,NEED_BENCH}, {"TOUCH",   "test",   LOCAL_TOUCH,0,NEED_ANY},
  {"SAVE",     "log now", 'M',0,NEED_BENCH}, {"< PAD",   "back",   LOCAL_PAD,0,NEED_ANY},
};

static char    padCmd  = 0;
static uint8_t padNums = 1;
static uint8_t padNeed = NEED_BENCH;
static char    padText[24] = "";

#define TRIM_LIMIT 25.0f
static float   trimP = 0, trimY = 0;

static bool addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = LINK_CHANNEL;
  p.encrypt = false;
  p.ifidx   = WIFI_IF_STA;
  return esp_now_add_peer(&p) == ESP_OK;
}

static void handleRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len < (int)sizeof(LinkHdr)) return;
  LinkHdr h;
  memcpy(&h, data, sizeof(h));
  if (h.magic != LINK_MAGIC) return;
  if (h.ver != LINK_PROTO_VER) { protoMismatch = true; return; }
  if (h.len > len - (int)sizeof(LinkHdr)) return;

  if (!paired) {
    memcpy(rocketMac, mac, 6);
    if (addPeer(rocketMac)) { paired = true; viewDirty = true; }
  } else if (memcmp(rocketMac, mac, 6) != 0) {
    return;
  }

  const uint8_t *pay = data + sizeof(LinkHdr);
  if (h.type == MSG_TEXT && h.len) {
    portENTER_CRITICAL(&conMux);
    for (uint8_t i = 0; i < h.len; i++) conPutc((char)pay[i]);
    portEXIT_CRITICAL(&conMux);
    sdWrite((const char *)pay, h.len);
    conDirty = true;
  } else if (h.type == MSG_TLM && h.len >= (int)sizeof(LinkTlm)) {
    portENTER_CRITICAL(&tlmMux);
    memcpy(&tlm, pay, sizeof(LinkTlm));
    portEXIT_CRITICAL(&tlmMux);
    lastTlmMs = millis();
  }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *d, int n) {
  handleRecv(info->src_addr, d, n);
}
#else
static void onRecv(const uint8_t *mac, const uint8_t *d, int n) {
  handleRecv(mac, d, n);
}
#endif

static void sendRaw(uint8_t type, const uint8_t *pay, uint8_t n) {
  uint8_t pkt[sizeof(LinkHdr) + LINK_MAX_PAYLOAD];
  LinkHdr h = {};
  h.magic = LINK_MAGIC;
  h.ver   = LINK_PROTO_VER;
  h.type  = type;
  h.len   = n;
  h.seq   = seqCmd++;
  memcpy(pkt, &h, sizeof(h));
  if (n) memcpy(pkt + sizeof(h), pay, n);
  esp_now_send(paired ? rocketMac : BCAST, pkt, sizeof(h) + n);
}

static void sendCmd(const char *s, bool echo = true) {
  size_t n = strlen(s);
  if (n > LINK_MAX_PAYLOAD) n = LINK_MAX_PAYLOAD;
  sendRaw(MSG_CMD, (const uint8_t *)s, (uint8_t)n);
  if (!echo) return;
  char e[48];
  snprintf(e, sizeof(e), ">> %s", s);
  conPuts(e);
}

static bool linkLive() { return paired && (millis() - lastTlmMs < STALE_MS); }
static bool inBench()  { return linkLive() && (tlm.flags & LF_BENCH); }

static void sendFor(char cmd, uint8_t need, const char *arg) {
  if (need == NEED_BENCH) { sendCmd("t\n", false); delay(80); }
  if (need == NEED_MAIN)  { sendCmd("q\n", false); delay(80); }
  char out[40];
  snprintf(out, sizeof(out), "%c%s\n", cmd, arg ? arg : "");
  sendCmd(out);
}

static void drawButton(int x, int y, int w, int h, const char *top,
                       const char *bot, uint16_t fill, uint16_t fg, int topSize) {
  lcd.fillRoundRect(x + 1, y + 1, w - 2, h - 2, 3, fill);
  lcd.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 3, COL_EDGE);
  while (topSize > 1 && (int)strlen(top) * 6 * topSize > w - 6) topSize--;
  lcd.setTextColor(fg, fill);
  if (bot && *bot) {
    lcd.setTextDatum(textdatum_t::top_center);
    lcd.setTextSize(topSize);
    lcd.drawString(top, x + w / 2, y + (h - 8 - 8 * topSize) / 2);
    lcd.setTextSize(1);
    lcd.setTextColor(COL_DIM, fill);
    lcd.drawString(bot, x + w / 2, y + h - 11);
  } else {
    lcd.setTextDatum(textdatum_t::middle_center);
    lcd.setTextSize(topSize);
    lcd.drawString(top, x + w / 2, y + h / 2);
  }
  lcd.setTextDatum(textdatum_t::top_left);
  lcd.setTextSize(1);
}

static void drawHeader() {
  LovyanGFX *g = hdrSprOk ? (LovyanGFX *)&hdrSpr : (LovyanGFX *)&lcd;
  g->fillRect(0, 0, SCR_W, HDR_H, COL_HDR);
  g->setTextSize(1);
  g->setTextDatum(textdatum_t::top_left);

  bool live = linkLive();
  LinkTlm t;
  portENTER_CRITICAL(&tlmMux);
  t = tlm;
  portEXIT_CRITICAL(&tlmMux);

  bool burning = paired && !live && t.state == 1 && (millis() - lastTlmMs < 12000);
  const char *linkTxt = protoMismatch ? "PROTO MISMATCH"
                      : !paired       ? "searching"
                      : live          ? "linked"
                      : burning       ? "burn, quiet" : "NO SIGNAL";
  g->setTextColor(protoMismatch ? COL_BAD : !paired ? COL_WARN
                   : live ? COL_OK : burning ? COL_WARN : COL_BAD, COL_HDR);
  g->drawString(linkTxt, 4, 3);

  g->setTextColor(sdFault ? COL_BAD : sdRecording ? COL_BAD : COL_DIM, COL_HDR);
  if (sdDropped) {
    char d[20];
    snprintf(d, sizeof(d), "SD-%lu", (unsigned long)sdDropped);
    g->drawString(d, 262, 3);
  } else {
    g->drawString(sdFault ? "SD FAULT" : sdRecording ? "REC" : "not rec", 268, 3);
  }

  if (live) {
    const char *st = t.state == 0 ? "PAD" : t.state == 1 ? "BOOST" : "COAST";
    g->setTextColor(t.state ? COL_BAD : COL_DIM, COL_HDR);
    g->drawString(st, 92, 3);
    g->setTextColor((t.flags & LF_ARMED) ? COL_BAD : COL_OK, COL_HDR);
    g->drawString((t.flags & LF_ARMED) ? "ARMED" : "safe", 126, 3);
    g->setTextColor(COL_DIM, COL_HDR);
    if (t.flags & LF_INHIBIT) g->drawString("inhib", 170, 3);
    else if (t.flags & LF_BENCH) g->drawString("bench", 170, 3);
    else if (!(t.flags & LF_ZEROED)) { g->setTextColor(COL_WARN, COL_HDR); g->drawString("NOZERO", 170, 3); }

    char l[72];
    g->setTextColor(COL_TEXT, COL_HDR);
    snprintf(l, sizeof(l), "P %+6.1f  Y %+6.1f  tilt %5.1f  %4.2fg",
             t.pitch, t.yaw, t.tilt, t.accelG);
    g->drawString(l, 4, 15);
    g->setTextColor(COL_DIM, COL_HDR);
    snprintf(l, sizeof(l), "cmd %+5.1f/%+5.1f  sv %5.1f/%5.1f  trim %+.1f/%+.1f",
             t.cmdP, t.cmdY, t.svP, t.svY, t.offP, t.offY);
    g->drawString(l, 4, 27);
  } else {
    g->setTextColor(burning ? COL_WARN : COL_DIM, COL_HDR);
    g->drawString(burning ? "BOOST: radio quiet for the burn, back shortly"
                : paired  ? "rocket silent. Powered down, or out of range?"
                          : "broadcasting. Power the rocket on.", 4, 19);
  }
  if (hdrSprOk) hdrSpr.pushSprite(0, 0);
}

static void drawFooter() {
  const char *pg = (view == VIEW_CON) ? "BACK" : "LOG";
  lcd.fillRect(0, FTR_Y, SCR_W, FTR_H, COL_BG);
  drawButton(  0, FTR_Y, 80, FTR_H, pg,     "", COL_BTN,  COL_TEXT, 1);
  drawButton( 80, FTR_Y, 80, FTR_H, "UP",   "", COL_BTN,  COL_TEXT, 1);
  drawButton(160, FTR_Y, 80, FTR_H, "DOWN", "", COL_BTN,  COL_TEXT, 1);
  drawButton(240, FTR_Y, 80, FTR_H, "STOP", "", COL_STOP, COL_TEXT, 1);
}

static void drawPad() {
  lcd.fillRect(0, BODY_Y, SCR_W, BODY_H, COL_BG);
  for (int i = 0; i < NPAD; i++) {
    if (!PAD_BTN[i].cmd) continue;
    int cx = (i % 3) * 106, cy = BODY_Y + (i / 3) * 40;
    int w  = (i % 3 == 2) ? 108 : 106;
    uint16_t fill = COL_BTN;
    if (PAD_BTN[i].cmd == 'A') fill = COL_ARM;
    if (PAD_BTN[i].cmd == 'D') fill = COL_SAFE;
    if (PAD_BTN[i].cmd == LOCAL_REC && sdRecording) fill = COL_STOP;
    if (PAD_BTN[i].cmd == 'B') fill = COL_ARM;
    drawButton(cx, cy, w, 40, PAD_BTN[i].top, PAD_BTN[i].bot, fill, COL_TEXT, 2);
  }
}

static void drawBench() {
  lcd.fillRect(0, BODY_Y, SCR_W, BODY_H, COL_BG);
  for (int i = 0; i < NBENCH; i++) {
    if (!BENCH_BTN[i].cmd) continue;
    int cx = (i % 4) * 80, cy = BODY_Y + (i / 4) * 32;
    uint16_t fill = (BENCH_BTN[i].cmd == LOCAL_PAD) ? COL_BTN_HI : COL_BTN;
    drawButton(cx, cy, 80, 32, BENCH_BTN[i].top, BENCH_BTN[i].bot, fill, COL_TEXT, 1);
  }
}

static void drawConsole(bool clearFirst) {
  if (clearFirst) lcd.fillRect(0, BODY_Y, SCR_W, BODY_H, COL_BG);
  lcd.setTextSize(1);
  lcd.setTextDatum(textdatum_t::top_left);
  lcd.setTextColor(COL_TEXT, COL_BG);
  const int rows = BODY_H / 8;
  int newest = (int)conHead - conScroll;
  char line[CON_COLS + 2];
  for (int r = rows - 1; r >= 0; r--) {
    int idx = newest - (rows - 1 - r);
    while (idx < 0) idx += CON_LINES;
    idx %= CON_LINES;
    portENTER_CRITICAL(&conMux);
    strncpy(line, conBuf[idx], CON_COLS);
    line[CON_COLS] = 0;
    portEXIT_CRITICAL(&conMux);
    size_t n = strlen(line);
    while (n < CON_COLS) line[n++] = ' ';
    line[CON_COLS] = 0;
    lcd.drawString(line, 2, BODY_Y + r * 8);
  }
  if (conScroll > 0) {
    lcd.setTextColor(COL_WARN, COL_BG);
    lcd.drawString("^ scrolled back, DOWN to follow", 2, BODY_Y);
  }
}

static const char *KEY_LBL[16] = {
  "7","8","9","<-", "4","5","6",",", "1","2","3","-", ".","0","ENTER","X"
};
#define KEY_ENTER  14
#define KEY_CANCEL 15
#define KEY_BKSP    3
#define KEY_COMMA   7

static void drawKeypad() {
  lcd.fillRect(0, BODY_Y, SCR_W, BODY_H, COL_BG);
  char l[56];
  snprintf(l, sizeof(l), "%c%s_", padCmd, padText);
  lcd.setTextSize(2);
  lcd.setTextDatum(textdatum_t::top_left);
  lcd.setTextColor(COL_OK, COL_BG);
  lcd.drawString(l, 6, BODY_Y + 3);
  lcd.setTextSize(1);
  lcd.setTextColor(COL_DIM, COL_BG);
  lcd.drawString(padNums == 2 ? "two values: pitch , yaw    ENTER sends, X cancels"
                              : "one value                  ENTER sends, X cancels",
                 6, BODY_Y + 23);
  for (int i = 0; i < 16; i++) {
    int cx = (i % 4) * 80, cy = BODY_Y + 36 + (i / 4) * 31;
    uint16_t f = (i == KEY_ENTER) ? COL_OK : (i == KEY_CANCEL) ? COL_STOP : COL_BTN;
    int sz = (i == KEY_ENTER) ? 1 : 2;
    drawButton(cx, cy, 80, 31, KEY_LBL[i], "", f, (i == KEY_ENTER) ? COL_BG : COL_TEXT, sz);
  }
}

static const char *TRIM_LBL[8] = {"P +.5","P -.5","Y +.5","Y -.5",
                                  "P +.1","P -.1","Y +.1","Y -.1"};
static const float TRIM_DP[8] = {0.5f,-0.5f,0,0, 0.1f,-0.1f,0,0};
static const float TRIM_DY[8] = {0,0,0.5f,-0.5f, 0,0,0.1f,-0.1f};

static void drawTrim() {
  lcd.fillRect(0, BODY_Y, SCR_W, BODY_H, COL_BG);
  char l[56];
  lcd.setTextDatum(textdatum_t::top_left);
  lcd.setTextSize(2);
  lcd.setTextColor(COL_OK, COL_BG);
  snprintf(l, sizeof(l), "pitch %+.2f   yaw %+.2f", trimP, trimY);
  lcd.drawString(l, 6, BODY_Y + 2);
  lcd.setTextSize(1);
  lcd.setTextColor(COL_DIM, COL_BG);
  lcd.drawString("saved to the rocket's flash on every tap", 6, BODY_Y + 22);
  for (int i = 0; i < 8; i++) {
    int cx = (i % 4) * 80, cy = BODY_Y + 34 + (i / 4) * 32;
    drawButton(cx, cy, 80, 32, TRIM_LBL[i], "", COL_BTN, COL_TEXT, 2);
  }
  drawButton(  0, BODY_Y + 100, 160, 32, "CENTRE", "", COL_BTN,  COL_TEXT, 2);
  drawButton(160, BODY_Y + 100, 160, 32, "BACK",   "", COL_BTN,  COL_TEXT, 2);
}

#define TT_COLS 4
#define TT_ROWS 3
#define TT_MARKS 24
static int16_t ttX[TT_MARKS], ttY[TT_MARKS];
static uint8_t ttN = 0, ttHead = 0;
static int     ttLastErr = -1, ttLastRawX = 0, ttLastRawY = 0;

static void ttTarget(int i, int &x, int &y) {
  x = (SCR_W / (TT_COLS + 1)) * ((i % TT_COLS) + 1);
  y = BODY_Y + 24 + (i / TT_COLS) * ((BODY_H - 40) / (TT_ROWS - 1));
}

static void drawTouchTest() {
  lcd.fillRect(0, BODY_Y, SCR_W, BODY_H, COL_BG);
  lcd.setTextSize(1);
  lcd.setTextDatum(textdatum_t::top_left);
  lcd.setTextColor(COL_DIM, COL_BG);
  lcd.drawString("tap each cross. RECAL bottom left, LOG to leave", 4, BODY_Y + 2);

  for (int i = 0; i < TT_COLS * TT_ROWS; i++) {
    int tx, ty; ttTarget(i, tx, ty);
    lcd.drawFastHLine(tx - 6, ty, 13, COL_DIM);
    lcd.drawFastVLine(tx, ty - 6, 13, COL_DIM);
  }
  for (int k = 0; k < ttN; k++) {
    int i = (ttHead - 1 - k + TT_MARKS * 2) % TT_MARKS;
    uint16_t c = (k == 0) ? COL_OK : COL_BTN_HI;
    lcd.fillCircle(ttX[i], ttY[i], (k == 0) ? 3 : 2, c);
  }
  if (ttLastErr >= 0) {
    char l[64];
    snprintf(l, sizeof(l), "raw %4d,%4d   error %d px", ttLastRawX, ttLastRawY, ttLastErr);
    lcd.setTextColor(ttLastErr > 8 ? COL_BAD : COL_OK, COL_BG);
    lcd.drawString(l, 4, FTR_Y - 12);
  }
  drawButton(0, FTR_Y - 34, 70, 20, "RECAL", "", COL_BTN, COL_TEXT, 1);
}

static void ttRecord(int x, int y) {
  uint16_t a, b;
  if (touchRaw(a, b)) { ttLastRawX = a; ttLastRawY = b; }
  ttX[ttHead] = x; ttY[ttHead] = y;
  ttHead = (ttHead + 1) % TT_MARKS;
  if (ttN < TT_MARKS) ttN++;
  int best = 1 << 20;
  for (int i = 0; i < TT_COLS * TT_ROWS; i++) {
    int tx, ty; ttTarget(i, tx, ty);
    int d = (tx - x) * (tx - x) + (ty - y) * (ty - y);
    if (d < best) best = d;
  }
  ttLastErr = (int)(sqrtf((float)best) + 0.5f);
  viewDirty = true;
}

static void redraw(bool full) {
  switch (view) {
    case VIEW_PAD:    drawPad();            break;
    case VIEW_BENCH:  drawBench();          break;
    case VIEW_CON:    drawConsole(full);    break;
    case VIEW_KEYPAD: drawKeypad();         break;
    case VIEW_TRIM:   drawTrim();           break;
    case VIEW_TOUCH:  drawTouchTest();      break;
  }
  if (full) drawFooter();
}

static bool calPoint(int tx, int ty, int idx, int total, float &rx, float &ry) {
  lcd.fillScreen(COL_BG);
  lcd.drawCircle(tx, ty, 10, COL_TEXT);
  lcd.drawFastHLine(tx - 14, ty, 28, COL_TEXT);
  lcd.drawFastVLine(tx, ty - 14, 28, COL_TEXT);
  char l[40];
  snprintf(l, sizeof(l), "%d of %d   hold on the cross", idx + 1, total);
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextColor(COL_DIM, COL_BG);
  lcd.drawString(l, SCR_W / 2, SCR_H / 2 + 46);
  lcd.setTextDatum(textdatum_t::top_left);

  uint16_t a, b;
  while (touchRaw(a, b)) delay(20);
  delay(150);

  uint32_t t0 = millis();
  while (millis() - t0 < 30000) {
    if (!touchRaw(a, b)) { delay(15); continue; }
    delay(200);
    double sx = 0, sy = 0; int n = 0;
    for (int i = 0; i < 16; i++) {
      if (!touchRaw(a, b)) break;
      sx += a; sy += b; n++;
      delay(8);
    }
    if (n < 10) { while (touchRaw(a, b)) delay(20); continue; }
    rx = (float)(sx / n);
    ry = (float)(sy / n);
    lcd.fillCircle(tx, ty, 5, COL_OK);
    while (touchRaw(a, b)) delay(20);
    return true;
  }
  return false;
}

#define CAL_POINTS 9
static void runCalibration() {
  const int TX[CAL_POINTS] = {28, SCR_W/2, SCR_W-28, 28, SCR_W/2, SCR_W-28, 28, SCR_W/2, SCR_W-28};
  const int TY[CAL_POINTS] = {28, 28, 28, SCR_H/2, SCR_H/2, SCR_H/2, SCR_H-28, SCR_H-28, SCR_H-28};
  float rx[CAL_POINTS], ry[CAL_POINTS], sx[CAL_POINTS], sy[CAL_POINTS];
  for (int i = 0; i < CAL_POINTS; i++) {
    if (!calPoint(TX[i], TY[i], i, CAL_POINTS, rx[i], ry[i])) return;
    sx[i] = TX[i]; sy[i] = TY[i];
    delay(200);
  }
  TouchCal c = {};
  bool ok = fitAxis(rx, ry, sx, CAL_POINTS, c.a, c.b, c.c)
         && fitAxis(rx, ry, sy, CAL_POINTS, c.d, c.e, c.f);
  if (!ok) return;
  c.valid = true;
  tcal = c;
  touchSave();

  float worst = 0, sum = 0;
  for (int i = 0; i < CAL_POINTS; i++) {
    float px = c.a * rx[i] + c.b * ry[i] + c.c;
    float py = c.d * rx[i] + c.e * ry[i] + c.f;
    float e = sqrtf((px - sx[i]) * (px - sx[i]) + (py - sy[i]) * (py - sy[i]));
    sum += e;
    if (e > worst) worst = e;
  }
  char m[96];
  snprintf(m, sizeof(m), "[touch cal: mean %.1f px, worst %.1f px over %d points]\n",
           sum / CAL_POINTS, worst, CAL_POINTS);
  conPuts(m);
  if (worst > 8.0f)
    conPuts("[worst is over 8 px. Re-run TOUCH test and check where it is off.]\n");
}

static void keypadKey(int i) {
  size_t n = strlen(padText);
  if (i == KEY_ENTER) {
    sendFor(padCmd, padNeed, padText);
    view = VIEW_CON; conScroll = 0; viewDirty = true;
    return;
  }
  if (i == KEY_CANCEL) { view = pageReturn; viewDirty = true; return; }
  if (i == KEY_BKSP)   { if (n) padText[n - 1] = 0; viewDirty = true; return; }
  if (i == KEY_COMMA && padNums < 2) return;
  if (n < sizeof(padText) - 1) { padText[n] = KEY_LBL[i][0]; padText[n + 1] = 0; }
  viewDirty = true;
}

static void sendTrim() {
  char arg[24];
  snprintf(arg, sizeof(arg), "%.2f,%.2f", trimP, trimY);
  sendFor('O', NEED_MAIN, arg);
}

static void pressBtn(const Btn &b) {
  if (b.cmd == LOCAL_REC) {
    if (sdRecording) { sdStop(); conPuts("[recording stopped]\n"); }
    else if (sdStart()) { char m[48]; snprintf(m, sizeof(m), "[recording to %s]\n", sdPath); conPuts(m); }
    else conPuts("[no SD card, or it could not be written]\n");
    viewDirty = true;
    return;
  }
  if (b.cmd == LOCAL_BENCH) { view = VIEW_BENCH; viewDirty = true; return; }
  if (b.cmd == LOCAL_PAD)   { view = VIEW_PAD;   viewDirty = true; return; }
  if (b.cmd == LOCAL_DUMPSD) {
    if (!sdRecording) {
      if (sdStart()) { char m[48]; snprintf(m, sizeof(m), "[recording to %s]\n", sdPath); conPuts(m); }
      else { conPuts("[no SD card. Dump would not be kept, not dumping.]\n"); viewDirty = true; return; }
    }
    conPuts("[dumping newest log to the card. RECORD stays on until you stop it.]\n");
    sendFor('L', NEED_BENCH, nullptr);
    view = VIEW_CON; conScroll = 0; viewDirty = true;
    return;
  }
  if (b.cmd == LOCAL_TOUCH) {
    ttN = 0; ttHead = 0; ttLastErr = -1;
    view = VIEW_TOUCH; viewDirty = true; return;
  }
  if (b.cmd == LOCAL_TRIM) {
    portENTER_CRITICAL(&tlmMux);
    trimP = tlm.offP; trimY = tlm.offY;
    portEXIT_CRITICAL(&tlmMux);
    view = VIEW_TRIM; viewDirty = true;
    return;
  }
  if (b.nums) {
    padCmd = b.cmd; padNums = b.nums; padNeed = b.need; padText[0] = 0;
    pageReturn = view; view = VIEW_KEYPAD; viewDirty = true;
    return;
  }
  sendFor(b.cmd, b.need, nullptr);
  bool arming = (b.cmd == 'A' || b.cmd == 'B' || b.cmd == 'D');
  if (!arming) { pageReturn = view; view = VIEW_CON; conScroll = 0; }
  viewDirty = true;
}

static void handleTouch(int x, int y) {
  if (y >= FTR_Y) {
    int b = x / 80; if (b > 3) b = 3;
    if (b == 3) { sendCmd(" "); conPuts("[stop sent]\n"); return; }
    if (b == 0) {
      if (view == VIEW_CON) view = pageReturn;
      else { pageReturn = view; view = VIEW_CON; }
      viewDirty = true;
      return;
    }
    const int rows = BODY_H / 8;
    if (b == 1) { conScroll += rows / 2;
                  if (conScroll > CON_LINES - rows) conScroll = CON_LINES - rows;
                  view = VIEW_CON; }
    if (b == 2) { conScroll -= rows / 2; if (conScroll < 0) conScroll = 0;
                  view = VIEW_CON; }
    viewDirty = true;
    return;
  }
  if (y < BODY_Y) return;

  switch (view) {
    case VIEW_PAD: {
      int col = x / 106; if (col > 2) col = 2;
      int row = (y - BODY_Y) / 40; if (row > 3) row = 3;
      int i = row * 3 + col;
      if (i >= 0 && i < NPAD && PAD_BTN[i].cmd) pressBtn(PAD_BTN[i]);
      break;
    }
    case VIEW_BENCH: {
      int col = x / 80; if (col > 3) col = 3;
      int row = (y - BODY_Y) / 32; if (row > 4) row = 4;
      int i = row * 4 + col;
      if (i >= 0 && i < NBENCH && BENCH_BTN[i].cmd) pressBtn(BENCH_BTN[i]);
      break;
    }
    case VIEW_KEYPAD: {
      int py = y - (BODY_Y + 36);
      if (py < 0) break;
      int col = x / 80; if (col > 3) col = 3;
      int row = py / 31;  if (row > 3) row = 3;
      int i = row * 4 + col;
      if (i >= 0 && i < 16) keypadKey(i);
      break;
    }
    case VIEW_TOUCH: {
      if (x < 70 && y >= FTR_Y - 34 && y < FTR_Y - 14) {
        runCalibration();
        lcd.fillScreen(COL_BG);
        ttN = 0; ttHead = 0; ttLastErr = -1;
        viewDirty = true;
        return;
      }
      ttRecord(x, y);
      break;
    }
    case VIEW_TRIM: {
      int py = y - (BODY_Y + 34);
      if (py < 0) break;
      if (py < 64) {
        int col = x / 80; if (col > 3) col = 3;
        int i = (py / 32) * 4 + col;
        if (i >= 0 && i < 8) {
          trimP = constrain(trimP + TRIM_DP[i], -TRIM_LIMIT, TRIM_LIMIT);
          trimY = constrain(trimY + TRIM_DY[i], -TRIM_LIMIT, TRIM_LIMIT);
          sendTrim(); viewDirty = true;
        }
      } else if (py >= 66 && py < 98) {
        if (x < 160) sendFor('G', NEED_BENCH, nullptr);
        else { view = VIEW_PAD; }
        viewDirty = true;
      }
      break;
    }
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < CON_LINES; i++) conBuf[i][0] = 0;

  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(200);
  lcd.fillScreen(COL_BG);
  hdrSpr.setColorDepth(16);
  hdrSprOk = hdrSpr.createSprite(SCR_W, HDR_H) != nullptr;

  touchBegin();
  uint16_t rx, ry;
  bool held = touchRaw(rx, ry);
  bool hadCal = touchLoad();
  if (!hadCal) touchDefault();
  if (!hadCal || held) {
    lcd.fillScreen(COL_BG);
    lcd.setTextDatum(textdatum_t::middle_center);
    lcd.setTextColor(COL_TEXT, COL_BG);
    lcd.drawString(hadCal ? "Recalibrating touch" : "No saved touch calibration",
                   SCR_W / 2, SCR_H / 2 - 8);
    lcd.drawString("nine crosses, hold each one still", SCR_W / 2, SCR_H / 2 + 8);
    lcd.setTextDatum(textdatum_t::top_left);
    delay(1800);
    runCalibration();
  }
  if (!tcal.valid) touchDefault();

  lcd.fillScreen(COL_BG);
  conPuts("TVC handset ready.\n");
  conPuts(sdBegin() ? "SD card found. REC on the PAD page starts a file.\n"
                    : "No SD card. Everything else still works.\n");
  conPuts("Broadcasting for a rocket on ESP-NOW channel 1.\n");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(LINK_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onRecv);
    addPeer(BCAST);
  } else {
    conPuts("ESP-NOW would not start. Power cycle the handset.\n");
  }
  redraw(true);
}

void loop() {
  static uint32_t lastHello = 0, lastHdr = 0, lastBody = 0, lastTouch = 0;
  uint32_t now = millis();

  if (!paired && now - lastHello >= 400) { lastHello = now; sendRaw(MSG_HELLO, nullptr, 0); }

  int tx, ty;
  if (touchGet(tx, ty)) {
    if (now - lastTouch > 280) { lastTouch = now; handleTouch(tx, ty); }
  }

  sdPoll();

  if (now - lastHdr >= 150) { lastHdr = now; drawHeader(); }

  static View drawnView = VIEW_KEYPAD;
  if ((viewDirty || (conDirty && view == VIEW_CON)) && now - lastBody >= 120) {
    lastBody = now;
    bool full = viewDirty || view != drawnView;
    drawnView = view;
    conDirty = false;
    viewDirty = false;
    redraw(full);
  }
}
