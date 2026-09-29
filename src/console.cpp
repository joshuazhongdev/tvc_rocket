// ---------------------------------------------------------------------------
// console.cpp  -  ESP-NOW transport behind the console. See console.h.
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>
#include "console.h"

Console con;

// --- buffers ---------------------------------------------------------------
//
//two rings, different owners:
//txRing: written by the bench (main task), drained by linkPoll (main task), no locking needed
//rxRing: written by the ESP-NOW receive callback (WiFi task, other core), drained by the main task; needs a spinlock

#define TX_RING 4096
#define RX_RING 512

static uint8_t  txBuf[TX_RING];
static uint16_t txHead = 0, txTail = 0;        // head = next write, tail = next read
static uint16_t txDropped = 0;

static uint8_t  rxBuf[RX_RING];
static volatile uint16_t rxHead = 0, rxTail = 0;
static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

static const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static uint8_t  peerMac[6]  = {0,0,0,0,0,0};
static bool     paired      = false;
static bool     radioUp     = false;
static uint32_t lastSendMs  = 0;
static uint32_t lastTlmMs   = 0;
static uint16_t seqText = 0, seqTlm = 0;
static void (*tlmSrc)(LinkTlm &) = nullptr;
static volatile bool quiet = false;

//3 ms of spacing is about 80 kB/s of headroom, well above what any test produces
#define TX_GAP_MS 3

bool linkPaired()          { return paired; }
bool linkAlive()           { return radioUp; }
const uint8_t *linkPeerMac(){ return peerMac; }

static inline uint16_t txCount() { return (uint16_t)((txHead - txTail) & (TX_RING - 1)); }

static void txPush(uint8_t c) {
  uint16_t next = (uint16_t)((txHead + 1) & (TX_RING - 1));
  if (next == txTail) {
    //drops the oldest byte on overflow, so output doesn't block
    txTail = (uint16_t)((txTail + 1) & (TX_RING - 1));
    if (txDropped < 0xFFFF) txDropped++;
  }
  txBuf[txHead] = c;
  txHead = next;
}

static void rxPush(const uint8_t *d, int n) {
  portENTER_CRITICAL(&rxMux);
  for (int i = 0; i < n; i++) {
    uint16_t next = (uint16_t)((rxHead + 1) & (RX_RING - 1));
    if (next == rxTail) break;          //drops the newest byte on overflow
    rxBuf[rxHead] = d[i];
    rxHead = next;
  }
  portEXIT_CRITICAL(&rxMux);
}

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
  if (h.magic != LINK_MAGIC || h.ver != LINK_PROTO_VER) return;
  if (h.len > len - (int)sizeof(LinkHdr)) return;

  //first valid packet pairs; a second handset is ignored
  if (!paired) {
    memcpy(peerMac, mac, 6);
    if (addPeer(peerMac)) paired = true;
  } else if (memcmp(peerMac, mac, 6) != 0) {
    return;
  }

  if (h.type == MSG_CMD && h.len)
    rxPush(data + sizeof(LinkHdr), h.len);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  handleRecv(info->src_addr, data, len);
}
#else
static void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  handleRecv(mac, data, len);
}
#endif
//no send callback: signature differs across ESP-IDF releases; paced by wall clock instead

static bool sendFrame(uint8_t type, uint16_t seq, const uint8_t *payload, uint8_t n) {
  if (!radioUp) return false;
  uint8_t pkt[sizeof(LinkHdr) + LINK_MAX_PAYLOAD];
  LinkHdr h = {};
  h.magic = LINK_MAGIC;
  h.ver   = LINK_PROTO_VER;
  h.type  = type;
  h.len   = n;
  h.seq   = seq;
  memcpy(pkt, &h, sizeof(h));
  if (n) memcpy(pkt + sizeof(h), payload, n);
  const uint8_t *dst = paired ? peerMac : BCAST;
  return esp_now_send(dst, pkt, sizeof(h) + n) == ESP_OK;
}

void linkBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);        //station mode for the MAC, no association
  esp_wifi_set_ps(WIFI_PS_NONE);        //power save would add latency for no benefit here
  esp_wifi_set_channel(LINK_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { radioUp = false; return; }
  esp_now_register_recv_cb(onRecv);
  addPeer(BCAST);
  radioUp = true;
}

void linkArmedShutdown() {
  if (!radioUp) return;
  radioUp = false;
  paired  = false;
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
}

void linkSetTlmSource(void (*fn)(LinkTlm &)) { tlmSrc = fn; }

void linkSetQuiet(bool q) { quiet = q; }

void linkPoll() {
  if (!radioUp) return;
  //silent during the burn; buffered text goes out once burnout clears this flag
  if (quiet) return;
  uint32_t now = millis();

  //sent before the paired check: while unpaired this goes out as a broadcast, which is how the handset finds the rocket
  if (tlmSrc && now - lastTlmMs >= 100) {
    lastTlmMs = now;
    LinkTlm t = {};
    tlmSrc(t);
    linkSendTlm(t);
  }

  if (!paired) return;
  if (!txCount()) return;
  if (now - lastSendMs < TX_GAP_MS) return;

  uint8_t chunk[LINK_MAX_PAYLOAD];
  uint8_t n = 0;
  while (n < LINK_MAX_PAYLOAD && txCount()) {
    chunk[n++] = txBuf[txTail];
    txTail = (uint16_t)((txTail + 1) & (TX_RING - 1));
  }
  if (!sendFrame(MSG_TEXT, seqText++, chunk, n)) {
    //radio busy; bytes are already out of the ring and not re-queued
    if (txDropped < 0xFFFF - n) txDropped += n;
  }
  lastSendMs = now;
}

void linkSendTlm(const LinkTlm &t) {
  if (!radioUp || quiet) return;
  LinkTlm c = t;
  c.textDropped = txDropped;
  sendFrame(MSG_TLM, seqTlm++, (const uint8_t *)&c, sizeof(c));
}

// --- Console ---------------------------------------------------------------

size_t Console::write(uint8_t c) {
  Serial.write(c);
  if (radioUp) {
    txPush(c);
    //pumps on newline, not every byte, to avoid calling millis() thousands of times a second during a CSV test
    if (c == '\n') linkPoll();
  }
  return 1;
}

size_t Console::write(const uint8_t *b, size_t n) {
  Serial.write(b, n);
  if (radioUp) {
    bool nl = false;
    for (size_t i = 0; i < n; i++) { txPush(b[i]); if (b[i] == '\n') nl = true; }
    if (nl) linkPoll();
  }
  return n;
}

int Console::available() {
  //keeps the transmit buffer draining during tests that never return to loop()
  linkPoll();
  int s = Serial.available();
  if (s > 0) return s;
  portENTER_CRITICAL(&rxMux);
  int r = (int)((rxHead - rxTail) & (RX_RING - 1));
  portEXIT_CRITICAL(&rxMux);
  return r;
}

int Console::read() {
  linkPoll();
  if (Serial.available() > 0) return Serial.read();
  int v = -1;
  portENTER_CRITICAL(&rxMux);
  if (rxHead != rxTail) {
    v = rxBuf[rxTail];
    rxTail = (uint16_t)((rxTail + 1) & (RX_RING - 1));
  }
  portEXIT_CRITICAL(&rxMux);
  return v;
}

void Console::flush() {
  Serial.flush();
  //pushes what is queued; bounded so a wedged radio cannot hang flush()
  for (int i = 0; i < 64 && txCount(); i++) { linkPoll(); delay(TX_GAP_MS); }
}

int Console::peek() {
  if (Serial.available() > 0) return Serial.peek();
  int v = -1;
  portENTER_CRITICAL(&rxMux);
  if (rxHead != rxTail) v = rxBuf[rxTail];
  portEXIT_CRITICAL(&rxMux);
  return v;
}
