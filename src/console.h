// ---------------------------------------------------------------------------
//console.h: one console, two pipes
//
//con is a Stream that reads/writes both USB serial and the radio, so a
//wireless handset mirrors the bench menu without a second dispatcher
//
//every file that wants its output mirrored writes:
//
//     #define TVC_USE_CONSOLE
//     #include "console.h"
//
//before its other includes; the macro at the bottom rewrites Serial to con
//
//must be included first, before flightlog.h, whose printing is inline
//
//radio text is best effort: oldest bytes drop on overflow rather than
//block, since stalling inside test 8 would corrupt the 200 Hz loop it
//measures; the drop count is shown in the telemetry header
//
//radio is off in flight: linkArmedShutdown() kills it on arm, since WiFi
//jitter is not worth risking in the control loop
// ---------------------------------------------------------------------------
#pragma once
#include <Arduino.h>
#include "linkproto.h"

// --- transport -------------------------------------------------------------

//brings up WiFi in station mode, parks it on LINK_CHANNEL, starts ESP-NOW
void linkBegin();

//drains the transmit buffer; called from every console read and write
void linkPoll();

//pushes the status header immediately
void linkSendTlm(const LinkTlm &t);

//registers the status-header callback; linkPoll() sends one every 100 ms.
//a callback, not a loop() call, since benchRun() blocks and loop() does not run during a test
void linkSetTlmSource(void (*fn)(LinkTlm &));

//true once a handset has been heard from and locked on
bool linkPaired();

//six byte MAC of the paired handset, or all zeroes
const uint8_t *linkPeerMac();

//goes silent without tearing the radio down; set on boost, cleared at burnout so the handset recovers on its own
void linkSetQuiet(bool quiet);

//tears the radio down completely, irreversible until reboot
void linkArmedShutdown();

//false once linkArmedShutdown() has run
bool linkAlive();

// --- the console itself ----------------------------------------------------

class Console : public Stream {
public:
  //forwarders so `#define Serial con` survives the two places main.cpp uses Serial as something other than a Stream
  void begin(unsigned long baud) { Serial.begin(baud); }
  explicit operator bool() { return (bool)Serial; }

  size_t write(uint8_t c) override;
  size_t write(const uint8_t *b, size_t n) override;
  int    available() override;
  int    read() override;
  int    peek() override;
  void   flush() override;          //drains USB and the radio buffer
  using Print::write;
};

extern Console con;

#ifdef TVC_USE_CONSOLE
//deliberately a macro: renames every call site in a 700-line file without editing them
#define Serial con
#endif
