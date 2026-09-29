// ---------------------------------------------------------------------------
//linkproto.h: the wire format between the rocket and the CYD handset
//
//this file exists twice, byte for byte, once in each PlatformIO project,
//since the two are separate builds for different chips with no shared
//include path:
//
//     <project>/src/linkproto.h          rocket, ESP32-S3
//     <project>/cyd/src/linkproto.h      handset, ESP32
//
//change one copy and update the other to match, or the two ends will
//silently disagree about where fields start; LINK_PROTO_VER guards
//against that, since both ends check it and refuse to pair on a mismatch
// ---------------------------------------------------------------------------
#pragma once
#include <stdint.h>

#define LINK_MAGIC       0x31435654UL   //"TVC1" little-endian
#define LINK_PROTO_VER   2   //bumped when offP/offY joined LinkTlm
#define LINK_CHANNEL     1              //both ends must agree; ESP-NOW does not negotiate channels
#define LINK_MAX_PAYLOAD 238            //250 byte ESP-NOW limit minus the 12 byte header; the static_assert below guards it

enum : uint8_t {
  MSG_HELLO = 1,   //handset -> rocket, "I am here"
  MSG_TEXT  = 2,   //rocket  -> handset, console output, not newline aligned
  MSG_CMD   = 3,   //handset -> rocket, keystrokes for the bench menu
  MSG_TLM   = 4,   //rocket  -> handset, the status header
};

struct __attribute__((packed)) LinkHdr {
  uint32_t magic;
  uint8_t  ver;
  uint8_t  type;
  uint8_t  len;      //payload bytes following this header
  uint8_t  pad;
  uint16_t seq;      //per-type, wraps; detects loss, never reorders
  uint16_t rsvd;
};

//status header, sent at 10 Hz; plain floats and bytes, safe since both ends are little-endian Xtensa
struct __attribute__((packed)) LinkTlm {
  uint32_t ms;          //rocket millis()
  float    pitch, yaw;  //Kalman attitude, degrees
  float    tilt;        //pole-free arrow estimator, degrees
  float    cmdP, cmdY;  //last gimbal command, degrees of deflection
  float    svP,  svY;   //servo angles actually written
  float    accelG;      //accelerometer magnitude
  float    offP,  offY; //servo trim currently in force, degrees from centre
  uint8_t  state;       //0 PAD, 1 BOOST, 2 COAST
  uint8_t  flags;       //see LF_* below
  uint16_t textDropped; //console bytes lost to a full transmit buffer
};

#define LF_ARMED     0x01
#define LF_BENCH     0x02
#define LF_ZEROED    0x04
#define LF_GYROONLY  0x08
#define LF_TVCON     0x10
#define LF_LOGGING   0x20
#define LF_INHIBIT   0x40   //a manual disarm is blocking auto-arm

//catches a LinkHdr field addition at compile time
static_assert(sizeof(LinkHdr) + LINK_MAX_PAYLOAD <= 250,
              "LinkHdr + LINK_MAX_PAYLOAD exceeds the 250 byte ESP-NOW frame");
