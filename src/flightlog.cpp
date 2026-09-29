// ---------------------------------------------------------------------------
//flightlog.cpp: the recorder's state, defined exactly once
//
//flightlog.h is included by both main.cpp and bench.cpp; a `static` in the
//header would give each translation unit its own private copy of the ring
//buffer and counters, letting logTick() fill one copy while logSaveNow()
//reads another with no compiler warning
//one definition here, with `extern` declarations in the header, avoids that
// ---------------------------------------------------------------------------
//uses the same console redirect as main.cpp and bench.cpp, so flightlog.h's inline helpers bind to the same Serial/con across translation units
#define TVC_USE_CONSOLE
#include "console.h"

#include "flightlog.h"

LogRec   logBuf[LOG_SLOTS];
uint32_t logHead = 0, logCount = 0;
bool     logRunning = false, logSaved = false;
uint32_t logBootId = 0;
uint32_t logEpoch0 = 0;
uint16_t logPeakMg = 0;
uint32_t logBoostFirstMs = 0, logBoostLastMs = 0;
float    logMinVz = 1.0f;
bool     logReady = false;
