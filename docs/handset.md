# The wireless handset

Runs the rocket's whole bench menu with no data cable. ESP-NOW between the
ESP32-S3 flight computer and an ESP32-2432S028R touchscreen, the "Cheap Yellow
Display", about $12.

---

## The one design decision that matters

**The rocket keeps exactly one command dispatcher.** The handset is a keyboard
and a terminal for it, not a second implementation.

`src/console.h` defines `con`, a `Stream` that reads from USB serial **and** the
radio and writes to both. Files opt in with two lines before their other
includes:

```cpp
#define TVC_USE_CONSOLE
#include "console.h"
```

which rewrites `Serial` to `con` for the rest of that file.

**A command added to the bench menu is reachable from the handset the moment it
compiles.** The alternative, a second dispatcher on the handset, would have
drifted the first time a test was renumbered.

`flightlog.cpp` opts in even though it prints nothing itself: `flightlog.h`'s
helpers are `inline`, so a translation unit compiling them against the real
`Serial` while others compile them against `con` is an ODR violation, and the
linker is free to keep the copy that drops log output off the handset.

---

## Behaviour worth knowing

**Telemetry rides the console pump, not `loop()`.** `benchRun()` blocks, so
`loop()` does not execute at all during a bench session, which is exactly when
live attitude matters. `linkPoll()` is called from every console read and write
instead, and the bench does those constantly.

**The radio dies at liftoff, not at arming.** It used to die the moment the
vehicle armed, which made a remote disarm impossible: you could arm from the
handset and then had no way to take it back. On the pad the control loop is
idle, so WiFi costs nothing; from ignition it would be stealing interrupt time
from the only 25 ms that matter.

**Text is best effort, telemetry is not.** If a test prints faster than ESP-NOW
drains, the oldest bytes are dropped and counted, and the count rides in the
status header. Dropping beats blocking: stalling a print inside the closed-loop
test would corrupt the 200 Hz loop it is measuring. The flash log, not the
handset, is the record of a CSV run.

**Mode switching is unconditional.** `A`, `D`, `K` and `O` are handled in the
rocket's main loop, which `benchRun()` blocks; the numbered tests are handled
inside `benchRun()`, which the main loop does not know. So the handset sends `t`
or `q` before every command. It used to send them only when it believed the
rocket was in the other mode, and with stale telemetry it believed nothing, so
the command went to the wrong loop and vanished. **Neither loop has a default
case.**

---

## The SD card forces one hardware decision

The card is hard-wired to SPI3 at pins 18/19/23/5 and the touch panel to
25/32/39/33. One SPI host drives one pin set, so they cannot share SPI3, and
SPI2 already has the display.

The card gets the hardware. **The touch panel is bit-banged in software**
(`cyd/src/touch.h`). It is a four-wire, 24-bit exchange polled 20 times a
second, so software costs nothing here, and it avoids swapping the whole
graphics stack for a library combination that supports all three.

Calibration is **affine**, fitted by least squares to four corner touches:

```
screenX = a*rawX + b*rawY + c
screenY = d*rawX + e*rawY + f
```

The cross terms are what make it robust. A panel mounted rotated or mirrored
relative to the display comes out right without anyone working out which of the
eight possible flips this unit has. A min/max calibration cannot do that, and
getting the flip wrong gives you a screen where every button works except it is
the wrong button.

Hold a finger on the screen while the board boots to recalibrate.

---

## Testing it without hardware

`cyd/test/` compiles the real `cyd/src/main.cpp` against stubs for Arduino,
LovyanGFX, SD and Preferences, wires its radio to a model of the rocket, and
drives simulated touches through every button.

```
cd cyd/test && ./build.sh && ./test_ui && ./test_touch
```

216 checks, two seconds, no board.

The rocket model reproduces the two behaviours that actually bite: `benchRun()`
blocks the main loop, and neither dispatcher has a default case, so a command
sent to the wrong one is a silent no-op.

### Two lessons about the tests themselves

- A **pixel sweep passed while a corner test failed**, because the sweep
  computed the expected cell with the same `x / 106` the code used. A test that
  reuses the implementation's arithmetic cannot find an error in that
  arithmetic. The corner test used the drawn width instead, and caught a
  two-pixel strip on the right edge that fired the wrong button.
- A **trim test passed until the mock started clamping** the way the firmware
  does. An unfaithful mock is worse than no mock: it puts a green check over a
  real bug.
