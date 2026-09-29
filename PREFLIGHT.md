# TVC rocket: everything left before launch

Status as of 2026-09-12. Ordered by phase. Items marked **CRITICAL** are the
ones where skipping them either loses the flight or loses the data.

---

## Done since this list was written

- **Onboard logging.** RAM ring, 12.5 s at 200 Hz, written to flash after the
  flight. Files are numbered and never overwritten, and when space runs out the
  least flight-like file is evicted, so a pothole on the drive home can never
  destroy a real flight. Boot-counter timestamps, optional real clock.
- **`readlog.py`** downloads, saves CSV and plots. `--list`, `--slot`,
  `--erase`, `--set-clock`, `--plot`.
- **Automatic pad re-zero**, repeating every 5 s while still. No command input.
- **Arming interlock**, latched, gated on a re-zero, disarms if you pick the
  rocket up.
- **In-flight reset guard** and **stall failsafe**.
- **Exact linkage solver** in `linkage.h`, PID now commands gimbal degrees.
- **IR arming** written but compiled out behind `USE_IR_ARM 0`.

---

## Phase 0: measurements (blocks everything else)

Nothing downstream is trustworthy until these are real numbers rather than
placeholders.

1. **Linkage dimensions, per axis** — `hornR`, `rodL`, `armR`, `pivotX`,
   `pivotY`. Calipers. Pitch and yaw separately, they are probably not
   identical. Goes into `setupLinkages()` in `main.cpp` and `LINK` in
   `tvc_sim.py`.
2. **Servo trim, per axis** — the `write()` value that makes the nozzle
   actually square. Set it by eye with bench test 1.
3. **Liftoff mass** including the motor. Sets your g-loading and therefore how
   badly the accelerometer is swamped.
4. **CG position** from the tail. Target roughly 40 to 45 percent of body
   length for best gimbal authority.
5. **Moment of inertia** about pitch and yaw. Bifilar swing, or take it from
   CAD. Goes into `INERTIA` in the sim.
6. Update `tvc_sim.py`, re-run `--sweep`, pick gains. **The gains changed
   meaning** when the PID moved to gimbal degrees, so old values do not carry
   over.

---

## Phase 1: firmware still to write

### CRITICAL

~~7.~~ **DONE** Onboard logging.** LittleFS on the S3's flash, full state at 200 Hz,
   dumped over serial afterwards. Roughly 8 KB/s, so a 10 second flight is
   under 100 KB. Without this a failed flight teaches you nothing, and there
   is no serial cable on a flying rocket.

~~8.~~ **DONE** Automatic pad re-zero, no command input.** The rocket is sealed, so this
   has to happen on its own. Design:
   - While in `STATE_PAD`, watch for stillness: gyro magnitude under about
     1 deg/s and accelerometer magnitude within about 0.05 g of 1 g,
     continuously for 2 seconds.
   - On stillness, re-measure gyro bias and re-run the orientation zero.
     Reset `kalAnglePitch`, `kalAngleYaw`, the covariances, and re-seed the
     arrow `V` from the measured gravity direction.
   - Re-zero repeatedly, not once. The rocket may sit on the pad for minutes
     and gyro bias drifts with temperature.
   - Emit a `# EVENT rezero` line so the log shows when it last happened.
   - This fixes the failure where you power on in the car and the rocket
     spends the flight holding the car's attitude.

~~9.~~ **DONE** Arming interlock.** Boost detection must not go live until at least one
   successful re-zero has happened and the rocket has been still since.
   Currently any 2 g bump for 25 ms latches BOOST, which loading the rocket
   onto the rail can do.

~~10.~~ **DONE** In-flight reset guard.** If a brownout resets the board mid-burn it will
    re-run calibration while pulling 5 g and produce garbage. At boot, if the
    accelerometer is not near 1 g, skip calibration and go to a safe state.

~~11.~~ **DONE** Servo failsafe / watchdog.** If the loop stalls the servos hold their
    last position. Feed a watchdog, and centre the gimbal if it trips.

### New, from what we found along the way

12a. **Bulk capacitor, 470 to 1000 uF, across the ESP32's 5 V feed.** The
    servos spike hard and a brownout during boost loses the RAM buffer before
    it reaches flash. This prevents the reset rather than surviving it, and it
    is the single cheapest reliability item on this list.

12b. **Early flush.** The log still has a roughly 3.8 second window between
    ignition and the flash write. Flushing at burnout plus 300 ms, then again
    at plus 8 s for the coast data, shrinks that to about 1.1 seconds.

12c. **Confirm the LittleFS partition has room** on the first `pio run`. Each
    file is up to 70 KB and the default 8 MB scheme gives about 1.5 MB.

### Should have

12. **OTA update over WiFi.** Insurance against the USB port becoming hard to
    reach now that the ESP32 is mounted permanently.
13. **Add flight state and gimbal command to the live serial stream.** Both are
    in the flight log already, but the `b` stream is still 7 fields and shows
    neither. The visualizer parses 2, 4 or 7 fields, so it changes in step.
14. **Battery voltage on an ADC pin.** The single most useful diagnostic if
    the board resets during the burn.

---

## Phase 2: bench tests, in this order

15. **Test 9, axis and sign mapping.** First, because its result may send you
    back to a reflash or a connector swap. Sets `branchUp` and `dir`.
16. **Test 1, servo travel.** Confirms no binding, and gives real gimbal
    deflection at the servo limits to check against `linkage.py`.
17. **Test 2, IMU health.** Magnitude, noise, residual bias, axis map.
18. **Test 3, filter static noise, 30 s.** Baseline numbers for the config
    record.
19. **Test 4, servo vibration coupling.** How much servo buzz reaches the IMU.
20. **Test 7, gyro-only drift.** The flight configuration. The scaled 0.8 s
    number is your actual flight error budget.
21. **Test 8, gyro-only return to mark.** Catches a wrong `GYRO_SENSITIVITY`.
22. **Test 5, closed-loop demo.** Only after 9 has been applied. Tilt by hand,
    the gimbal must push back. If it slams to a stop, the sign is still wrong.
23. **Then set `tvcEnabled = true`.** Not before.

---

## Phase 3: hardware still to do

24. **Switch in the ESP32 power feed only**, not the main battery line, so the
    servos stay powered from the battery while the board runs on USB.
25. **Battery JST brought out** through the airframe as the master disconnect.
26. **Star ground at the battery negative.** Servo ground and ESP32 ground both
    go back to the battery separately. Do not daisy-chain servo current through
    the board.
27. **Confirm reachable after mounting:** USB port, the switch, the battery
    plug, and ideally the two servo connectors.
28. **CG check** against the 40 to 45 percent target, adjusting by moving
    components rather than adding nose ballast.

---

## Phase 4: static fire

29. **Test stand** with restraint rated several times peak thrust. A C11 peaks
    at 21.7 N.
30. **Logging confirmed running and writing** before the motor goes in. Put it
    on the checklist as a line item.
31. **Capture the accelerometer magnitude trace through the burn.** This is the
    one number the bench cannot give you, and it sets `BOOST_G` and
    `BURNOUT_G` properly. Currently they are guesses.
32. Two camera angles, one tight on the gimbal, one wide, with a sync marker.
33. Remember what a restrained fire does and does not test: servo authority
    under load, gimbal binding, mount survival, electronics through vibration.
    **Not** your gains, since the vehicle cannot rotate.

---

## Phase 5: recovery and motor

34. **Decide the delay.** On a 200 g airframe the C11-7 ejects roughly three
    seconds past apogee, falling. A C11-3 lands much closer. Recheck against
    your measured mass.
35. **Ejection ground test.** Confirm the charge actually pushes the nose off
    and draws the chute, with the airframe assembled as it will fly.
36. **Plan for the finless coast.** The rocket tumbles from burnout to
    ejection. Decide whether that is acceptable and what it does to the
    deployment.

---

## Phase 6: documentation

37. **Config record, one per run.** Firmware git hash, all gains, filter
    constants, linkage dimensions, mass, CG, motor. Filled in before the run.
38. **Test card per test.** The question, the numeric prediction, pass
    criteria, abort criteria.
39. **Hazard sheet and go/no-go**, including the misfire procedure: key out,
    controller disconnected, 60 second wait before approach.
40. **Pre-fire and pre-flight checklists**, printed, with boxes.
41. **Run log** filled in during, not from memory afterwards.
42. **Post-test writeup** while it is fresh.

---

## Phase 7: launch day

43. Site, and whatever authorisation it requires.
44. Weather and wind limits written down in advance so the decision is not made
    under pressure with a motor installed.
45. Standoff distance, extinguisher, water.
46. Power-on sequence: rocket vertical on the pad **before** the avionics are
    switched on, LED blink confirmed visible from outside, then arm.

---

## The shortest path

If time is tight, the order that preserves the most value is:

1. **Measurements** (phase 0). Nothing downstream is real until these are.
2. **Bulk capacitor** (12a). Ten minutes, and it protects everything else.
3. **Bench test 9**, then 1, 2, 7 (items 15, 16, 17, 20).
4. **Static fire** with logging on (phase 4), then set the real thresholds.
5. **Recovery and motor delay** (phase 5).
6. Everything else.

Phase 0 is now the only thing blocking. The firmware critical path is clear.
