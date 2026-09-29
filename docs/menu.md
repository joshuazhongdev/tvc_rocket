# Handset button reference

Two pages and a footer. Everything here maps onto a single-letter command in the
rocket's serial menu, so the screen and a laptop terminal always agree.

Buttons marked **(number)** open the keypad first. ENTER sends, X cancels, and
an empty entry sends the bare command so the rocket uses its default.

---

## PAD page — launch day

| button | sends | what it does |
|---|---|---|
| **CALIBRATE** | `K` | Forces a re-zero on the next stillness window, ignoring the 5 s rate limit. Sets the attitude reference every later angle is measured against, so take it **on the rail, in flight attitude**, and hold still about 2 seconds. Refused while armed. |
| **ARM** | `A` | Lets the rocket detect liftoff. Until it is armed, a 2 g spike is just you bumping it. Refused unless on the pad and zeroed at least once. |
| **DISARM** | `D` | Takes arming back and blocks auto re-arm, which would otherwise happen 20 s later on the stillness timer. **Never refused, under any condition.** Also centres the gimbal. |
| **CENTRE** | `G` | Both axes to zero deflection, through the linkage solver. |
| **TRIM** | local | Opens the trim page. ±0.5 and ±0.1 degree nudges per axis, written to the rocket's flash on every tap. |
| **RECORD** | local | Starts or stops recording everything the rocket prints to the handset's microSD card, as `/tvcNNN.txt`. |
| **LOGS** | `F` | Lists the flight log slots in the rocket's flash: which are used, how long, when. |
| **DUMP** (number) | `L<slot>` | Dumps one slot as CSV. Empty sends `L`, which dumps the most recent. Turn RECORD on first if you want it on the card. |
| **BENCH** | local | Opens the test page. |

---

## BENCH page — tests and tools

Numbered in the order you run them. That ordering is the point: test 1 can send
you back to a reflash, and test 8 depends on test 1 having been applied.

| button | sends | what it does |
|---|---|---|
| **1 AXIS map** | `1` | Measures the rotation between the IMU's axes and the gimbal's, and prints the mixing matrix. **Run this first.** Getting it wrong means every command lands on the wrong servo while the estimator stays perfectly correct. |
| **2 SERVO travel** | `2` | Sweeps both servos to their limits. Finds binding, and the real deflection you get at the extremes rather than the one the model predicts. |
| **3 IMU health** | `3` | Accelerometer magnitude, noise, and the bias left after calibration. A quick pass or fail on the sensor. |
| **4 FILTER noise** | `4` | 30 s with the servos still. The filter's noise floor with nothing disturbing it. |
| **5 NOISE + servo** | `5` | 10 s with the servos sweeping. The difference from test 4 is how much servo motion couples into the IMU through the airframe. |
| **6 GYRO drift** | `6` | 30 s gyro-only. **This is the flight configuration**, because under thrust the accelerometer measures thrust rather than gravity. The drift you measure here is your flight error budget. |
| **7 GYRO return** | `7` | Tilt away and back by hand. Checks the gyro-only estimate comes home to zero rather than accumulating. |
| **8 CLOSED loop** | `8` | **Servos move.** Tilt it by hand and the gimbal should push back. Scores the tilt-versus-command correlation and prints an explicit verdict. Both axes must be negative. This is the go or no-go test. |
| **LIVE csv** | `C` | Streams raw CSV until you press STOP. For watching the estimator rather than testing anything. |
| **GYRO only** | `g` | Toggles the gyro-only estimator on and off, so you can compare flight configuration against bench configuration on the same motion. |
| **GIMBAL degrees** (number) | `G<p>,<y>` | Command a deflection in **degrees of thrust**, solved through the four-bar. Deliberately unclamped: this is the tool for finding where it locks up. It is still guarded against the mirror solution past a toggle. |
| **ENVELOPE walk** (number) | `W<radius>` | Walks 12 bearings around a circle of that radius, one second each, and reports which are reachable. Empty uses the gimbal cap. This is the test that finds a corner you cannot reach. |
| **RAW servo** (number) | `S<p>,<y>` | **Raw servo degrees, model bypassed.** Every other command asks the linkage model where to put the servo, which makes them useless for checking whether the model is right. This one writes the angle you name and then reports what the model *thinks* it produced. Measure the real nozzle angle and compare. |
| **ERASE logs** (number) | `E<slot>` | Erases one slot, or all of them if you send it empty. |
| **GAIN kP / kI / kD** (number) | `p` `i` `d` | Sets the bench PID gains used by test 8. These are not the flight gains; those are compiled in. |
| **TOUCH test** | local | Twelve crosses, every tap drawn where the calibration says it landed, with the error in pixels. RECAL in the bottom left re-runs the nine-point calibration. |
| **EXIT bench** | `q` | Leaves bench mode and hands the rocket back to its main loop. |
| **< PAD back** | local | Returns to the launch page. |

---

## Footer — the same in every view

| button | what it does |
|---|---|
| **LOG / BACK** | Opens the console, or returns to the page you came from. |
| **UP / DOWN** | Scroll the console. Incoming text does not drag you forward while scrolled back. |
| **STOP** | Sends a single space, which aborts any running test. Harmless at the menu. Its position never changes, because STOP has to be hittable without looking. |

---

## What you do not have to think about

The rocket has **two dispatchers** and only one runs at a time. `benchRun()`
blocks the main loop, so `A`, `D`, `K` and `O` cannot be heard while a bench
session is open; the numbered tests cannot be heard while it is closed. Neither
has a default case, so a command sent to the wrong one silently does nothing.

The handset sends `t` or `q` before every command to put the rocket where the
command can be heard. It does this **unconditionally**, not only when it thinks
the rocket is in the other mode, because with stale telemetry it cannot know.
That was a real bug: DISARM pressed on a quiet link did nothing at all.
