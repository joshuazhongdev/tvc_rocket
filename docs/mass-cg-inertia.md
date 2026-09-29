# Mass, CG and inertia, and the gains that come out of them

`tvc_sim.py` needs exactly four numbers about your vehicle. Everything else in
it is either measured elsewhere or comes from the motor data sheet.

| line in tvc_sim.py | what it is | how you get it |
|---|---|---|
| `MASS_KG` | liftoff mass, motor installed | kitchen scale |
| `L_GIMBAL_M` | gimbal **pivot** to CG, metres | balance test |
| `INERTIA` | pitch/yaw inertia about the CG, kg m² | bifilar pendulum |
| `THRUST_N`, `BURN_S` | motor data, confirmed by static fire | |

Estes C11 for reference: 8.8 Ns total, 10.9 N average, 21.7 N peak, 0.8 s burn,
35 g total, 12 g propellant, 24 × 70 mm.

---

## 1. Mass

Weigh the complete rocket with the motor in, ready to fly. Everything: nose
cone, electronics, battery, wadding, chute.

**Check thrust to weight before anything else.** Standard guidance is 5:1; the
floor people accept is 3. Below that the rocket leaves the rail too slowly to be
controllable and no gain fixes it.

Worth knowing if you are heavy: **the Estes black powder line does not help.**
C11, D12 and E12 all average 10 to 11 N. A D and an E carry more total impulse
than a C, but they spend it by burning longer, not harder. If your
thrust-to-weight is the problem, the options are losing mass or a composite
motor, not a bigger Estes.

---

## 2. CG

Balance the **loaded** rocket across a ruler edge. Mark the balance point,
measure from there to the **gimbal pivot**.

**Not the nozzle exit.** The moment arm runs from the point the thrust line
pivots about. Using the exit plane overstates your control authority by the
length of the nozzle, which on a 24 mm motor is not a rounding error.

**Measure it loaded.** Propellant sits at the tail, so a loaded rocket has its
CG furthest aft, the shortest moment arm and the least authority. That is also
the instant the motor lights. Tune for the worst case, which is the one you get
at liftoff.

Measure it empty too. The CG moves forward 5 to 10 mm over the burn, and knowing
the range tells you how much margin the gains have.

---

## 3. Inertia, by bifilar pendulum

Two bits of string. This is the only measurement that needs a rig.

**Setup.** Lay the rocket horizontally. Run two strings of equal length `L`
straight up to a fixed bar, attached to the body at two points separated by `D`
along its length, **positioned so the CG is exactly midway between them.** Both
strings in the same vertical plane as the rocket's axis.

**Measure.** Twist a few degrees about the vertical axis through the CG, so the
nose swings horizontally, and let go. Time **20 full oscillations**, divide by
20 for the period `T`.

```
I = m g D² T² / (16 π² L)
```

Mass in kg, distances in metres, period in seconds.

**Worked example**, 250 g rocket:

| D | L | period | 20 swings |
|---|---|---|---|
| 0.25 m | 1.0 m | 2.55 s | 51 s |
| 0.25 m | 1.5 m | 3.12 s | 62 s |
| 0.30 m | 1.0 m | 2.12 s | 42 s |
| 0.30 m | 1.5 m | 2.60 s | 52 s |

Longer strings give a slower swing and a more forgiving stopwatch. 1.5 m is a
good compromise.

**Rules that matter more than they look:**

- **Small amplitude.** Under about 15 degrees. The formula assumes it.
- **Pure twist, no swaying.** If the rocket is also swinging side to side, stop
  and restart. A mixed mode gives a period that means nothing.
- **CG exactly midway between the strings.** This is the assumption the whole
  formula rests on, and the easiest thing to get wrong.
- **Loaded**, same as the CG.

**Sanity check.** A slender rod of the same mass and length has `I = m L² / 12`.
For 250 g and 550 mm that is 0.0063 kg m². A real rocket, with mass concentrated
at the ends, lands between **0.8 and 1.2 times** that. Outside that band, the
rig is wrong, not the rocket.

### Why not the single-pivot swing test

Hanging the rocket from one pivot and timing it as a pendulum also works:

```
I_cg = m g d T² / (4 π²) − m d²
```

but it ends in a subtraction of two similar numbers, so a 5 mm error in `d`
becomes roughly a 10% error in `I`. The bifilar has no such subtraction. Use the
swing test only as a cross-check.

---

## 4. Gains

```
python3 tvc_sim.py --sweep
```

The sweep searches kP, kI and kD and reports which combinations recover from a
misalignment without oscillating. **Take a result from the middle of the stable
region, not the edge**, because the edge is where a 10% inertia error puts you
outside it.

Then set `flightKp`, `flightKi`, `flightKd` in `src/main.cpp` and re-run bench
test 8, which is the only place the real servos, the real linkage and the real
estimator all meet.

**`LINK` in the simulator must match `setupLinkages()` in the firmware.** The sim
actuates through a full four-bar solve, so a stale linkage block means it is
tuning for a gimbal that does not exist. This has already happened once here,
and the gains it produced were silently wrong.

---

## 5. Boost and burnout thresholds

These are a function of mass and people forget to revisit them.

An accelerometer under thrust reads **T/m**, not (T−mg)/m. Gravity is already in
the reading: sitting on the pad it reports 1.00 g.

Worked for a C11 on a 495 g airframe:

| | thrust | reading |
|---|---|---|
| pad, still | 4.86 N | 1.00 g |
| average thrust | 10.9 N | 2.24 g |
| ignition peak | 21.7 N | 4.47 g |

**There is only 1.24 g between sitting on the pad and a burning motor.** The
same motor under a 200 g rocket gives 4.56 g. Thresholds chosen for a light
airframe are wrong on a heavy one, and the failure is quiet: `BURNOUT_G` of
1.5 g is 7.28 N of thrust, which a C11 decays through with real burn time left,
so TVC would switch off while the motor is still pushing.

**Bias toward detecting.** A false boost on the pad moves servos and starts a
log, and a reboot undoes it. A missed boost means no TVC for the whole flight.
Detect eagerly, exit late.

Also check `MAX_BURN_MS` against your motor. It is a backstop that forces the
coast transition, and if it is shorter than your burn it cuts TVC early.
