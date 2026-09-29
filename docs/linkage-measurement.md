# Measuring and fitting the gimbal linkage

Calipers are not good enough. That is the short version, and it is the most
useful thing in this repo.

---

## Conventions

**90 always means centre.** `SERVO_CENTER` is 90 and never changes. The angle
that makes a nozzle square is `SERVO_CENTER + servoOffset`, and `servoOffset` is
the only trim you tune. `linkageInit()` derives `servoTrim` from it, so the two
cannot drift apart.

**Each axis lives in its own servo's frame.** Origin at that servo's output
shaft. There is no shared global origin. `armNeutral` is the direction from the
gimbal pivot to the arm's pushrod hole at neutral, counterclockwise from that
frame's +x axis.

**The servo window follows the trim**, `±SERVO_SWING_DEG` around each axis's own
neutral, rather than a pair of global constants. A fixed 70/130 window is
centred on 90 and goes lopsided the moment the trim moves to 100.

---

## The measurement

Calipers give you five of the six parameters to about half a millimetre, and
give you `armNeutral` not at all, because it is an angle between two holes you
cannot put a caliper across. Fit it instead:

1. Tape a **long lever** to the nozzle. Lever length is the whole game: a 12 mm
   stick carries about 0.95 degrees of reading noise, a 116 mm pencil about
   0.25.
2. Command the servo to **11 raw positions** with `S<n>,<n>`, which bypasses the
   linkage model entirely. That is the point: every other command asks the model
   where to put the servo, which makes them useless for checking the model.
3. Measure the **sideways travel of the lever tip** at each position and convert
   to an angle.
4. Least squares over all six geometry parameters, with the caliper readings as
   priors and a 1 degree measurement sigma.

### Results on this vehicle

| | yaw | pitch |
|---|---|---|
| lever | 12 mm stick | 116 mm pencil |
| reading noise | ~0.95 deg | ~0.25 deg |
| rms residual | 0.89 → 0.51 deg | 0.62 → 0.37 deg |
| points used | 11 | 10 of 11 |

Pitch dropped one point as an obvious misread: its step from the neighbour was
0.10 cm where every other step in the sweep was 0.25 to 0.45.

**The caliper model's error was one-sided on both axes.** The positive half
matched within a few tenths while the negative half was out by nearly two
degrees. With the pre-fit values, commanding −8 and +8 on yaw actually delivered
**−6.9 and +7.7**, a 13% asymmetry, and it was visible by eye before any fit was
run.

### Fitted values

| field | pitch | yaw |
|---|---|---|
| hornR | 18.4 | 18.1 |
| rodL | 18.4 | 26.2 |
| armR | 33.5 | 35.0 |
| pivotX | 19.5 | 25.6 |
| pivotY | −19.9 | 33.8 |
| armNeutral | 103.0 | 252.0 |
| servoOffset | +10.0 | +10.0 |
| branchUp | true | false |
| dir | +1 | +1 |

Linkage ratio across the working range: 0.53 to 0.56 on pitch, about 0.60 on
yaw. Flat on both, no near-toggle region in use.

---

## How to know whether to believe a fit

A fit that lowers rms is not automatically right. It can buy a better curve by
distorting a real dimension. Two things settled these:

- **It reproduced something already seen by eye.** The asymmetry was noticed
  before the fit ran, and the two directions looked equal afterwards. Two
  independent observations agreeing is what makes a fit trustworthy.
- **It made a testable claim.** The pitch fit moved `pivotX` from the measured
  22.0 to 19.5, a 2.5 mm disagreement far outside caliper error. A re-measure
  returned 19. The fit was right and the original reading was wrong.

The remaining 0.5 mm is worth at most 0.2 degrees of nozzle angle, at the
extreme of travel, falling to zero at centre. Below the reading noise, so it
does not matter.

---

## Two traps

### The toggle position moves with the trim

`armNeutral` on the yaw axis took three attempts. The first two were inferred
from an apparent travel limit that only looked like a mechanical lockup because
it was computed against a trim of 90 rather than the real 100. **A toggle sits
at a fixed deflection, not a fixed servo angle.** Change the trim and it moves.

### Inverting the solver is harder than it looks

To ask "what nozzle angle does servo angle X give" you need the inverse of
`linkageServoAngle`, and the obvious bracket is wrong twice over:

- Bracketing inward from ±25 degrees lands on the **mirror branch** past a
  toggle, which is a valid pair of circle intersections and a physically
  impossible pose.
- The servo angle is **not monotonic** over that range, so both endpoints can
  share a sign and a root finder reports no root, or the wrong one.

Correct method: walk outward from 0 in small steps and take the **first** sign
change. A fit built on the naive bracket returned +23.97 degrees for servo 85
through 97, which is nonsense, and looked plausible enough to nearly ship.

---

## Verifying on the bench

- **Envelope walk** (`W<radius>`) at 12 bearings, twice, at your working radius
  and beyond it. Any bearing that reports unreachable is a real limit.
- **`G0`** should put both servos exactly on their trim.
- **`G0,-8` and `G0,8`** should look equal by eye. This is the check that
  catches a one-sided geometry error, and it is free.
- **Servo travel symmetric about the trim.** Unequal travel between axes is
  fine if they are geared differently; unequal travel within one axis is not.
