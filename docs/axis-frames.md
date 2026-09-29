# Your IMU is not aligned with your gimbal

This is the bug that is hardest to find by staring at code, because nothing is
wrong with the code.

---

## The problem

The estimator reports pitch and yaw about the **IMU's** axes. The servos deflect
the nozzle about the **gimbal's** axes. If the sensor board is rotated about the
nose relative to the gimbal, the estimator stays perfectly correct and every
command lands on the wrong servo, partly or entirely.

On this vehicle the two frames are **47 degrees apart, with the handedness
flipped.** Nobody designed that. It is where the board fitted.

Symptoms if you have this and do not know it:

- Tilting in pure pitch makes both servos move.
- The closed loop half works: it opposes some tilts and amplifies others.
- Cross-axis coupling that no amount of PID tuning removes, because it is not a
  tuning problem.

---

## Measuring it

Bench test 1 does this. Tilt the vehicle by hand in each gimbal axis in turn
while the test records the attitude the estimator reports, then it solves for
the rotation between the two frames and prints the matrix.

Three independent runs on this vehicle:

| run | pitch bearing | separation | squared up |
|---|---|---|---|
| 1 | −53.8 | −78.4 | −48.0 |
| 2 | −50.6 | −84.7 | −47.9 |
| 3 | −49.7 | −82.1 | −45.7 |

Mean **−47.2 degrees, sd 1.3.** Hand tilts varied by 30% between runs and the
answer moved 3 degrees, so the test is insensitive to the thing the operator
cannot control, which is what you want from a hand-driven measurement.

The ideal separation between the two axes is 90 degrees. Measuring 78 to 85 is
the honest residual of tilting a rocket by hand.

```cpp
float mixPP = +0.6697f, mixPY = -0.7426f;
float mixYP = -0.7426f, mixYY = -0.6697f;
```

The determinant is −1, not +1: this is a rotation **and** a reflection. That is
the handedness flip, and it is why simply rotating coordinates would not have
been enough.

---

## Applying it

Rotate **once**, where the attitude is produced, not in the controller:

```cpp
gimbalPitch = mixPP * kalAnglePitch + mixPY * kalAngleYaw;
gimbalYaw   = mixYP * kalAnglePitch + mixYY * kalAngleYaw;
```

Everything downstream then reports the axis a servo actually moves: telemetry,
the flight log, the live CSV and the closed-loop test. `pidControl` works in one
frame and does not need to know two frames exist.

**The Kalman state stays in the IMU frame.** Its measurements arrive in that
frame, and rotating the state without rotating the measurements would quietly
destroy the filter. Bench test 1 also stays in the IMU frame, because measuring
the relationship between the frames is the one thing that cannot be done from
inside the rotated one.

This is a small-angle treatment. Pitch and yaw are rotations about two
perpendicular body axes, and rotating that pair about the nose is exact only
while the angles are small. Flight angles are small. Bench angles are not, so do
not read too much into gimbal-frame numbers while waving the vehicle at 60
degrees.

---

## This replaces `dir`

A reversed axis shows up as a negative diagonal term in the matrix. Leave
`Linkage::dir` at +1 and let the matrix carry the sign, so there is one place to
look rather than two that can disagree.

---

## Confirm it before you fly

Bench test 8 closes the loop and scores the correlation between tilt and
command. Both axes must come out **negative**, meaning the gimbal opposed the
tilt. A positive correlation is positive feedback: a device that drives the
rocket over as hard as it can. The test says so explicitly and you should
believe it.
