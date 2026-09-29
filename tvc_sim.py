#!/usr/bin/env python3
"""
TVC closed-loop simulator.

Runs the same estimator and PID logic that is in src/main.cpp against a
simulated gimballed rocket, so you can tune kP/kI/kD before a motor is
involved. A restrained static fire cannot do this: the vehicle cannot rotate,
so the loop never sees the plant respond.

The filter runs inside the loop, so filter lag and filter noise affect
stability exactly as they will in flight. That coupling is the main thing this
tells you that a spreadsheet cannot.

Usage:
    python3 tvc_sim.py                      # 5 deg initial tilt, recover
    python3 tvc_sim.py --gust               # mid-burn disturbance
    python3 tvc_sim.py --kp 1.6 --kd 0.25   # try gains
    python3 tvc_sim.py --estimator arrow    # feed the PID from the arrow filter
    python3 tvc_sim.py --sweep              # grid over kP and kD, no plots
    python3 tvc_sim.py --perfect            # bypass the filter, true angle in
    python3 tvc_sim.py --gate 0             # ungated, i.e. what happens without
                                            # the flight state machine

The accelerometer is gated by default, matching the STATE_PAD / STATE_BOOST /
STATE_COAST machine in src/main.cpp. During boost the estimator coasts on the
gyro alone, because the accelerometer is measuring thrust rather than gravity.
"""

import argparse
import math
import numpy as np

# ---------------------------------------------------------------------------
# vehicle constants: measure these on the actual rocket; tuning is only as
# good as these guesses
# ---------------------------------------------------------------------------
# Motor: Estes C11. 10.9 N average, 21.7 N peak, 8.8 Ns, 0.8 s burn, 35 g.
THRUST_N      = 10.9     # average motor thrust, newtons
MASS_KG       = 0.200    # liftoff mass including motor, kg
L_GIMBAL_M    = 0.20     # gimbal pivot to CG, metres (not nozzle exit to CG);
                         # measure loaded, since that's when the CG is
                         # furthest aft, the arm shortest, and that is also
                         # the instant the motor lights
INERTIA       = 0.008    # pitch/yaw moment of inertia about the CG, kg m^2.
                         # Bifilar pendulum: I = m g D^2 T^2 / (16 pi^2 L),
                         # hung horizontal on two strings length L separated
                         # by D, twisted gently and timed over 20 swings.
                         # Sanity check against a slender rod, m*len^2/12: a
                         # real rocket lands between 0.8 and 1.2 times that.
BURN_S        = 0.8      # motor burn time

# actuation: measure these five on the gimbal PITCH axis, fitted to measured
# nozzle angles (rms 0.62 -> 0.37 deg with a 116 mm lever); must match
# src/main.cpp's setupLinkages()
LINK = dict(hornR=18.4, rodL=18.4, armR=33.5,
            pivotX=19.5, pivotY=-19.9,
            armNeutral=103.0, servoTrim=100.0, branchUp=True, dir=+1.0)
# servoTrim +/- SERVO_SWING_DEG, matching linkageServoMin/Max in linkage.h.
SERVO_MIN, SERVO_MAX = 70.0, 130.0
SERVO_RATE    = 400.0    # slew limit, deg/s of SERVO travel (0.15 s per 60 deg)
SERVO_LAG_S   = 0.020    # first-order lag on top of the rate limit
TVC_SIGN      = +1       # which way a positive servo command pushes the
                         # nose; confirm by hand on the real rocket

# sensor, matching an MPU9250 at +/-2 g and +/-250 dps
ACC_NOISE_LSB = 300.0
GYR_NOISE_DPS = 1.0
GYR_BIAS_DPS  = 0.2      # residual bias left after calibrateGyro, not raw bias
LSB_PER_G     = 16384.0

# loop rates, matching the firmware
CTRL_HZ  = 200.0
PHYS_SUB = 5              # physics substeps per control step

# filter tuning, matching main.cpp
Q_ANGLE, Q_BIAS, R_MEASURE = 0.001, 0.003, 0.03
VEC_SCALE = (math.pi / 180.0) ** 2


# ---------------------------------------------------------------------------
# Linkage: a line-for-line port of src/linkage.h, so the sim actuates through
# the same geometry the firmware does.
# ---------------------------------------------------------------------------
def _horn_angle(k, arm_deg):
    a = math.radians(arm_deg)
    px = k["pivotX"] + k["armR"] * math.cos(a)
    py = k["pivotY"] + k["armR"] * math.sin(a)
    d = math.hypot(px, py)
    if d < 1e-9:
        return None
    x1 = (k["hornR"] ** 2 + d * d - k["rodL"] ** 2) / (2.0 * d)
    h2 = k["hornR"] ** 2 - x1 * x1
    if h2 < 0:
        return None
    y1 = (1.0 if k["branchUp"] else -1.0) * math.sqrt(h2)
    ang = math.atan2(py, px)
    ca, sa = math.cos(ang), math.sin(ang)
    return math.degrees(math.atan2(x1 * sa + y1 * ca, x1 * ca - y1 * sa))


LINK["hornNeutral"] = _horn_angle(LINK, LINK["armNeutral"])
if LINK["hornNeutral"] is None:
    raise SystemExit("Linkage does not close at neutral. Check the five dimensions.")


def servo_from_gimbal(k, defl_deg):
    """Gimbal deflection from centre -> servo command. None if unreachable."""
    h = _horn_angle(k, k["armNeutral"] + defl_deg)
    if h is None:
        return None
    return k["servoTrim"] + k["dir"] * (h - k["hornNeutral"])


def linkage_range(k, lo=SERVO_MIN, hi=SERVO_MAX, limit=45.0):
    out = [0.0, 0.0]
    for i, step in ((1, 0.05), (0, -0.05)):
        x = 0.0
        while abs(x) <= limit:
            x += step
            s = servo_from_gimbal(k, x)
            if s is None or s < lo or s > hi:
                break
            out[i] = x
    return out[0], out[1]


DEFL_MIN, DEFL_MAX = linkage_range(LINK)


def gimbal_from_servo(k, servo_deg):
    """Inverts the linkage numerically: finds the gimbal angle a given
    servo position produces."""
    lo, hi = DEFL_MIN, DEFL_MAX
    slo, shi = servo_from_gimbal(k, lo), servo_from_gimbal(k, hi)
    if slo is None or shi is None:
        return 0.0
    if slo > shi:
        lo, hi, slo, shi = hi, lo, shi, slo
    servo_deg = min(max(servo_deg, slo), shi)
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        sm = servo_from_gimbal(k, mid)
        if sm is None:
            return 0.5 * (lo + hi)
        if sm < servo_deg:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


# ---------------------------------------------------------------------------
# Estimators: line-for-line ports of src/main.cpp; change one and the other
# must follow, or this stops predicting the vehicle.
# ---------------------------------------------------------------------------
def wrap(a):
    while a > 180: a -= 360
    while a < -180: a += 360
    return a


class Kal1D:
    """kalmanFilter1D, one instance per tracked quantity."""
    def __init__(self):
        self.angle = 0.0
        self.bias = 0.0
        self.P = [[1.0, 0.0], [0.0, 1.0]]

    def predict_only(self, rate, dt, Qa, Qb):
        """Coasts on the gyro alone, used when the accelerometer isn't
        measuring gravity (most of the flight, under thrust)."""
        P = self.P
        self.angle = wrap(self.angle + dt * (rate - self.bias))
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Qa)
        P[0][1] -= dt * P[1][1]
        P[1][0] -= dt * P[1][1]
        P[1][1] += Qb * dt
        return self.angle

    def step(self, meas, rate, dt, Qa, Qb, R):
        P = self.P
        self.angle = wrap(self.angle + dt * (rate - self.bias))
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Qa)
        P[0][1] -= dt * P[1][1]
        P[1][0] -= dt * P[1][1]
        P[1][1] += Qb * dt
        y = wrap(meas - self.angle)
        S = P[0][0] + R
        K0, K1 = P[0][0] / S, P[1][0] / S
        self.angle = wrap(self.angle + K0 * y)
        self.bias += K1 * y
        p00, p01 = P[0][0], P[0][1]
        P[0][0] -= K0 * p00
        P[0][1] -= K0 * p01
        P[1][0] -= K1 * p00
        P[1][1] -= K1 * p01
        return self.angle


class AngleEstimator:
    """updateAngles()"""
    def __init__(self):
        self.kp = Kal1D()
        self.kr = Kal1D()

    def update(self, A, G, dt, gate=None):
        raw_roll = wrap(math.degrees(math.atan2(A[1], A[2])))
        raw_pitch = wrap(math.degrees(math.atan2(-A[0], math.hypot(A[1], A[2]))))
        rr = math.radians(self.kr.angle)
        pr = math.radians(self.kp.angle)
        sr, cr = math.sin(rr), math.cos(rr)
        tp = max(-5.0, min(5.0, math.tan(pr)))
        pitch_rate = G[1] * cr - G[2] * sr
        roll_rate = G[0] + tp * (G[1] * sr + G[2] * cr)
        mag = math.sqrt(A[0] ** 2 + A[1] ** 2 + A[2] ** 2) / LSB_PER_G
        if gate is not None and abs(mag - 1.0) > gate:
            p = self.kp.predict_only(pitch_rate, dt, Q_ANGLE, Q_BIAS)
            r = self.kr.predict_only(roll_rate, dt, Q_ANGLE, Q_BIAS)
        else:
            p = self.kp.step(raw_pitch, pitch_rate, dt, Q_ANGLE, Q_BIAS, R_MEASURE)
            r = self.kr.step(raw_roll, roll_rate, dt, Q_ANGLE, Q_BIAS, R_MEASURE)
        return p, r, raw_pitch, raw_roll


class ArrowEstimator:
    """updateGravityVector()"""
    def __init__(self):
        self.V = [0.0, 0.0, 1.0]
        self.k = [Kal1D() for _ in range(3)]
        for i, f in enumerate(self.k):
            f.angle = self.V[i]

    def update(self, A, G, dt, gate=None):
        w = [math.radians(G[0]), math.radians(G[1]), math.radians(G[2])]
        V = self.V
        rate = [-(w[1] * V[2] - w[2] * V[1]),
                -(w[2] * V[0] - w[0] * V[2]),
                -(w[0] * V[1] - w[1] * V[0])]
        mag = math.sqrt(A[0] ** 2 + A[1] ** 2 + A[2] ** 2)
        if mag < 2000.0:
            return self.angles()
        #accelerometer gate: under thrust it isn't measuring gravity, so trusting it corrupts the estimate
        if gate is not None and abs(mag / LSB_PER_G - 1.0) > gate:
            for i in range(3):
                V[i] = self.k[i].predict_only(rate[i], dt,
                                              Q_ANGLE * VEC_SCALE, Q_BIAS * VEC_SCALE)
        else:
            for i in range(3):
                V[i] = self.k[i].step(A[i] / mag, rate[i], dt,
                                      Q_ANGLE * VEC_SCALE, Q_BIAS * VEC_SCALE,
                                      R_MEASURE * VEC_SCALE)
        n = math.sqrt(V[0] ** 2 + V[1] ** 2 + V[2] ** 2)
        if n > 1e-6:
            for i in range(3):
                V[i] /= n
                self.k[i].angle = V[i]
        return self.angles()

    def angles(self):
        V = self.V
        return (math.degrees(math.atan2(-V[0], math.hypot(V[1], V[2]))),
                math.degrees(math.atan2(V[1], V[2])))


class PID:
    """pidControl(), including the servo clamp and anti-windup."""
    def __init__(self, kp, ki, kd):
        self.kP, self.kI, self.kD = kp, ki, kd
        self.i = [0.0, 0.0]
        self.last = [0.0, 0.0]

    def step(self, target, meas, dt):
        out = []
        for ax in (0, 1):
            e = target[ax] - meas[ax]
            ilim = max(abs(DEFL_MIN), abs(DEFL_MAX)) / max(self.kI, 1e-3)
            self.i[ax] = max(-ilim, min(ilim, self.i[ax] + e * dt))
            d = (e - self.last[ax]) / dt
            self.last[ax] = e
            c = e * self.kP + self.i[ax] * self.kI + d * self.kD
            out.append(max(DEFL_MIN, min(DEFL_MAX, c)))   # gimbal degrees
        return out


# ---------------------------------------------------------------------------
# Plant
# ---------------------------------------------------------------------------
def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def simulate(kp, ki, kd, misalign=1.0, rate0=0.0, gust=False, estimator="angle",
             perfect=False, gate=None, tmax=None, seed=0):
    """Starts vertical with the estimator correctly initialised, as
    calibrateOrientation() gives on the pad. Disturbances: a fixed thrust
    misalignment to trim out, and an optional tip-off rate off the rail."""
    rng = np.random.default_rng(seed)
    dt = 1.0 / CTRL_HZ
    hdt = dt / PHYS_SUB
    tmax = tmax if tmax is not None else BURN_S

    R = np.eye(3)
    w = np.array([0.0, math.radians(rate0), 0.0])   # tip-off, body rates rad/s
    servo = np.full(2, LINK["servoTrim"])   # actual servo position, deg
    gbias = rng.normal(0, GYR_BIAS_DPS, 3)

    est_a, est_v = AngleEstimator(), ArrowEstimator()
    pid = PID(kp, ki, kd)
    log = []

    n = int(tmax * CTRL_HZ)
    for i in range(n):
        t = i * dt
        thrust = THRUST_N if t < BURN_S else 0.0

        # --- sense ---
        g_body = R[2, :].copy()                       # R^T @ zhat
        a = g_body + np.array([0, 0, thrust / (MASS_KG * 9.81)])
        A = a * LSB_PER_G + rng.normal(0, ACC_NOISE_LSB, 3)
        G = np.degrees(w) + gbias + rng.normal(0, GYR_NOISE_DPS, 3)

        pitch_f, roll_f, pitch_raw, roll_raw = est_a.update(A, G, dt, gate)
        pitch_v, roll_v = est_v.update(A, G, dt, gate)

        true_pitch = math.degrees(math.atan2(-g_body[0], math.hypot(g_body[1], g_body[2])))
        true_roll = math.degrees(math.atan2(g_body[1], g_body[2]))

        if perfect:
            meas = (true_pitch, true_roll)
        elif estimator == "arrow":
            meas = (pitch_v, roll_v)
        else:
            meas = (pitch_f, roll_f)

        # --- control ---
        cmd = pid.step((0.0, 0.0), meas, dt)

        # --- actuate ---
        #cmd is gimbal degrees; converts to servo, applies lag/slew in servo degrees, converts back to the gimbal angle the plant sees
        gim = np.zeros(2)
        for ax in (0, 1):
            want_defl = float(np.clip(cmd[ax], DEFL_MIN, DEFL_MAX))
            target = servo_from_gimbal(LINK, want_defl)
            if target is None:
                target = LINK["servoTrim"]
            lag = dt / (SERVO_LAG_S + dt) if SERVO_LAG_S > 0 else 1.0
            want = servo[ax] + (target - servo[ax]) * lag
            step = max(-SERVO_RATE * dt, min(SERVO_RATE * dt, want - servo[ax]))
            servo[ax] += step
            gim[ax] = gimbal_from_servo(LINK, servo[ax])

        # --- plant ---
        for _ in range(PHYS_SUB):
            tau = np.zeros(3)
            if thrust > 0:
                #misalign is a fixed error in where the nozzle actually points
                tau[1] = TVC_SIGN * L_GIMBAL_M * thrust * math.sin(math.radians(gim[0] + misalign))
                tau[0] = TVC_SIGN * L_GIMBAL_M * thrust * math.sin(math.radians(gim[1]))
            if gust and BURN_S * 0.45 <= t < BURN_S * 0.45 + 0.05:
                tau[1] += 0.20                      # 50 ms kick, newton metres
            w = w + (tau / INERTIA) * hdt
            R = R @ (np.eye(3) + skew(w) * hdt)
            u, _, vt = np.linalg.svd(R)             # keep it a rotation
            R = u @ vt

        log.append((t, true_pitch, pitch_f, pitch_v, pitch_raw, gim[0], meas[0]))

    return np.array(log)


def summarize(log, tol=1.0):
    t, true_p = log[:, 0], log[:, 1]
    peak = np.max(np.abs(true_p))
    settled = None
    for i in range(len(t)):
        if np.all(np.abs(true_p[i:]) < tol):
            settled = t[i]
            break
    diverged = peak > 45 or not np.isfinite(peak)
    final = abs(true_p[-1])
    return peak, settled, diverged, final


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kp", type=float, default=1.0)
    ap.add_argument("--ki", type=float, default=0.1)
    ap.add_argument("--kd", type=float, default=0.05)
    ap.add_argument("--misalign", type=float, default=1.0,
                help="fixed thrust misalignment in degrees, the thing TVC exists to trim")
    ap.add_argument("--rate", type=float, default=0.0,
                help="tip-off rate off the rail, deg/s")
    ap.add_argument("--gust", action="store_true")
    ap.add_argument("--estimator", choices=["angle", "arrow"], default="angle")
    ap.add_argument("--perfect", action="store_true")
    ap.add_argument("--gate", type=float, default=0.2,
                    help="reject accelerometer when |a| differs from 1 g by more than this. "
                         "Defaults to 0.2 to match the flight state machine in main.cpp. "
                         "Pass --gate 0 for the ungated case.")
    ap.add_argument("--sweep", action="store_true")
    args = ap.parse_args()

    if args.sweep:
        print(f"thrust {THRUST_N} N, L {L_GIMBAL_M} m, I {INERTIA} kg m^2, "
              f"gimbal travel {DEFL_MIN:.1f} to {DEFL_MAX:.1f} deg\n")
        print("   kP     kD |   peak    final   verdict     (degrees of tilt; final is at burnout)")
        for kp in (0.5, 1.0, 1.5, 2.0, 3.0, 4.0):
            for kd in (0.0, 0.05, 0.1, 0.2, 0.4):
                lg = simulate(kp, args.ki, kd, args.misalign, args.rate,
                              estimator=args.estimator, gate=args.gate)
                peak, settle, div, final = summarize(lg)
                v = "DIVERGED" if div else "ok"
                print(f" {kp:5.2f}  {kd:5.2f} | {peak:6.2f}   {final:6.2f}   {v}")
        return

    log = simulate(args.kp, args.ki, args.kd, args.misalign, args.rate,
                   args.gust, args.estimator, args.perfect, args.gate)
    peak, settle, div, final = summarize(log)
    print(f"gains kP={args.kp} kI={args.ki} kD={args.kd}   "
          f"estimator={'true angle' if args.perfect else args.estimator}")
    print(f"peak tilt {peak:.2f} deg,  tilt at burnout {final:.2f} deg,  "
          f"{'DIVERGED' if div else 'stable'}")
    est_err = np.abs(log[:, 6] - log[:, 1])
    print(f"estimator error fed to the PID: mean {est_err.mean():.2f} deg, "
          f"max {est_err.max():.2f} deg")

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("(install matplotlib for plots)")
        return

    plt.style.use("dark_background")
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    ax1.plot(log[:, 0], log[:, 4], color="#5a6472", lw=0.8, label="raw accel angle")
    ax1.plot(log[:, 0], log[:, 1], color="#6bff8f", lw=2.0, label="true tilt")
    ax1.plot(log[:, 0], log[:, 2], color="#ffb454", lw=1.4, label="angle kalman")
    ax1.plot(log[:, 0], log[:, 3], color="#4ec9ff", lw=1.4, label="arrow kalman")
    ax1.axhline(0, color="#39414d", lw=0.8)
    ax1.set_ylabel("pitch, degrees")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.set_title(f"kP={args.kp} kI={args.ki} kD={args.kd}   peak {peak:.1f} deg   burnout {final:.1f} deg")

    ax2.plot(log[:, 0], log[:, 5], color="#ff6b6b", lw=1.4)
    ax2.axhline(DEFL_MAX, color="#5a6472", ls="--", lw=0.8)
    ax2.axhline(DEFL_MIN, color="#5a6472", ls="--", lw=0.8)
    ax2.set_ylabel("gimbal, degrees")
    ax2.set_xlabel("time, s")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
