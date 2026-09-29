#!/usr/bin/env python3
"""
Four-bar linkage transfer function for a TVC gimbal.

Answers whether one degree of servo is one degree of gimbal, which it never
is, by solving the actual geometry instead of assuming a ratio.

THE GEOMETRY

    S = servo pivot           A = far end of the servo horn
    G = gimbal pivot          B = far end of the gimbal arm
    A to B = the pushrod, a rigid link of fixed length

    horn radius  r_h = S to A
    arm radius   r_g = G to B
    pivot base   d   = S to G
    pushrod      L   = A to B

Rotating the horn moves A around a circle of radius r_h about S. B must stay
on a circle of radius r_g about G, and exactly L from A, so B is wherever
those two circles cross.

    circle 1: centre A, radius L
    circle 2: centre G, radius r_g

Two crossings exist. The linkage lives on one of them, the branch it was
assembled in, so the crossing nearest the previous step is picked and the
solution walks along continuously. If the circles stop crossing, the linkage
has hit a physical lockup and cannot reach that horn angle; the script
reports this rather than returning nonsense.

MEASURING THE FOUR NUMBERS

    r_h  centre of the servo output shaft to the pushrod hole in the horn
    r_g  gimbal pivot axis to the pushrod hole in the gimbal arm
    d    servo shaft centre to gimbal pivot axis
    L    pushrod hole to pushrod hole, with the gimbal centred

Calipers on all four. Getting d and L right matters more than expected,
since they set how fast the ratio drifts away from linear.

Usage:
    python3 linkage.py                       # uses the values below
    python3 linkage.py --rh 8 --rg 12 --d 40 --L 38
"""

import argparse
import math
import numpy as np

# --- measured on the gimbal, millimetres ------------------------------------
R_HORN = 8.0     # servo horn, shaft centre to pushrod hole
R_ARM = 12.0     # gimbal arm, pivot axis to pushrod hole
D_BASE = 40.0    # servo shaft to gimbal pivot
L_ROD = 40.0     # pushrod, hole to hole at neutral

SWEEP_DEG = 30.0  # how far either side of neutral to plot

# palette: validated categorical slots 1 and 2
C_TRUE, C_LIN = "#2a78d6", "#eb6834"
C_INK, C_MUTED, C_GRID = "#0b0b0b", "#52514e", "#dcdcd6"


def circle_intersections(c1, r1, c2, r2):
    """Where two circles cross. Returns [] if they do not."""
    (x1, y1), (x2, y2) = c1, c2
    dx, dy = x2 - x1, y2 - y1
    dist = math.hypot(dx, dy)
    if dist > r1 + r2 or dist < abs(r1 - r2) or dist == 0:
        return []
    a = (r1 * r1 - r2 * r2 + dist * dist) / (2 * dist)
    h2 = r1 * r1 - a * a
    if h2 < 0:
        return []
    h = math.sqrt(h2)
    xm, ym = x1 + a * dx / dist, y1 + a * dy / dist
    return [(xm + h * dy / dist, ym - h * dx / dist),
            (xm - h * dy / dist, ym + h * dx / dist)]


def solve(r_h, r_g, d, L, sweep=SWEEP_DEG, step=0.25):
    """Sweep the horn and return (horn_deg, gimbal_deg). Neutral is defined as
    the assembled pose, so both curves pass through the origin."""
    S, G = (0.0, 0.0), (d, 0.0)

    # neutral pose: horn perpendicular to the base line -- how these are
    # normally assembled, so the linkage sits in its most linear region
    horn0 = math.pi / 2
    A0 = (S[0] + r_h * math.cos(horn0), S[1] + r_h * math.sin(horn0))
    pts = circle_intersections(A0, L, G, r_g)
    if not pts:
        raise SystemExit(
            f"Linkage does not close at neutral with r_h={r_h}, r_g={r_g}, "
            f"d={d}, L={L}.\nThe pushrod cannot reach. Check the four numbers: "
            f"L must be within {abs(math.hypot(A0[0]-G[0], A0[1]-G[1]) - r_g):.1f} "
            f"and {math.hypot(A0[0]-G[0], A0[1]-G[1]) + r_g:.1f} mm.")
    B0 = max(pts, key=lambda p: p[1])          # the assembled branch
    arm0 = math.atan2(B0[1] - G[1], B0[0] - G[0])

    horn = np.arange(-sweep, sweep + step, step)
    gim, prev = [], B0
    for hd in horn:
        ang = horn0 + math.radians(hd)
        A = (S[0] + r_h * math.cos(ang), S[1] + r_h * math.sin(ang))
        pts = circle_intersections(A, L, G, r_g)
        if not pts:
            gim.append(np.nan)
            continue
        B = min(pts, key=lambda p: (p[0] - prev[0]) ** 2 + (p[1] - prev[1]) ** 2)
        prev = B
        gim.append(math.degrees(math.atan2(B[1] - G[1], B[0] - G[0]) - arm0))
    return horn, np.array(gim)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rh", type=float, default=R_HORN)
    ap.add_argument("--rg", type=float, default=R_ARM)
    ap.add_argument("--d", type=float, default=D_BASE)
    ap.add_argument("--L", type=float, default=L_ROD)
    ap.add_argument("--sweep", type=float, default=SWEEP_DEG)
    ap.add_argument("--csv", action="store_true", help="print the table instead of plotting")
    a = ap.parse_args()

    horn, gim = solve(a.rh, a.rg, a.d, a.L, a.sweep)
    ok = ~np.isnan(gim)

    # local ratio, gimbal degrees per horn degree
    ratio = np.gradient(gim, horn)
    mid = len(horn) // 2
    r0 = ratio[mid]
    small_angle = a.rh / a.rg          # the textbook approximation
    linear = horn * r0

    if a.csv:
        print("horn_deg,gimbal_deg,local_ratio")
        for h, g, r in zip(horn, gim, ratio):
            print(f"{h:.2f},{g:.4f},{r:.4f}")
        return

    dev = np.nanmax(np.abs(gim - linear))
    print(f"geometry: r_horn {a.rh} mm, r_arm {a.rg} mm, base {a.d} mm, rod {a.L} mm")
    print(f"ratio at neutral          {r0:.3f} gimbal deg per horn deg")
    print(f"small-angle r_h/r_g       {small_angle:.3f}   "
          f"({100*(r0-small_angle)/small_angle:+.1f}% off)")
    print(f"ratio range over +/-{a.sweep:.0f} deg  "
          f"{np.nanmin(ratio):.3f} to {np.nanmax(ratio):.3f}")
    print(f"worst deviation from straight line  {dev:.2f} deg")
    if np.isnan(gim).any():
        bad = horn[np.isnan(gim)]
        print(f"LOCKUP: linkage cannot reach horn angles "
              f"{bad.min():.1f} to {bad.max():.1f} deg")
    print(f"gimbal travel for a servo command of +/-30 deg: "
          f"{np.nanmin(gim):.1f} to {np.nanmax(gim):.1f} deg")

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("(pip3 install matplotlib for the plot, or use --csv)")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.5, 7.2), sharex=True,
                                   gridspec_kw={"height_ratios": [2, 1]})
    fig.patch.set_facecolor("#fcfcfb")

    for ax in (ax1, ax2):
        ax.set_facecolor("#fcfcfb")
        ax.grid(True, color=C_GRID, lw=0.8)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(C_GRID)
        ax.tick_params(colors=C_MUTED, labelsize=9)

    ax1.plot(horn, linear, lw=2, color=C_LIN, ls=(0, (5, 3)),
             label=f"straight line at {r0:.2f} per degree")
    ax1.plot(horn, gim, lw=2, color=C_TRUE, label="actual linkage")
    ax1.set_ylabel("gimbal angle, degrees", color=C_MUTED, fontsize=10)
    ax1.set_title("Gimbal angle vs servo horn angle", color=C_INK,
                  fontsize=12, loc="left", pad=12)
    leg = ax1.legend(frameon=False, fontsize=9, loc="upper left")
    for t in leg.get_texts():
        t.set_color(C_MUTED)
    ax1.annotate(f"worst departure from straight: {dev:.2f}°",
                 xy=(0.99, 0.06), xycoords="axes fraction",
                 fontsize=9, color=C_MUTED, ha="right")

    ax2.plot(horn, ratio, lw=2, color=C_TRUE)
    ax2.axhline(r0, lw=2, color=C_LIN, ls=(0, (5, 3)))
    ax2.set_ylabel("local ratio", color=C_MUTED, fontsize=10)
    ax2.set_xlabel("servo horn angle from neutral, degrees", color=C_MUTED, fontsize=10)
    ax2.set_title("Gimbal degrees per horn degree", color=C_INK,
                  fontsize=11, loc="left", pad=10)
    ax2.annotate(f"{np.nanmin(ratio):.2f} to {np.nanmax(ratio):.2f}",
                 xy=(0.99, 0.06), xycoords="axes fraction",
                 fontsize=9, color=C_MUTED, ha="right")

    plt.tight_layout()
    plt.savefig("linkage.png", dpi=150, facecolor="#fcfcfb")
    print("\nsaved linkage.png")
    plt.show()


if __name__ == "__main__":
    main()
