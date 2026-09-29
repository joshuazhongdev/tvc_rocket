# The four-bar linkage in Desmos

A graphing-calculator model of `src/linkage.h`, verified identical to the
firmware solver to six decimal places on both axes across the full travel.

Useful for seeing why a fixed gimbal-per-servo ratio is wrong, and for watching
a mechanical lockup happen.

**Set Desmos to degrees first.** Wrench icon, top right, Degrees.

---

## The idea in one line

Four points. **S** the servo shaft, **A** the hole on the horn, **B** the hole
on the gimbal arm, **G** the gimbal pivot. Three of the four links have fixed
length, so once you choose where the arm points, **A** is forced: it must be
`hornR` from S and `rodL` from B at the same time. Two circles, one
intersection. That is the whole solve.

---

## Sliders

```
h = 18          horn: shaft to pushrod hole
L = 26          pushrod, hole to hole
m = 35          gimbal arm: pivot to pushrod hole
P = (26, 34)    gimbal pivot, from the servo shaft
n = 252         armNeutral, arm direction at neutral
t = 100         servoTrim
c = -1          branch. -1 is branchUp false, +1 is true
k = 1           dir
```

---

## The solve, one line at a time

Each is a function of `d`, the deflection you are asking for in gimbal degrees.

```
B(d) = P + m*(cos(n + d), sin(n + d))
D(d) = sqrt(B(d).x^2 + B(d).y^2)
X(d) = (h^2 + D(d)^2 - L^2) / (2*D(d))
Y(d) = c*sqrt(h^2 - X(d)^2)
g(d) = arctan(B(d).y, B(d).x)
A(d) = (X(d)*cos(g(d)) - Y(d)*sin(g(d)), X(d)*sin(g(d)) + Y(d)*cos(g(d)))
H(d) = arctan(A(d).y, A(d).x)
S(d) = t + k*(H(d) - H(0))
```

1. **`B(d)`** puts the arm's pushrod hole where you asked.
2. **`D(d)`** is how far that is from the servo shaft.
3. **`X(d)`** is the two-circle formula: distance along the S-to-B line where
   the circles cross.
4. **`Y(d)`** is the perpendicular offset. **This is where lockup lives.** If
   `h² − X²` goes negative the circles do not meet and there is no solution.
   Desmos simply stops drawing, which is exactly the behaviour you want to see.
5. **`g(d)`** is the tilt of the S-to-B line.
6. **`A(d)`** rotates `(X, Y)` back into real coordinates.
7. **`H(d)`** reads off the horn angle.
8. **`S(d)`** is what to write to the servo, referenced to `H(0)` so it is a
   move from neutral rather than an absolute angle.

`arctan(y, x)` with two arguments is Desmos's `atan2`.

---

## Plot 1: the transfer function

```
y = S(x)
```

Add `y = 70` and `y = 130` for the servo window.

It is **not** a straight line. The slope is servo degrees per gimbal degree and
it changes across the travel. That curvature is the reason `linkage.h` exists
instead of a single multiply. Where the curve stops dead is the mechanical
lockup.

## Plot 2: the gearing

```
R(x) = 1 / (d/dx S(x))
```

Gimbal degrees per horn degree, the local gearing. If it dives toward zero
anywhere, the linkage is approaching a toggle and has no authority there.

## Plot 3: the mechanism

Add a slider `u = 0`, range −20 to 20:

```
polygon((0,0), A(u))     the horn
polygon(A(u), B(u))      the pushrod
polygon(P, B(u))         the gimbal arm
polygon((0,0), P)        the ground link, fixed
```

Add the circles to see the construction:

```
x^2 + y^2 = h^2
(x - B(u).x)^2 + (y - B(u).y)^2 = L^2
```

`A(u)` is one of the two places those circles cross. `c` picks which one. Flip
`c` and watch the horn jump to the mirror position, which is the wrong branch
that `branchUp` exists to select between.

---

## Things worth trying

- **Drag `u` past the limit.** Watch the circles separate. That is a lockup,
  and it is why every solver function returns a bool.
- **Change `n` by 10 degrees.** The whole transfer curve shifts and one side of
  the travel collapses. `armNeutral` is the parameter no caliper can reach, and
  this shows in one drag why getting it wrong matters.
- **Change `t`.** The curve slides vertically and the lockup moves with it in
  servo terms. A toggle is at a fixed **deflection**, not a fixed servo angle.
  This is the single most misleading thing about four-bar linkages.
- **Shorten `L` by 2 mm.** See how much the travel changes. That tells you how
  precise your caliper work needed to be, and it is usually more precise than
  you managed.
