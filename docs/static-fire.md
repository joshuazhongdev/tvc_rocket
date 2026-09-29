# Static fire checklist

---

## First, what a static fire cannot tell you

**A rigidly clamped rocket's accelerometer reads 1.00 g the whole burn, no
matter how hard the motor pushes.**

An accelerometer measures specific force, which excludes gravity. Clamped, the
vehicle does not accelerate, so thrust and the clamp force cancel:

```
T + N − mg = 0        →     N = mg − T
f = (T + N)/m = (T + mg − T)/m = g
```

The same motor in free flight on a 495 g airframe reads **2.24 g average** and
**4.47 g at the ignition peak.**

So a static fire on a rigid stand **does not validate `BOOST_G` or
`BURNOUT_G`**, which is the reason most people think they are doing one. If you
want thrust numbers you need a load cell; if you want the accelerometer trace
you need the vehicle free to move.

What it **does** test, and these are worth a burn:

- The motor lights from your igniter and your controller.
- Motor retention holds, and the casing stays put.
- The gimbal does not foul under vibration with a motor installed.
- Wiring and electronics survive the heat and the shake.
- The flight logger writes a real file under real conditions.
- **Whether vibration false-triggers boost detection.** The accelerometer will
  not see 2.24 g of thrust, but it will see vibration, and black powder motors
  vibrate hard. If `BOOST_G 1.8` with a 40 ms confirm latches on shake alone,
  you want to find that out bolted to a stand.

---

## Firmware, before the burn

- [ ] **`tvcEnabled = false`.** Clamped, the estimator sees no tilt, so the
      controller has nothing to correct and TVC adds no information. It only
      adds a way for vibration to slam the servos. Turn it on for a hop, not
      for a clamped burn.
- [ ] Flight logger proven end to end on the bench: fake a flight, wait the
      8 s flush, reboot, `F` and `L` and read it back.
- [ ] `BOOST_G`, `BURNOUT_G` and `MAX_BURN_MS` sized for **this** mass and
      **this** motor. See [mass-cg-inertia.md](mass-cg-inertia.md).
- [ ] Bench tests 1, 2 and 8 run and recorded, so you have a before picture to
      compare the after against.
- [ ] Handset recording to SD, from behind cover, so you still have data if the
      onboard flash write fails.
- [ ] Fresh battery. A brownout mid-burn loses the one run you set up for.

## Mechanical, before the burn

- [ ] **Envelope walk with the motor installed.** Motor mass and the casing can
      foul a gimbal that swung freely empty.
- [ ] Servo wires, battery leads and the IMU harness clear of the exhaust path
      and not under tension at full gimbal deflection.
- [ ] Motor retention rated for the job, and checked by pulling on it.
- [ ] **Plan for the ejection charge.** A delay motor fires it a few seconds
      after burnout, out the forward end. Either point that somewhere harmless
      or expect wadding across your workspace. It is also a free test of
      recovery deployment if you set it up to be one.
- [ ] Stand rigid and heavier than you think it needs to be, on ground that
      will not tip.
- [ ] Nothing flammable within the exhaust cone. Black powder throws sparks.

## Site and safety

- [ ] Fire it outdoors, on bare ground or a hard surface, with no dry grass.
- [ ] Water and a fire extinguisher within reach.
- [ ] **Electrical ignition from a distance**, with a launch controller that
      has a removable safety key and continuity check. Never a fuse or a
      handheld flame.
- [ ] Stand well back and behind something solid. Treat it as a launch.
- [ ] Nobody downrange of the nozzle or the forward end.
- [ ] A static fire is a rocket motor operation and the same rules apply as for
      a launch. Check what is permitted where you are, and follow your national
      body's safety code.

## Afterwards

- [ ] Wait. Let it cool before touching anything.
- [ ] Pull the log and check: did the state machine latch? When? Did vibration
      trigger boost early?
- [ ] Re-run the envelope walk. A gimbal that moved freely before and binds now
      has taken heat or shock somewhere.
- [ ] Re-run test 8. The mixing matrix should be unchanged; if it is not,
      something moved.
- [ ] Photograph the nozzle and the mount before cleaning them.

---

## An alternative worth considering

On a 495 g airframe a C11 reaches about **9 metres** and lands in a few seconds.
That flight carries barely more energy than a clamped burn of the same motor,
and unlike the static fire it exercises the things you actually care about: the
accelerometer sees real thrust, boost and burnout latch on real data, and the
gimbal has something to correct.

If the site and the rules allow a low hop, it is a better use of a motor than a
clamped burn. Do the mechanical checks above either way.
