# Handset tests

Compiles the real `cyd/src/main.cpp` on your Mac against stubs for Arduino,
LovyanGFX, SD and Preferences, wires its radio to a model of the rocket, and
drives simulated touches through every button.

```
cd cyd/test
./build.sh && ./test_ui && ./test_touch
```

No board required. Takes about two seconds.

## What the rocket model gets right

The two behaviours that actually bite are modelled exactly:

- **`benchRun()` blocks.** While it runs the main loop never executes, so `A`,
  `D`, `K` and `O` are never seen, and the bench switch has no default case, so
  they are silently swallowed.
- **`checkSerialInput()` only runs outside bench mode** and knows none of the
  bench commands, so those are silently swallowed there.

Every command sent to the wrong loop is a no-op with no error message, on real
hardware and here. That is what these tests hunt for. `d` is kD inside bench
and DISARM outside it: same letter, two meanings, decided by which loop is
running.

## Bugs these tests found

1. **DISARM silently did nothing when telemetry was stale.** The handset only
   sent the mode switch when it believed the rocket was in the other mode, and
   with no recent telemetry it believed nothing. `sendFor` now sends the switch
   unconditionally.
2. **The rightmost two pixels of the PAD page fired the wrong button.** Three
   cells of 106 is 318 and the third is drawn 108 wide, so `x / 106` returned
   column 3 at x=318. The right edge of DISARM triggered CENTRE. Every grid now
   clamps its column and row.
3. **Handset and rocket trim diverged past the clamp.** The rocket limits trim
   to +/-25; the handset did not, so nudging past it walked the displayed value
   off while the rocket sat at the limit, and the screen lied about the trim you
   were about to fly.

## Two lessons about the tests themselves

- The **pixel sweep passed while the corner test failed**, because the sweep
  computed the expected cell with the same `x / 106` the code used. A test that
  reuses the implementation's arithmetic cannot find an error in that
  arithmetic. The corner test used the *drawn* width instead, and caught it.
- The **trim test passed before the mock clamped**. An unfaithful mock is worse
  than no mock: it produces a green check over a real bug.
