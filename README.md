# TVC Rocket Avionics

Thrust vector control for a model rocket, built end to end on an ESP32-S3
with an MPU9250 and MG90S servos (SG90-compatible), using Kalman filtering
and PID.

A wireless touchscreen handset runs the bench suite, arms, trims and
calibrates the rocket with no cable attached.

Static fire testing is done. No actual flight yet.

## Why this exists

TVC is usually gatekept behind expensive flight computers or hand-waved
geometry. This runs end to end on a $10 ESP32-S3 and off-the-shelf hobby
servos, so it's something any hobbyist can actually build and use.

## Hardware

| Part | Detail |
| --- | --- |
| Flight computer | ESP32-S3 DevKitC-1 |
| IMU | MPU9250, I2C `0x68`, 400 kHz |
| Actuation | 2x MG90S servos (SG90-compatible), 50 Hz, 500-2400 us |
| Handset | ESP32-2432S028R (Cheap Yellow Display) |

## Layout

```
src/            rocket firmware
  main.cpp        estimator, flight state machine, PID, arming
  linkage.h       four-bar linkage solver
  bench.cpp       bench test menu
  flightlog.*     flash-based flight recorder
  console.*       ESP-NOW link
  linkproto.h     wire format, duplicated in cyd/src
cyd/            handset firmware, separate PlatformIO project
  test/           host test harness, no hardware needed
tvc_sim.py      offline simulator, PID gain sweep
readlog.py      pull flight logs off the vehicle
imu_3d_viz.py   live 3D attitude visualizer
docs/           measurement procedures and design notes
```

## Quick start

```sh
pio run -t upload -t monitor            # rocket
cd cyd && pio run -t upload             # handset, separate project
cd cyd/test && ./build.sh && ./test_ui && ./test_touch   # handset tests, no hardware
```

## Before it will fly

1. Linkage geometry, fitted to measured angles. [docs/linkage-measurement.md](docs/linkage-measurement.md)
2. IMU-to-gimbal rotation, from bench test 1. [docs/axis-frames.md](docs/axis-frames.md)
3. Mass, CG, moment of inertia, then PID gains. [docs/mass-cg-inertia.md](docs/mass-cg-inertia.md)
4. Boost/burnout thresholds, sized to your mass.
5. Static fire. Done. [docs/static-fire.md](docs/static-fire.md)

## Documentation

| | |
| --- | --- |
| [docs/linkage-measurement.md](docs/linkage-measurement.md) | Fitting the linkage geometry to measured angles |
| [docs/axis-frames.md](docs/axis-frames.md) | The IMU-to-gimbal rotation and how to correct for it |
| [docs/mass-cg-inertia.md](docs/mass-cg-inertia.md) | Bifilar pendulum, CG, and sizing gains to your mass |
| [docs/four-bar-in-desmos.md](docs/four-bar-in-desmos.md) | The linkage solver in a graphing calculator |
| [docs/handset.md](docs/handset.md) | How the wireless handset works |
| [docs/menu.md](docs/menu.md) | What every bench test does |
| [docs/static-fire.md](docs/static-fire.md) | Static fire checklist |

## Safety

The PID gains in this repo are tuned for this vehicle: its mass, its motor,
its linkage geometry. They will not transfer to a different rocket or motor
without retuning. Follow [docs/mass-cg-inertia.md](docs/mass-cg-inertia.md) to
fit your own before flying anything else.

Closed-loop control is enabled (`tvcEnabled = true`) after confirming the
control sign on bench test 8. If you build one of these yourself, leave it
`false` until you've run that same check on your own hardware.

- Keep fins on the airframe. There's no aerodynamic backup if the loop is wrong.
- Fly only where model rocket launches are permitted, following your national
  safety code.

## Licence

This project's own code is MIT, see [LICENSE](LICENSE).

Third-party libraries keep their own licences:

| Library | Licence |
| --- | --- |
| ESP32 Arduino core | LGPL 2.1 |
| ESP32Servo | LGPL 2.1 or later |
| LovyanGFX | FreeBSD |
