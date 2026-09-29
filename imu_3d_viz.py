#!/usr/bin/env python3
"""
3D IMU visualizer for the TVC sketch.

Draws up to three rockets at once so you can see what each estimator does:

    grey ghost  = raw accelerometer angles, unfiltered
    orange      = Kalman filter on two angles (pitch and roll)
    blue        = Kalman filter on the gravity arrow

Tip it slowly and the orange and blue rockets should agree. Tip it past 90
degrees nose-over and the orange one folds back on itself while the blue one
keeps going, which is the whole point of the arrow version.

Usage:
    python3 imu_3d_viz.py                       # uses the port below
    python3 imu_3d_viz.py /dev/cu.usbmodem1101
    python3 imu_3d_viz.py --list                # show available ports

Keys (click the plot window first):
    q  quit
    g  toggle the raw ghost
    a  toggle the angle (orange) rocket
    v  toggle the arrow (blue) rocket
    s  toggle display smoothing
"""

import sys
import time
import threading
from collections import deque

import numpy as np

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Missing pyserial.  pip3 install pyserial")
    sys.exit(1)

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d.art3d import Line3DCollection
except ImportError:
    print("Missing matplotlib.  pip3 install matplotlib")
    sys.exit(1)


DEFAULT_PORT = "/dev/cu.usbmodem1101"
BAUD = 115200
SMOOTH_ALPHA = 0.35     # 1.0 = no smoothing, lower = smoother but laggier

running = True
show_ghost = True
show_angle = True
show_arrow = True
smoothing = True

state = {
    "raw": (0.0, 0.0),
    "kal": (0.0, 0.0),
    "arrow": (0.0, 0.0, 1.0),
    "have_raw": False,
    "have_arrow": False,
    "state": None,
    "cmd": (0.0, 0.0),
    "hz": 0.0,
    "status": "waiting for board...",
}
lock = threading.Lock()


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
    for p in ports:
        print(f"  {p.device}   {p.description}")


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if DEFAULT_PORT in ports:
        return DEFAULT_PORT
    for d in ports:
        low = d.lower()
        if any(k in low for k in ("wchusbserial", "usbserial", "usbmodem", "ttyusb", "ttyacm")):
            return d
    return ports[0] if ports else None


def open_serial(port_name):
    """Opens without DTR/RTS: on a CH340 those lines are wired to EN and
    BOOT on the ESP32, so a normal open would reset or hold the board in
    reset. Matches monitor_dtr=0/monitor_rts=0 in platformio.ini."""
    ser = serial.Serial()
    ser.port = port_name
    ser.baudrate = BAUD
    ser.timeout = 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def parse_line(line):
    """2 values = one angle pair, 4 = raw + kalman, 7 = raw + kalman + arrow."""
    if not line or line[0] == "#":
        return None
    parts = line.split(",")
    if len(parts) not in (2, 4, 7, 10):
        return None
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        return None
    if any(abs(v) > 720 or v != v for v in vals):   # v != v catches nan
        return None
    return vals


def reader_thread(port_name):
    global running
    try:
        ser = open_serial(port_name)
    except Exception as e:
        print(f"Could not open {port_name}: {e}")
        running = False
        return

    print(f"Connected to {port_name} at {BAUD} baud.")
    print("If the board was just plugged in, give it about 4 s to calibrate.")

    time.sleep(0.3)
    ser.reset_input_buffer()

    got_data = False
    last_poke = 0.0
    stamps = deque(maxlen=50)

    while running:
        now = time.time()

        # asks for the combined stream until data arrives; covers attaching
        # late, since the sketch also defaults to it on its own
        if not got_data and now - last_poke > 1.0:
            try:
                ser.write(b"b\n")
            except Exception:
                pass
            last_poke = now

        # drops the backlog rather than rendering stale data if reading falls behind
        try:
            if ser.in_waiting > 4096:
                ser.reset_input_buffer()
        except Exception:
            pass

        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception:
            continue

        if not line:
            continue

        if line.startswith("#"):
            with lock:
                state["status"] = line[1:].strip()
            print(line)
            continue

        vals = parse_line(line)
        if vals is None:
            continue

        got_data = True
        stamps.append(now)
        hz = 0.0
        if len(stamps) > 2:
            span = stamps[-1] - stamps[0]
            if span > 0:
                hz = (len(stamps) - 1) / span

        with lock:
            if len(vals) == 10:
                state["raw"] = (vals[0], vals[1])
                state["kal"] = (vals[2], vals[3])
                state["arrow"] = (vals[4], vals[5], vals[6])
                state["state"] = int(vals[7])
                state["cmd"] = (vals[8], vals[9])
                state["have_raw"] = True
                state["have_arrow"] = True
            elif len(vals) == 7:
                state["raw"] = (vals[0], vals[1])
                state["kal"] = (vals[2], vals[3])
                state["arrow"] = (vals[4], vals[5], vals[6])
                state["have_raw"] = True
                state["have_arrow"] = True
            elif len(vals) == 4:
                state["raw"] = (vals[0], vals[1])
                state["kal"] = (vals[2], vals[3])
                state["have_raw"] = True
                state["have_arrow"] = False
            else:
                state["kal"] = (vals[0], vals[1])
                state["have_raw"] = False
                state["have_arrow"] = False
            state["hz"] = hz
            state["status"] = "streaming"

    try:
        ser.close()
    except Exception:
        pass


# ---------------- Rocket geometry ----------------
STATE_NAME = {0: "PAD", 1: "BOOST", 2: "COAST"}


def state_label(v):
    """The firmware adds 10 to the state field while armed, so one field
    carries both; 12 means COAST and armed."""
    if v is None:
        return "no state field (old firmware)"
    armed = v >= 10
    name = STATE_NAME.get(v % 10, f"? {v}")
    return f"{name}{'  ARMED' if armed else ''}"


def build_rocket():
    w = 0.35
    body_bottom, body_top, nose_tip = -1.2, 0.4, 1.6
    corners = np.array([
        [-w, -w, body_bottom], [w, -w, body_bottom],
        [w,  w, body_bottom], [-w,  w, body_bottom],
        [-w, -w, body_top],   [w, -w, body_top],
        [w,  w, body_top],    [-w,  w, body_top],
    ])
    nose = np.array([[0.0, 0.0, nose_tip]])
    points = np.vstack([corners, nose])
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
        (4, 8), (5, 8), (6, 8), (7, 8),
    ]
    return points, edges


def rotation_from_angles(pitch_deg, roll_deg):
    """Matches the sketch's angle estimator:
       roll  = atan2(Ay, Az)              -> full +/-180
       pitch = atan2(-Ax, hypot(Ay, Az))  -> +/-90
       Body to world is Ry(pitch) @ Rx(roll)."""
    p = np.radians(pitch_deg)
    r = np.radians(roll_deg)
    Ry = np.array([[ np.cos(p), 0, np.sin(p)],
                   [ 0,         1, 0        ],
                   [-np.sin(p), 0, np.cos(p)]])
    Rx = np.array([[1, 0,          0        ],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r),  np.cos(r)]])
    return Ry @ Rx


def rotation_from_arrow(v):
    """The arrow points to the floor in rocket coordinates; builds the
    rotation that carries it back onto world up. No angles, so no pole
    and no wraparound."""
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-9:
        return np.eye(3)
    v = v / n
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.clip(v @ z, -1.0, 1.0))
    axis = np.cross(v, z)
    s = np.linalg.norm(axis)
    if s < 1e-9:
        # arrow is already along z, or exactly opposite it (rocket inverted)
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    axis = axis / s
    K = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    th = np.arccos(c)
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * (K @ K)


def slew_angle(cur, target, a):
    """Smooths the short way around, so 179 -> -179 does not sweep back
    through zero."""
    d = ((target - cur + 180.0) % 360.0) - 180.0
    return ((cur + a * d + 180.0) % 360.0) - 180.0


def slew_vec(cur, target, a):
    out = cur + a * (np.asarray(target, dtype=float) - cur)
    n = np.linalg.norm(out)
    return out / n if n > 1e-9 else cur


def main():
    global running, show_ghost, show_angle, show_arrow, smoothing

    args = list(sys.argv[1:])
    if "--list" in args:
        list_ports()
        return

    port_name = args[0] if args else find_port()
    if not port_name:
        print("No serial ports found. Plug in the board, or run with --list.")
        return

    t = threading.Thread(target=reader_thread, args=(port_name,), daemon=True)
    t.start()
    time.sleep(0.3)
    if not running:
        return

    points, edges = build_rocket()

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")
    try:
        fig.canvas.manager.set_window_title("TVC IMU — q quit, g/a/v toggle, s smoothing")
    except Exception:
        pass

    def on_key(event):
        global running, show_ghost, show_angle, show_arrow, smoothing
        if event.key == "q":
            running = False
            plt.close("all")
        elif event.key == "g":
            show_ghost = not show_ghost
        elif event.key == "a":
            show_angle = not show_angle
        elif event.key == "v":
            show_arrow = not show_arrow
        elif event.key == "s":
            smoothing = not smoothing
    fig.canvas.mpl_connect("key_press_event", on_key)

    lim = 1.8
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z (nose up)")
    try:
        ax.set_proj_type("ortho")
    except Exception:
        pass

    ax_len = 0.4
    ax.plot([0, ax_len], [0, 0], [-lim, -lim], color="#ff6b6b", lw=1.5)
    ax.plot([0, 0], [0, ax_len], [-lim, -lim], color="#6bff8f", lw=1.5)
    ax.plot([0, 0], [0, 0], [-lim, -lim + ax_len], color="#6ba8ff", lw=1.5)

    base = [[points[i], points[j]] for i, j in edges]
    ghost_lines = Line3DCollection(base, colors="#5a6472", linewidths=1.2)
    angle_lines = Line3DCollection(base, colors="#ffb454", linewidths=2.2)
    arrow_lines = Line3DCollection(base, colors="#4ec9ff", linewidths=2.2)
    ax.add_collection3d(ghost_lines)
    ax.add_collection3d(angle_lines)
    ax.add_collection3d(arrow_lines)

    txt = fig.text(0.02, 0.95, "", fontsize=10, family="monospace", color="#e6ecf3")
    legend = fig.text(0.02, 0.03,
                      "grey = raw   orange = angle kalman   blue = arrow kalman",
                      fontsize=9, family="monospace", color="#96a0ad")

    disp = {"kp": 0.0, "kr": 0.0, "rp": 0.0, "rr": 0.0,
            "v": np.array([0.0, 0.0, 1.0])}

    def set_seg(coll, R, visible):
        if not visible:
            coll.set_visible(False)
            return
        rot = points @ R.T
        coll.set_segments([[rot[i], rot[j]] for i, j in edges])
        coll.set_visible(True)

    def update(_frame):
        with lock:
            rp, rr = state["raw"]
            kp, kr = state["kal"]
            av = state["arrow"]
            have_raw = state["have_raw"]
            have_arrow = state["have_arrow"]
            hz = state["hz"]
            status = state["status"]
            fstate = state["state"]
            cmd = state["cmd"]

        a = SMOOTH_ALPHA if smoothing else 1.0
        disp["kp"] = slew_angle(disp["kp"], kp, a)
        disp["kr"] = slew_angle(disp["kr"], kr, a)
        disp["rp"] = slew_angle(disp["rp"], rp, a)
        disp["rr"] = slew_angle(disp["rr"], rr, a)
        disp["v"] = slew_vec(disp["v"], av, a)

        set_seg(ghost_lines, rotation_from_angles(disp["rp"], disp["rr"]),
                have_raw and show_ghost)
        set_seg(angle_lines, rotation_from_angles(disp["kp"], disp["kr"]),
                show_angle)
        set_seg(arrow_lines, rotation_from_arrow(disp["v"]),
                have_arrow and show_arrow)

        # tilt off vertical, from the arrow; stays honest to 180 deg, unlike either angle alone
        tilt = np.degrees(np.arccos(np.clip(disp["v"][2], -1.0, 1.0)))

        txt.set_text(
            f"angle kalman   pitch {kp:7.2f}   roll {kr:7.2f}\n"
            f"raw            pitch {rp:7.2f}   roll {rr:7.2f}\n"
            f"arrow kalman   {av[0]:6.3f} {av[1]:6.3f} {av[2]:6.3f}   tilt {tilt:5.1f}\n"
            f"{hz:5.1f} Hz   {status}"
        )
        return ghost_lines, angle_lines, arrow_lines, txt, legend

    ani = FuncAnimation(fig, update, interval=25, blit=False, cache_frame_data=False)
    fig._tvc_ani = ani        # keep a reference so it is not garbage collected
    plt.show()

    running = False
    print("\nClosed.")


if __name__ == "__main__":
    main()
