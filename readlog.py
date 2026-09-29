#!/usr/bin/env python3
"""
Download and plot a flight log from the rocket.

The board keeps its logs in flash. This connects over serial, drives the bench
menu for you, captures the CSV, saves it to a file and plots it.

Usage:
    python3 readlog.py --list          # what is on flash
    python3 readlog.py                 # newest log: download, save, plot
    python3 readlog.py --slot 0        # a specific slot
    python3 readlog.py --plot fl.csv   # re-plot a file you already have
    python3 readlog.py --erase         # wipe flash (dump first)
    python3 readlog.py --set-clock     # give the board the current time

Nothing is deleted from flash unless you ask with --erase.
"""

import argparse
import csv
import os
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Missing pyserial.  pip3 install pyserial")
    sys.exit(1)

# tried first; falls back to autodetecting a usbserial/wchusbserial port
DEFAULT_PORT = "/dev/cu.wchusbserial5B7A1145741"
BAUD = 115200

C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, MUT, GRID, BG = "#0b0b0b", "#52514e", "#dcdcd6", "#fcfcfb"


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if DEFAULT_PORT in ports:
        return DEFAULT_PORT
    for d in ports:
        low = d.lower()
        if any(k in low for k in ("wchusbserial", "usbserial", "usbmodem", "ttyusb", "ttyacm")):
            return d
    return ports[0] if ports else None


def open_serial(port):
    """DTR/RTS low, so opening the port doesn't reset the board or
    interrupt anything in progress."""
    s = serial.Serial()
    s.port = port
    s.baudrate = BAUD
    s.timeout = 1.0
    s.dtr = False
    s.rts = False
    s.open()
    return s


def talk(ser, cmd, until, timeout=90.0, echo=False):
    """Send a command, collect lines until `until` matches or we time out."""
    ser.reset_input_buffer()
    ser.write(cmd.encode())
    ser.flush()
    out, t0 = [], time.time()
    while time.time() - t0 < timeout:
        line = ser.readline().decode("utf-8", errors="ignore").rstrip("\r\n")
        if not line:
            continue
        out.append(line)
        if echo:
            print(line)
        if until and until(line):
            break
    return out


def wait_boot(ser, timeout=30.0):
    """Finds out what the board is doing before sending it anything:
    opening the port resets it, and a 't' sent during the several-second
    boot is eaten or discarded, never reaching the menu.

    Returns one of: ready, bench, running, quiet, timeout.
    """
    t0, quiet = time.time(), 0
    while time.time() - t0 < timeout:
        line = ser.readline().decode("utf-8", errors="ignore").rstrip("\r\n")
        if not line:
            quiet += 1
            # six seconds of silence likely means the bench menu is already
            # drawn; the handshake below confirms it either way
            if quiet >= 6:
                return "quiet"
            continue
        quiet = 0
        if "READY" in line:
            return "ready"
        if "TVC BENCH" in line:
            return "bench"
        if line[:1].isdigit():
            return "running"       # streaming CSV, so the main loop is up
    return "timeout"


def enter_bench(ser, tries=4):
    """Gets to the bench menu and confirms it: 't' enters from the main
    loop but is ignored once inside, so each attempt follows with '?' too;
    the banner printed either way is the confirmation.
    """
    if wait_boot(ser) == "timeout":
        print("The board is not saying anything. Check the cable and the port.")
        return False

    for _ in range(tries):
        ser.reset_input_buffer()
        ser.write(b"t")
        ser.flush()
        time.sleep(0.3)
        ser.write(b"?")
        ser.flush()

        refused = False
        t0 = time.time()
        while time.time() - t0 < 5.0:
            line = ser.readline().decode("utf-8", errors="ignore").rstrip("\r\n")
            if not line:
                continue
            if "TVC BENCH" in line:
                time.sleep(0.6)     # let the rest of the menu land first
                ser.reset_input_buffer()
                return True
            if "bench refused" in line:
                refused = True
                break

        if refused:
            # still armed or in a flight state; the bench won't open until that clears
            print("Board is armed. Disarming so the menu will open.")
            ser.write(b"D")
            ser.flush()
            time.sleep(0.8)

    print("Could not reach the bench menu.")
    print("If the gimbal is still armed, press DISARM on the handset, or")
    print("power cycle the rocket and run this again.")
    return False


def cmd_list(ser):
    if not enter_bench(ser):
        return
    for line in talk(ser, "F", lambda l: "file(s)" in l or "flash is empty" in l, 15):
        print(line)


def cmd_erase(ser):
    if not enter_bench(ser):
        return
    for line in talk(ser, "E", lambda l: "erased" in l, 15):
        print(line)


def cmd_set_clock(ser):
    if not enter_bench(ser):
        return
    now = int(time.time())
    for line in talk(ser, f"T{now}\n", lambda l: "clock set" in l, 10):
        print(line)


def cmd_download(ser, slot):
    if not enter_bench(ser):
        return None
    cmd = "L\n" if slot is None else f"L{slot}\n"
    print("Downloading, this takes a moment at 115200 baud...")
    lines = talk(ser, cmd, lambda l: l.startswith("# END"), 180)

    meta, rows, header, capturing = {}, [], None, False
    for line in lines:
        if line.startswith("# BEGIN"):
            for tok in line[7:].split():
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    meta[k] = v
            capturing = True
            continue
        if line.startswith("# END"):
            break
        if not capturing or line.startswith("#"):
            continue
        if header is None:
            header = line.split(",")
            continue
        parts = line.split(",")
        if len(parts) == len(header):
            rows.append(parts)

    if not rows:
        print("No data captured. Check the board is at the bench menu and the log exists.")
        return None

    name = f"flight_slot{meta.get('slot', 'x')}.csv"
    with open(name, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print(f"Saved {len(rows)} samples to {name}")
    for k, v in meta.items():
        print(f"  {k}: {v}")
    return name


def plot(path):
    try:
        import numpy as np
        import matplotlib.pyplot as plt
    except ImportError:
        print("(pip3 install matplotlib numpy to plot)")
        return

    d = np.genfromtxt(path, delimiter=",", names=True)

    def col(*names):
        """Accepts either column spelling: pitch/yaw moved onto gimbal
        axes at some point, and older CSVs use the old names."""
        for n in names:
            if n in d.dtype.names:
                return d[n]
        raise KeyError(f"none of {names} in {d.dtype.names}")

    t = (d["t_ms"] - d["t_ms"][0]) / 1000.0
    tilt = np.degrees(np.arccos(np.clip(col("vz"), -1, 1)))
    state = d["state"]

    fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True,
                            gridspec_kw={"height_ratios": [2, 1.3, 1.3]})
    fig.patch.set_facecolor(BG)
    for ax in axs:
        ax.set_facecolor(BG)
        ax.grid(True, color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        for s in ("left", "bottom"):
            ax.spines[s].set_color(GRID)
        ax.tick_params(colors=MUT, labelsize=9)
        # shade the powered phase
        boost = t[state == 1]
        if len(boost):
            ax.axvspan(boost[0], boost[-1], color="#eb6834", alpha=0.08, lw=0)

    a, b, c = axs
    a.plot(t, col("gimPitch_deg", "pitch"), lw=1.6, color=C1, label="pitch (gimbal axis)")
    a.plot(t, col("gimYaw_deg", "yaw"), lw=1.6, color=C2, label="yaw (gimbal axis)")
    a.plot(t, tilt, lw=2.2, color=C3, label="tilt off vertical")
    a.set_ylabel("degrees", color=MUT, fontsize=10)
    a.set_title(f"{os.path.basename(path)}   orange band = motor burning",
                color=INK, fontsize=12, loc="left", pad=12)
    leg = a.legend(frameon=False, fontsize=9, loc="upper left")
    for tx in leg.get_texts():
        tx.set_color(MUT)

    b.plot(t, col("accG"), lw=1.8, color=C1)
    b.axhline(1.8, color=MUT, ls="--", lw=1.0)
    b.axhline(1.0, color=MUT, ls=":", lw=1.0)
    b.annotate("BOOST_G 1.8", (0.99, 0.86), xycoords="axes fraction",
               ha="right", fontsize=8, color=MUT)
    b.annotate("BURNOUT_G 1.0", (0.99, 0.06), xycoords="axes fraction",
               ha="right", fontsize=8, color=MUT)
    b.set_ylabel("accel, g", color=MUT, fontsize=10)
    b.set_title("what the accelerometer saw. Set your thresholds from this.",
                color=INK, fontsize=10, loc="left", pad=8)

    c.plot(t, col("cmdP_deg", "cmdP"), lw=1.6, color=C1, label="pitch command")
    c.plot(t, col("cmdY_deg", "cmdY"), lw=1.6, color=C2, label="yaw command")
    c.set_ylabel("gimbal, deg", color=MUT, fontsize=10)
    c.set_xlabel("seconds from start of log", color=MUT, fontsize=10)
    leg = c.legend(frameon=False, fontsize=9, loc="upper left")
    for tx in leg.get_texts():
        tx.set_color(MUT)

    out = os.path.splitext(path)[0] + ".png"
    plt.tight_layout()
    plt.savefig(out, dpi=150, facecolor=BG)
    print(f"Plot saved to {out}")
    plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default=None)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--slot", type=int, default=None)
    ap.add_argument("--erase", action="store_true")
    ap.add_argument("--set-clock", action="store_true")
    ap.add_argument("--plot", default=None, help="re-plot an existing CSV, no board needed")
    a = ap.parse_args()

    if a.plot:
        plot(a.plot)
        return

    port = a.port or find_port()
    if not port:
        print("No serial port found.")
        return
    ser = open_serial(port)
    print(f"Connected to {port}")
    try:
        if a.list:
            cmd_list(ser)
        elif a.erase:
            cmd_erase(ser)
        elif a.set_clock:
            cmd_set_clock(ser)
        else:
            name = cmd_download(ser, a.slot)
            if name:
                plot(name)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
