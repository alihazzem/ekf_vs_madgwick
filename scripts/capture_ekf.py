"""
capture_ekf.py  —  Capture EKF stream from STM32 over UART.
               Live-plots roll, pitch, yaw for axis verification.

Usage
-----
  python capture_ekf.py                   # uses defaults below
  python capture_ekf.py COM3              # override port
  python capture_ekf.py COM3 115200       # override port + baud
  python capture_ekf.py COM3 115200 2     # override print divisor (2 = 50 Hz)

How it works
------------
  - Auto-sends MPU INIT / MPU PRINT <N> / MPU STREAM ON.
  - Parses the firmware "D," lines and extracts EKF roll/pitch/yaw only.
  - Shows a live 3-panel plot (roll / pitch / yaw) that scrolls in real time.
  - CSV is still saved (same column layout as before — other tools still work).
  - Type any CLI command in the terminal and press Enter to forward it.
  - Type  q  or  quit  (or Ctrl+C) to stop.

Dependencies
------------
  pip install pyserial matplotlib
"""

import sys
import csv
import time
import threading
import collections
import serial
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime

# ── Configuration ──────────────────────────────────────────────────────────────
PORT        = "COM7"    # change to your port (e.g. /dev/ttyUSB0 on Linux)
BAUD        = 115200
PRINT_DIV   = 1         # 1 = 100 Hz, 2 = 50 Hz, 5 = 20 Hz

PLOT_WINDOW = 500       # number of samples shown in the live window (~5 s @ 100 Hz)
# ───────────────────────────────────────────────────────────────────────────────

CSV_HEADER = [
    "t_ms",
    "ax_raw", "ay_raw", "az_raw",
    "gx_raw", "gy_raw", "gz_raw",
    "mad_roll_mdeg", "mad_pitch_mdeg", "mad_yaw_mdeg", "mad_us",
    "ekf_roll_mdeg", "ekf_pitch_mdeg", "ekf_yaw_mdeg",
    "traceP_1e6", "ekf_us",
    "bx_uradps", "by_uradps", "bz_uradps",
]

# Column indices (0-based in the parsed row, after stripping the "D" prefix)
IDX_T_MS        = 0
IDX_MAD_ROLL    = 7    # mad_roll_mdeg
IDX_MAD_PITCH   = 8    # mad_pitch_mdeg
IDX_MAD_YAW     = 9    # mad_yaw_mdeg
IDX_EKF_ROLL    = 11   # ekf_roll_mdeg
IDX_EKF_PITCH   = 12   # ekf_pitch_mdeg
IDX_EKF_YAW     = 13   # ekf_yaw_mdeg

EXPECTED_FIELDS = len(CSV_HEADER)   # 19

DATA_PREFIX = b"D,"


# ── Shared ring buffers (written by reader thread, read by plot thread) ────────
_lock   = threading.Lock()
_t_buf  = collections.deque(maxlen=PLOT_WINDOW)
_r_buf  = collections.deque(maxlen=PLOT_WINDOW)
_p_buf  = collections.deque(maxlen=PLOT_WINDOW)
_y_buf  = collections.deque(maxlen=PLOT_WINDOW)
_rm_buf = collections.deque(maxlen=PLOT_WINDOW)
_pm_buf = collections.deque(maxlen=PLOT_WINDOW)
_ym_buf = collections.deque(maxlen=PLOT_WINDOW)


def send_cmd(ser: serial.Serial, cmd: str):
    """Send a CLI command and wait briefly for the board to reply."""
    ser.write((cmd.strip() + "\r\n").encode())
    time.sleep(0.15)


def reader_thread(ser: serial.Serial, writer: csv.writer,
                  counter: list, running: threading.Event):
    """
    Background thread: reads every line from the serial port.
    - 'D,...' lines  → parsed, written to CSV, EKF angles pushed to plot buffers
    - Everything else → printed to console as board reply
    """
    while running.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException:
            running.clear()
            break

        if not raw:
            continue

        line = raw.strip()

        if line.startswith(DATA_PREFIX):
            try:
                parts = line.split(b",")
                if len(parts) != EXPECTED_FIELDS + 1:
                    if len(parts) == 11:
                        row = [int(p) for p in parts[1:]] + [0] * (EXPECTED_FIELDS - 10)
                    else:
                        continue
                else:
                    row = [int(p) for p in parts[1:]]
            except ValueError:
                continue

            writer.writerow(row)
            counter[0] += 1

            # Push EKF and Madg angles to ring buffers (convert mdeg → deg)
            t_s   = row[IDX_T_MS] / 1000.0
            roll  = row[IDX_EKF_ROLL]  / 1000.0
            pitch = row[IDX_EKF_PITCH] / 1000.0
            yaw   = row[IDX_EKF_YAW]   / 1000.0
            
            m_roll = row[IDX_MAD_ROLL] / 1000.0
            m_pitch = row[IDX_MAD_PITCH] / 1000.0
            m_yaw = row[IDX_MAD_YAW] / 1000.0

            with _lock:
                _t_buf.append(t_s)
                _r_buf.append(roll)
                _p_buf.append(pitch)
                _y_buf.append(yaw)
                _rm_buf.append(m_roll)
                _pm_buf.append(m_pitch)
                _ym_buf.append(m_yaw)

            if counter[0] % 100 == 0:
                print(f"  [{counter[0]} samples | "
                      f"{counter[0] / 100.0:.1f} s @ 100 Hz]      ",
                      end="\r", flush=True)
        else:
            text = line.decode(errors="replace")
            if text:
                print(f"\n  >> {text}", flush=True)


def build_plot():
    """
    Create the live roll / pitch / yaw figure.
    Returns (fig, axes, line_objects).
    """
    matplotlib.rcParams.update({
        "figure.facecolor":  "#1a1a2e",
        "axes.facecolor":    "#16213e",
        "axes.edgecolor":    "#0f3460",
        "axes.labelcolor":   "#e0e0e0",
        "xtick.color":       "#888",
        "ytick.color":       "#888",
        "grid.color":        "#0f3460",
        "grid.linewidth":    0.8,
        "text.color":        "#e0e0e0",
        "font.family":       "monospace",
    })

    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    fig.suptitle("EKF vs Madgwick MARG — Live Roll / Pitch / Yaw", fontsize=13, fontweight="bold",
                 color="#e94560", y=0.98)
    fig.subplots_adjust(hspace=0.08, left=0.08, right=0.97, top=0.93, bottom=0.08)

    colors = ["#e94560", "#0f9b8e", "#f5a623"]
    labels = ["Roll  (deg)", "Pitch (deg)", "Yaw   (deg)"]
    lines_ekf  = []
    lines_madg = []

    for ax, color, label in zip(axes, colors, labels):
        # EKF Line (Solid)
        (ln_e,) = ax.plot([], [], color=color, linewidth=2.0, antialiased=True, label="EKF")
        # Madgwick Line (Dashed)
        (ln_m,) = ax.plot([], [], color="#a2aaca", linewidth=1.5, linestyle="--", antialiased=True, label="Madgwick")
        
        ax.set_ylabel(label, fontsize=10)
        ax.set_ylim(-185, 185)
        ax.axhline(0, color="#555", linewidth=0.6, linestyle="--")
        ax.grid(True, axis="y")
        ax.set_xlim(0, 5)          # will auto-scroll
        ax.legend(loc="upper left")
        lines_ekf.append(ln_e)
        lines_madg.append(ln_m)

    axes[-1].set_xlabel("Board time (s)", fontsize=10)

    # Value annotations on the right side of each panel
    anns = [ax.text(0.98, 0.88, "", transform=ax.transAxes,
                    ha="right", va="top", fontsize=11, fontweight="bold",
                    color="#ffffff", bbox=dict(boxstyle="round,pad=0.2",
                                       facecolor="#0d0d1a", alpha=0.8))
            for ax in axes]

    return fig, axes, lines_ekf, lines_madg, anns


def animate(frame, axes, lines_ekf, lines_madg, anns):
    """FuncAnimation callback — updates lines and scrolls x-axis."""
    with _lock:
        if len(_t_buf) < 2:
            return lines_ekf + lines_madg + anns
        ts = list(_t_buf)
        rs = list(_r_buf)
        ps = list(_p_buf)
        ys = list(_y_buf)
        rms = list(_rm_buf)
        pms = list(_pm_buf)
        yms = list(_ym_buf)

    data = [(rs, rms, "Roll"), (ps, pms, "Pitch"), (ys, yms, "Yaw")]
    t_min = ts[0]
    t_max = ts[-1]
    span  = max(t_max - t_min, 5.0)

    for ln_e, ln_m, ann, ax, (vals_e, vals_m, name) in zip(lines_ekf, lines_madg, anns, axes, data):
        ln_e.set_data(ts, vals_e)
        ln_m.set_data(ts, vals_m)
        ax.set_xlim(t_max - span, t_max)
        ann.set_text(f"{name}\nEKF: {vals_e[-1]:+7.2f}°\nMadg: {vals_m[-1]:+7.2f}°")

    return lines_ekf + lines_madg + anns


def input_loop(ser: serial.Serial, running: threading.Event):
    """Runs in main thread after plot is shown — forwards user input to board."""
    while running.is_set():
        try:
            cmd = input()
        except EOFError:
            break
        cmd = cmd.strip()
        if not cmd:
            continue
        if cmd.lower() in ("q", "quit", "exit"):
            running.clear()
            plt.close("all")
            break
        send_cmd(ser, cmd)


def main():
    port      = sys.argv[1] if len(sys.argv) > 1 else PORT
    baud      = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD
    print_div = int(sys.argv[3]) if len(sys.argv) > 3 else PRINT_DIV

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_file  = f"ekf_capture_{timestamp}.csv"

    print(f"\nOpening {port} at {baud} baud ...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: cannot open port — {e}")
        sys.exit(1)

    print(f"Output  : {out_file}")
    print("─" * 60)
    print("Live plot: Roll / Pitch / Yaw (EKF vs Madgwick MARG)")
    print("Type any CLI command and press Enter to forward to board.")
    print("Type  q  or  quit  (or Ctrl+C) to stop.\n")

    f      = open(out_file, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(CSV_HEADER)

    counter = [0]
    running = threading.Event()
    running.set()

    t_reader = threading.Thread(
        target=reader_thread, args=(ser, writer, counter, running), daemon=True
    )
    t_reader.start()

    # ── Auto-init sequence ─────────────────────────────────────────────────────
    time.sleep(0.2)
    send_cmd(ser, "MPU STREAM OFF")
    time.sleep(0.1)
    send_cmd(ser, "MPU INIT")

    # ── Calibration prompt ─────────────────────────────────────────────────────
    print("Place the board FLAT and STILL.")
    choice = input("Press Enter to calibrate gyro (3 s), or type 's' to skip: ").strip().lower()
    if choice != "s":
        print("Calibrating — keep the board still ...")
        send_cmd(ser, "MPU CAL GYRO 3000")
        time.sleep(3.5)
        print("Calibration done.\n")
    else:
        print("Calibration skipped.\n")

    # ── Start streaming ───────────────────────────────────────────────────────
    send_cmd(ser, f"MPU PRINT {print_div}")
    send_cmd(ser, "MPU STREAM ON")
    print("Streaming started — rotate the board to verify axes.\n")

    # ── Build and launch live plot ─────────────────────────────────────────────
    fig, axes, lines_ekf, lines_madg, anns = build_plot()

    ani = animation.FuncAnimation(
        fig, animate,
        fargs=(axes, lines_ekf, lines_madg, anns),
        interval=50,        # ~20 fps redraw
        blit=True,
        cache_frame_data=False,
    )

    # Run input loop in a separate thread so the plt event loop owns main thread
    inp_thread = threading.Thread(target=input_loop,
                                  args=(ser, running),
                                  daemon=True)
    inp_thread.start()

    try:
        plt.show()          # blocks until window is closed
    except KeyboardInterrupt:
        pass
    finally:
        running.clear()

    # ── Shutdown ──────────────────────────────────────────────────────────────
    print(f"\n\nStopping stream ...")
    try:
        send_cmd(ser, "MPU STREAM OFF")
    except Exception:
        pass

    t_reader.join(timeout=2)
    f.close()
    ser.close()

    print(f"Done. {counter[0]} samples saved to {out_file}\n")


if __name__ == "__main__":
    main()
