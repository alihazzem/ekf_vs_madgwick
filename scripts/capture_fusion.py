"""
capture_fusion.py  —  Simultaneously capture Myo EMG (BLE) and STM32 IMU (UART).

Produces two time-synchronised CSV files in ../emg_data/:
  fusion_emg_TIMESTAMP.csv  — EMG stream with PC wall-clock timestamp
  fusion_imu_TIMESTAMP.csv  — IMU stream with PC wall-clock timestamp

Time alignment
--------------
  Both CSVs share a pc_t_ms column derived from the same PC clock origin
  (set at the moment MPU STREAM ON is sent).  In MATLAB, use interp1 or
  nearest-neighbour matching on pc_t_ms to align the two streams.

Usage
-----
  python capture_fusion.py                     # defaults below
  python capture_fusion.py COM3                # override COM port
  python capture_fusion.py COM3 115200 2       # port, baud, IMU print divisor

EMG CSV columns
---------------
  pc_t_ms     : PC wall-clock ms since capture start (common time axis)
  emg_t_ms    : elapsed ms from Myo callback timestamps (native Myo clock)
  emg0–emg7   : 8-channel signed EMG (-128 to +127)

IMU CSV columns (19 fields — identical to capture_ekf.py plus pc_t_ms)
-----------------------------------------------------------------------
  pc_t_ms     : PC wall-clock ms since capture start (common time axis)
  t_ms        : board timestamp from HAL_GetTick()
  ax_raw .. gz_raw             : raw int16 sensor counts
  mad_roll_mdeg .. mad_us      : Madgwick filter output
  ekf_roll_mdeg .. ekf_us      : EKF filter output
  bx_uradps .. bz_uradps       : EKF gyro bias estimate

Dependencies
------------
  pip install myo-python pyserial
  Myo Connect must be running.
"""

import sys
import csv
import os
import time
import math
import threading
import serial
import myo
from myo import Hub, DeviceListener
from datetime import datetime
from collections import deque


# ── Configuration ─────────────────────────────────────────────────────────────
PORT      = "COM7"
BAUD      = 115200
PRINT_DIV = 1          # 1 = 100 Hz, 2 = 50 Hz
SDK_PATH  = ""         # set to Myo SDK bin/ path if myo.init() fails
OUT_DIR   = "../emg_data"

# Online EMG processing (kept causal / low-latency)
ONLINE_EMG_PROC = True
EMG_FS_HZ = 200.0
HP_HZ = 20.0
NOTCH_HZ = 0.0        # set 50.0 or 60.0 to enable
NOTCH_Q = 30.0
ENV_MS = 150.0
FEAT_WIN_MS = 100.0
FEAT_STEP_MS = 50.0
ZC_THRESH = 5.0
# ─────────────────────────────────────────────────────────────────────────────

EMG_HEADER = [
    "pc_t_ms", "emg_t_ms",
    "emg0", "emg1", "emg2", "emg3",
    "emg4", "emg5", "emg6", "emg7",
]

IMU_HEADER = [
    "pc_t_ms", "t_ms",
    "ax_raw", "ay_raw", "az_raw",
    "gx_raw", "gy_raw", "gz_raw",
    "ekf_roll_mdeg", "ekf_pitch_mdeg", "ekf_yaw_mdeg",
    "traceP_1e6", "ekf_us",
    "bx_uradps", "by_uradps", "bz_uradps",
]

# Incoming firmware stream still contains 19 numeric fields after 'D,'.
IMU_STREAM_FIELDS = 19
DATA_PREFIX     = b"D,"

EMG_PROC_HEADER = ["pc_t_ms", "emg_t_ms"]
EMG_PROC_HEADER += [f"emg_raw{i}" for i in range(8)]
EMG_PROC_HEADER += [f"emg_hp{i}" for i in range(8)]
EMG_PROC_HEADER += [f"emg_rect{i}" for i in range(8)]
EMG_PROC_HEADER += [f"emg_env{i}" for i in range(8)]

EMG_FEAT_HEADER = ["pc_t_ms", "emg_t_ms"]
EMG_FEAT_HEADER += [f"mav{i}" for i in range(8)]
EMG_FEAT_HEADER += [f"rms{i}" for i in range(8)]
EMG_FEAT_HEADER += [f"zc{i}" for i in range(8)]
EMG_FEAT_HEADER += [f"wl{i}" for i in range(8)]
EMG_FEAT_HEADER += [f"env_mean{i}" for i in range(8)]


def send_cmd(ser: serial.Serial, cmd: str):
    ser.write((cmd.strip() + "\r\n").encode())
    time.sleep(0.15)


def pc_time_ms(origin_ns: int) -> float:
    """Monotonic high-resolution timestamp in ms relative to shared origin."""
    if origin_ns <= 0:
        return 0.0
    return (time.perf_counter_ns() - origin_ns) / 1e6


def imu_fields_to_ekf_only(fields_19: list) -> list:
    """
    Convert full 19-field stream row to EKF-only row by dropping Madgwick fields.

    Full order (19):
        t_ms, ax, ay, az, gx, gy, gz,
        mad_roll, mad_pitch, mad_yaw, mad_us,
        ekf_roll, ekf_pitch, ekf_yaw, traceP, ekf_us, bx, by, bz

    EKF-only order (15):
        t_ms, ax, ay, az, gx, gy, gz,
        ekf_roll, ekf_pitch, ekf_yaw, traceP, ekf_us, bx, by, bz
    """
    return (
        fields_19[0:7]
        + fields_19[11:16]
        + fields_19[16:19]
    )


def zc_deadzone(sig: list, thresh: float) -> int:
    count = 0
    for i in range(len(sig) - 1):
        a = sig[i]
        b = sig[i + 1]
        if (a * b < 0.0) and (abs(a) >= thresh) and (abs(b) >= thresh):
            count += 1
    return count


class OnlineEmgProcessor:
    """Sample-by-sample causal EMG processing + hop-based window features."""

    def __init__(self, fs_hz: float, hp_hz: float, notch_hz: float,
                 notch_q: float, env_ms: float, feat_win_ms: float,
                 feat_step_ms: float, zc_thresh: float):
        self.fs_hz = fs_hz
        self.zc_thresh = zc_thresh

        # High-pass state
        dt = 1.0 / fs_hz
        rc = 1.0 / (2.0 * math.pi * hp_hz) if hp_hz > 0 else 0.0
        self.hp_alpha = (rc / (rc + dt)) if hp_hz > 0 else 0.0
        self.hp_prev_x = [0.0] * 8
        self.hp_prev_y = [0.0] * 8

        # Optional notch biquad state
        self.notch_en = notch_hz > 0 and notch_hz < 0.5 * fs_hz
        self.nb = [1.0, 0.0, 0.0]
        self.na = [1.0, 0.0, 0.0]
        self.nb_x1 = [0.0] * 8
        self.nb_x2 = [0.0] * 8
        self.nb_y1 = [0.0] * 8
        self.nb_y2 = [0.0] * 8
        if self.notch_en:
            w0 = 2.0 * math.pi * notch_hz / fs_hz
            alpha = math.sin(w0) / (2.0 * notch_q)
            b0 = 1.0
            b1 = -2.0 * math.cos(w0)
            b2 = 1.0
            a0 = 1.0 + alpha
            a1 = -2.0 * math.cos(w0)
            a2 = 1.0 - alpha
            self.nb = [b0 / a0, b1 / a0, b2 / a0]
            self.na = [1.0, a1 / a0, a2 / a0]

        # Envelope trailing RMS state
        self.env_win = max(1, int(round(env_ms * fs_hz / 1000.0)))
        self.env_buf = [deque(maxlen=self.env_win) for _ in range(8)]
        self.env_sumsq = [0.0] * 8

        # Feature window/hop buffers
        self.feat_win = max(1, int(round(feat_win_ms * fs_hz / 1000.0)))
        self.feat_step = max(1, int(round(feat_step_ms * fs_hz / 1000.0)))
        self.feat_buf = [deque(maxlen=self.feat_win) for _ in range(8)]
        self.env_feat_buf = [deque(maxlen=self.feat_win) for _ in range(8)]
        self.feat_pc_t_buf = deque(maxlen=self.feat_win)
        self.feat_emg_t_buf = deque(maxlen=self.feat_win)
        self.hop_counter = 0

    def _hp_step(self, x: float, ch: int) -> float:
        if self.hp_alpha <= 0.0:
            return x
        y = self.hp_alpha * (self.hp_prev_y[ch] + x - self.hp_prev_x[ch])
        self.hp_prev_x[ch] = x
        self.hp_prev_y[ch] = y
        return y

    def _notch_step(self, x: float, ch: int) -> float:
        if not self.notch_en:
            return x
        b0, b1, b2 = self.nb
        _, a1, a2 = self.na
        y = (b0 * x + b1 * self.nb_x1[ch] + b2 * self.nb_x2[ch]
             - a1 * self.nb_y1[ch] - a2 * self.nb_y2[ch])
        self.nb_x2[ch] = self.nb_x1[ch]
        self.nb_x1[ch] = x
        self.nb_y2[ch] = self.nb_y1[ch]
        self.nb_y1[ch] = y
        return y

    def _env_step(self, x: float, ch: int) -> float:
        buf = self.env_buf[ch]
        if len(buf) == self.env_win:
            old = buf[0]
            self.env_sumsq[ch] -= old * old
        buf.append(x)
        self.env_sumsq[ch] += x * x
        n = len(buf)
        if n <= 0:
            return 0.0
        return math.sqrt(max(0.0, self.env_sumsq[ch] / float(n)))

    def update(self, emg_raw, pc_t_ms, emg_t_ms):
        hp = [0.0] * 8
        rect = [0.0] * 8
        env = [0.0] * 8

        for ch in range(8):
            x = float(emg_raw[ch])
            y = self._hp_step(x, ch)
            y = self._notch_step(y, ch)
            hp[ch] = y
            rect[ch] = abs(y)
            env[ch] = self._env_step(y, ch)

            self.feat_buf[ch].append(y)
            self.env_feat_buf[ch].append(env[ch])

        self.feat_pc_t_buf.append(float(pc_t_ms))
        self.feat_emg_t_buf.append(float(emg_t_ms))

        self.hop_counter += 1
        feat_row = None
        feat_pc_t = None
        feat_emg_t = None
        if self.hop_counter >= self.feat_step and len(self.feat_buf[0]) == self.feat_win:
            self.hop_counter = 0

            mav = []
            rms = []
            zc = []
            wl = []
            env_mean = []

            for ch in range(8):
                w = list(self.feat_buf[ch])
                e = list(self.env_feat_buf[ch])

                abs_w = [abs(v) for v in w]
                mav.append(sum(abs_w) / float(len(abs_w)))
                rms.append(math.sqrt(sum(v * v for v in w) / float(len(w))))
                zc.append(float(zc_deadzone(w, self.zc_thresh)))
                wl.append(sum(abs(w[i + 1] - w[i]) for i in range(len(w) - 1)))
                env_mean.append(sum(e) / float(len(e)))

            feat_row = mav + rms + zc + wl + env_mean

            # Window-center timestamps reduce feature time jitter.
            mid = len(self.feat_pc_t_buf) // 2
            feat_pc_t = list(self.feat_pc_t_buf)[mid]
            feat_emg_t = list(self.feat_emg_t_buf)[mid]

        return hp, rect, env, feat_row, feat_pc_t, feat_emg_t


# ── IMU reader thread ─────────────────────────────────────────────────────────

def imu_reader_thread(ser: serial.Serial, writer: csv.writer,
                      counters: list, running: threading.Event,
                      t_origin_ns: list):
    """
    Reads D, lines from UART and writes them to the IMU CSV.
    t_origin_ns[0] is set by main() when streaming starts — provides the
    common PC clock origin shared with the EMG listener.
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
                if len(parts) == IMU_STREAM_FIELDS + 1:
                    fields = [int(p) for p in parts[1:]]
                elif len(parts) == 11:
                    # Tolerate old 10-field stream (Madgwick-only era)
                    fields = [int(p) for p in parts[1:]] + [0] * (IMU_STREAM_FIELDS - 10)
                else:
                    continue
            except ValueError:
                continue

            pc_t = pc_time_ms(t_origin_ns[0])
            writer.writerow([pc_t] + imu_fields_to_ekf_only(fields))
            counters[1] += 1
        else:
            text = line.decode(errors="replace")
            if text:
                print(f"\n  [IMU] >> {text}", flush=True)


# ── Myo EMG listener ──────────────────────────────────────────────────────────

class FusionEmgListener(DeviceListener):

    def __init__(self, writer: csv.writer, counters: list,
                 running: threading.Event, t_origin_ns: list,
                 emg_proc_writer: csv.writer = None,
                 emg_feat_writer: csv.writer = None,
                 online_proc: OnlineEmgProcessor = None):
        super().__init__()
        self._writer = writer
        self._counters = counters
        self._running = running
        self._t_origin_ns = t_origin_ns
        self._t_start = None   # first Myo callback timestamp (µs)
        self._proc_writer = emg_proc_writer
        self._feat_writer = emg_feat_writer
        self._online_proc = online_proc

    def on_connected(self, event):
        print("  [EMG] Myo connected — streaming EMG")
        event.device.stream_emg(True)

    def on_disconnected(self, event):
        print("\n  [EMG] Myo disconnected")

    def on_emg(self, event):
        if not self._running.is_set():
            return

        if self._t_start is None:
            self._t_start = event.timestamp

        emg_t_ms = int((event.timestamp - self._t_start) / 1000)

        pc_t = pc_time_ms(self._t_origin_ns[0])

        emg_raw = list(event.emg)
        self._writer.writerow([pc_t, emg_t_ms] + emg_raw)

        if self._online_proc and self._proc_writer:
            hp, rect, env, feat, feat_pc_t, feat_emg_t = self._online_proc.update(emg_raw, pc_t, emg_t_ms)
            self._proc_writer.writerow([pc_t, emg_t_ms] + emg_raw + hp + rect + env)
            if feat is not None and self._feat_writer:
                self._feat_writer.writerow([feat_pc_t, feat_emg_t] + feat)
                self._counters[2] += 1

        self._counters[0] += 1


# ── Progress printer ──────────────────────────────────────────────────────────

def progress_thread(counters: list, running: threading.Event):
    """Prints a combined sample count line every second."""
    while running.is_set():
        time.sleep(1.0)
        print(f"  EMG: {counters[0]:5d} samples  |  "
              f"IMU: {counters[1]:5d} samples  |  "
              f"FEAT: {counters[2]:5d} windows      ",
              end="\r", flush=True)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    port      = sys.argv[1] if len(sys.argv) > 1 else PORT
    baud      = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD
    print_div = int(sys.argv[3]) if len(sys.argv) > 3 else PRINT_DIV

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir    = os.path.normpath(os.path.join(script_dir, OUT_DIR))
    os.makedirs(out_dir, exist_ok=True)

    emg_path = os.path.join(out_dir, f"fusion_emg_{timestamp}.csv")
    imu_path = os.path.join(out_dir, f"fusion_imu_{timestamp}.csv")
    emg_proc_path = os.path.join(out_dir, f"fusion_emg_online_{timestamp}.csv")
    emg_feat_path = os.path.join(out_dir, f"fusion_emg_feat_{timestamp}.csv")

    print(f"\nEMG out : {emg_path}")
    print(f"IMU out : {imu_path}")
    if ONLINE_EMG_PROC:
        print(f"EMG proc: {emg_proc_path}")
        print(f"EMG feat: {emg_feat_path}")
    print("─" * 60)

    # ── Open serial ───────────────────────────────────────────────────────────
    print(f"Opening {port} at {baud} baud ...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: cannot open {port} — {e}")
        sys.exit(1)

    # ── Init Myo SDK ──────────────────────────────────────────────────────────
    print("Initialising Myo SDK ...")
    try:
        myo.init(sdk_path=SDK_PATH) if SDK_PATH else myo.init()
    except Exception as e:
        print(f"ERROR: myo.init() failed — {e}")
        print("Set SDK_PATH at the top of this script.")
        ser.close()
        sys.exit(1)

    # ── Open CSV files ────────────────────────────────────────────────────────
    emg_f  = open(emg_path, "w", newline="")
    imu_f  = open(imu_path, "w", newline="")
    emg_proc_f = None
    emg_feat_f = None

    emg_wr = csv.writer(emg_f)
    imu_wr = csv.writer(imu_f)
    emg_wr.writerow(EMG_HEADER)
    imu_wr.writerow(IMU_HEADER)

    emg_proc_wr = None
    emg_feat_wr = None
    online_proc = None
    if ONLINE_EMG_PROC:
        emg_proc_f = open(emg_proc_path, "w", newline="")
        emg_feat_f = open(emg_feat_path, "w", newline="")
        emg_proc_wr = csv.writer(emg_proc_f)
        emg_feat_wr = csv.writer(emg_feat_f)
        emg_proc_wr.writerow(EMG_PROC_HEADER)
        emg_feat_wr.writerow(EMG_FEAT_HEADER)
        online_proc = OnlineEmgProcessor(
            fs_hz=EMG_FS_HZ,
            hp_hz=HP_HZ,
            notch_hz=NOTCH_HZ,
            notch_q=NOTCH_Q,
            env_ms=ENV_MS,
            feat_win_ms=FEAT_WIN_MS,
            feat_step_ms=FEAT_STEP_MS,
            zc_thresh=ZC_THRESH,
        )

    # counters[0] = EMG samples, counters[1] = IMU samples, counters[2] = feature windows
    counters = [0, 0, 0]
    running  = threading.Event()
    running.set()
    t_origin_ns = [0]   # shared monotonic origin (ns) — set when streaming starts

    # ── Start IMU reader thread ───────────────────────────────────────────────
    imu_t = threading.Thread(
        target=imu_reader_thread,
        args=(ser, imu_wr, counters, running, t_origin_ns),
        daemon=True)
    imu_t.start()

    # ── MPU init + calibration ────────────────────────────────────────────────
    time.sleep(0.2)
    send_cmd(ser, "MPU INIT")

    print("\nPlace the board FLAT and STILL for gyro calibration.")
    choice = input("Press Enter to calibrate (3 s), or 's' to skip: ").strip().lower()
    if choice != "s":
        print("Calibrating ...")
        send_cmd(ser, "MPU CAL GYRO 3000")
        time.sleep(3.5)
        print("Calibration done.\n")
    else:
        print("Calibration skipped.\n")

    # ── Start both streams ────────────────────────────────────────────────────
    print("Waiting for Myo armband connection ...")
    hub      = Hub()
    listener = FusionEmgListener(
        emg_wr,
        counters,
        running,
        t_origin_ns,
        emg_proc_writer=emg_proc_wr,
        emg_feat_writer=emg_feat_wr,
        online_proc=online_proc,
    )

    with hub.run_in_background(listener):
        # Give the Myo a moment to connect before starting the IMU stream
        time.sleep(1.0)

        # Set the shared clock origin — both streams timestamp relative to this
        t_origin_ns[0] = time.perf_counter_ns()

        send_cmd(ser, f"MPU PRINT {print_div}")
        send_cmd(ser, "MPU STREAM ON")

        # Start progress printer
        prog_t = threading.Thread(
            target=progress_thread, args=(counters, running), daemon=True)
        prog_t.start()

        print("Both streams running. Type CLI commands or press Enter to stop.\n")

        # ── Interactive command loop ──────────────────────────────────────────
        try:
            while running.is_set():
                try:
                    cmd = input()
                except EOFError:
                    break
                cmd = cmd.strip()
                if not cmd:
                    break
                if cmd.lower() in ("q", "quit", "exit"):
                    break
                send_cmd(ser, cmd)
        except KeyboardInterrupt:
            pass

    # ── Shutdown ──────────────────────────────────────────────────────────────
    print(f"\n\nStopping streams ...")
    try:
        send_cmd(ser, "MPU STREAM OFF")
    except Exception:
        pass

    running.clear()
    imu_t.join(timeout=2)
    emg_f.close()
    imu_f.close()
    if emg_proc_f is not None:
        emg_proc_f.close()
    if emg_feat_f is not None:
        emg_feat_f.close()
    ser.close()

    print(f"Done.")
    print(f"  EMG : {counters[0]} samples  → {emg_path}")
    print(f"  IMU : {counters[1]} samples  → {imu_path}\n")
    if ONLINE_EMG_PROC:
        print(f"  EMG online : {counters[0]} rows  → {emg_proc_path}")
        print(f"  EMG feat   : {counters[2]} rows  → {emg_feat_path}\n")


if __name__ == "__main__":
    main()
