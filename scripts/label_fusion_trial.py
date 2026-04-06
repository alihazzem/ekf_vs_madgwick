"""
label_fusion_trial.py  -- Register labeled fusion trials in a metadata CSV.

This script appends one row per trial to ../emg_data/trial_labels.csv.
It can auto-pick the latest capture timestamp or use a provided timestamp.

Usage
-----
python label_fusion_trial.py --gesture REST --trial 1 --session S01
python label_fusion_trial.py --gesture FIST --trial 2 --session S01 --notes "good quality"
python label_fusion_trial.py --timestamp 20260330_124817 --gesture OPEN --trial 3 --session S01
python label_fusion_trial.py --list-unlabeled

Expected files per timestamp
----------------------------
- fusion_emg_<timestamp>.csv
- fusion_imu_<timestamp>.csv
- fusion_emg_online_<timestamp>.csv
- fusion_emg_feat_<timestamp>.csv
"""

import argparse
import csv
import glob
import os
from datetime import datetime

DATA_DIR = "../emg_data"
LABEL_FILE = "trial_labels.csv"


def parse_args():
    p = argparse.ArgumentParser(description="Add labels for fusion capture trials.")
    p.add_argument("--timestamp", default="", help="Capture timestamp, e.g. 20260330_124817")
    p.add_argument("--gesture", default="", help="Gesture label, e.g. REST, FIST, OPEN")
    p.add_argument("--trial", type=int, default=0, help="Trial number within session")
    p.add_argument("--session", default="", help="Session ID, e.g. S01")
    p.add_argument("--subject", default="", help="Optional subject/operator ID")
    p.add_argument("--notes", default="", help="Optional notes")
    p.add_argument("--list-unlabeled", action="store_true", help="List feature timestamps not in labels")
    return p.parse_args()


def find_latest_timestamp(data_dir):
    feat_files = sorted(glob.glob(os.path.join(data_dir, "fusion_emg_feat_*.csv")))
    if not feat_files:
        raise FileNotFoundError("No fusion_emg_feat_*.csv files found.")
    latest = os.path.basename(feat_files[-1])
    return latest[len("fusion_emg_feat_"):-4]


def make_paths(data_dir, ts):
    return {
        "emg_raw": os.path.join(data_dir, f"fusion_emg_{ts}.csv"),
        "imu": os.path.join(data_dir, f"fusion_imu_{ts}.csv"),
        "emg_online": os.path.join(data_dir, f"fusion_emg_online_{ts}.csv"),
        "emg_feat": os.path.join(data_dir, f"fusion_emg_feat_{ts}.csv"),
    }


def ensure_label_file(path):
    if os.path.exists(path):
        return
    header = [
        "timestamp",
        "gesture_label",
        "trial_id",
        "session_id",
        "subject_id",
        "notes",
        "created_at",
        "emg_raw_file",
        "imu_file",
        "emg_online_file",
        "emg_feat_file",
    ]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)


def load_existing_timestamps(label_path):
    if not os.path.exists(label_path):
        return set()
    out = set()
    with open(label_path, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            ts = row.get("timestamp", "").strip()
            if ts:
                out.add(ts)
    return out


def list_unlabeled(data_dir, label_path):
    existing = load_existing_timestamps(label_path)
    feat_files = sorted(glob.glob(os.path.join(data_dir, "fusion_emg_feat_*.csv")))
    all_ts = [os.path.basename(p)[len("fusion_emg_feat_"):-4] for p in feat_files]
    missing = [ts for ts in all_ts if ts not in existing]

    if not missing:
        print("No unlabeled feature files found.")
        return

    print("Unlabeled capture timestamps:")
    for ts in missing:
        print(ts)


def main():
    args = parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.normpath(os.path.join(script_dir, DATA_DIR))
    label_path = os.path.join(data_dir, LABEL_FILE)

    os.makedirs(data_dir, exist_ok=True)
    ensure_label_file(label_path)

    if args.list_unlabeled:
        list_unlabeled(data_dir, label_path)
        return

    if not args.gesture or args.trial <= 0 or not args.session:
        raise ValueError("Provide --gesture, --trial (>0), and --session")

    ts = args.timestamp.strip() if args.timestamp else find_latest_timestamp(data_dir)
    paths = make_paths(data_dir, ts)

    missing = [k for k, p in paths.items() if not os.path.exists(p)]
    if missing:
        raise FileNotFoundError(f"Missing files for timestamp {ts}: {', '.join(missing)}")

    created_at = datetime.now().isoformat(timespec="seconds")

    with open(label_path, "a", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            ts,
            args.gesture.strip().upper(),
            args.trial,
            args.session.strip(),
            args.subject.strip(),
            args.notes.strip(),
            created_at,
            os.path.basename(paths["emg_raw"]),
            os.path.basename(paths["imu"]),
            os.path.basename(paths["emg_online"]),
            os.path.basename(paths["emg_feat"]),
        ])

    print("Label added:")
    print(f"  timestamp: {ts}")
    print(f"  gesture  : {args.gesture.strip().upper()}")
    print(f"  trial    : {args.trial}")
    print(f"  session  : {args.session.strip()}")
    print(f"  file     : {label_path}")


if __name__ == "__main__":
    main()
