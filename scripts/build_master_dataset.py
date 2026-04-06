"""
build_master_dataset.py  -- Build one labeled master feature dataset from trial_labels.csv.

Reads:
  ../emg_data/trial_labels.csv
  and each referenced fusion_emg_feat_*.csv

Writes:
  ../emg_data/master_features_labeled.csv
  ../emg_data/master_class_counts.csv
  (optional) split files if split mode is enabled

Usage
-----
python build_master_dataset.py
python build_master_dataset.py --split-mode trial
python build_master_dataset.py --split-mode session --train 0.7 --val 0.15 --test 0.15
"""

import argparse
import csv
import os
import random
from collections import Counter, defaultdict


DEFAULT_DATA_DIR = "../emg_data"
DEFAULT_LABELS = "trial_labels.csv"
DEFAULT_OUT = "master_features_labeled.csv"
DEFAULT_COUNTS = "master_class_counts.csv"


def parse_args():
    p = argparse.ArgumentParser(description="Build labeled master dataset from per-trial feature files.")
    p.add_argument("--data-dir", default=DEFAULT_DATA_DIR, help="Data directory path")
    p.add_argument("--labels", default=DEFAULT_LABELS, help="Labels CSV filename in data-dir")
    p.add_argument("--out", default=DEFAULT_OUT, help="Master output CSV filename in data-dir")
    p.add_argument("--counts-out", default=DEFAULT_COUNTS, help="Class counts CSV filename in data-dir")
    p.add_argument("--split-mode", choices=["none", "trial", "session"], default="none",
                   help="Create split assignment by trial or by session")
    p.add_argument("--train", type=float, default=0.7, help="Train fraction")
    p.add_argument("--val", type=float, default=0.15, help="Validation fraction")
    p.add_argument("--test", type=float, default=0.15, help="Test fraction")
    p.add_argument("--seed", type=int, default=42, help="Random seed for split shuffling")
    return p.parse_args()


def check_split(train, val, test):
    s = train + val + test
    if abs(s - 1.0) > 1e-9:
        raise ValueError(f"train+val+test must equal 1.0 (got {s})")


def read_csv_dict(path):
    with open(path, "r", newline="") as f:
        r = csv.DictReader(f)
        if r.fieldnames is None:
            raise ValueError(f"CSV has no header: {path}")
        return r.fieldnames, list(r)


def assign_splits(group_keys, train, val, test, seed):
    rng = random.Random(seed)
    keys = list(group_keys)
    rng.shuffle(keys)

    n = len(keys)
    n_train = int(round(train * n))
    n_val = int(round(val * n))

    # Keep counts bounded
    n_train = min(max(n_train, 0), n)
    n_val = min(max(n_val, 0), n - n_train)
    n_test = n - n_train - n_val

    out = {}
    for k in keys[:n_train]:
        out[k] = "train"
    for k in keys[n_train:n_train + n_val]:
        out[k] = "val"
    for k in keys[n_train + n_val:]:
        out[k] = "test"

    # Safety check
    if len(out) != n or (n_train + n_val + n_test) != n:
        raise RuntimeError("Internal split assignment error")

    return out


def main():
    args = parse_args()
    check_split(args.train, args.val, args.test)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.normpath(os.path.join(script_dir, args.data_dir))
    labels_path = os.path.join(data_dir, args.labels)
    out_path = os.path.join(data_dir, args.out)
    counts_path = os.path.join(data_dir, args.counts_out)

    if not os.path.exists(labels_path):
        raise FileNotFoundError(f"Labels file not found: {labels_path}")

    label_header, label_rows = read_csv_dict(labels_path)
    if not label_rows:
        raise ValueError("Labels file has no rows")

    required_label_cols = [
        "timestamp", "gesture_label", "trial_id", "session_id", "emg_feat_file"
    ]
    missing_label_cols = [c for c in required_label_cols if c not in label_header]
    if missing_label_cols:
        raise ValueError(f"Labels file missing columns: {', '.join(missing_label_cols)}")

    # Read and concatenate per-trial features with metadata
    master_rows = []
    feature_header = None
    missing_feat_files = []

    for lr in label_rows:
        feat_file = lr.get("emg_feat_file", "").strip()
        if not feat_file:
            continue

        feat_path = os.path.join(data_dir, feat_file)
        if not os.path.exists(feat_path):
            missing_feat_files.append(feat_file)
            continue

        feat_header_cur, feat_rows = read_csv_dict(feat_path)
        if feature_header is None:
            feature_header = feat_header_cur
        elif feat_header_cur != feature_header:
            raise ValueError(
                f"Feature header mismatch in file {feat_file}.\n"
                f"Expected: {feature_header}\n"
                f"Got: {feat_header_cur}"
            )

        for fr in feat_rows:
            out_row = {
                "timestamp": lr.get("timestamp", ""),
                "gesture_label": lr.get("gesture_label", ""),
                "trial_id": lr.get("trial_id", ""),
                "session_id": lr.get("session_id", ""),
                "subject_id": lr.get("subject_id", ""),
                "notes": lr.get("notes", ""),
                "source_feat_file": feat_file,
            }
            for c in feature_header:
                out_row[c] = fr.get(c, "")
            master_rows.append(out_row)

    if missing_feat_files:
        print("WARNING: Missing feature files listed in labels:")
        for f in missing_feat_files:
            print(f"  - {f}")

    if not master_rows:
        raise ValueError("No master rows produced. Check labels and feature files.")

    # Optional split assignment
    split_assign = {}
    if args.split_mode != "none":
        if args.split_mode == "trial":
            group_keys = sorted({r["timestamp"] for r in master_rows})
            split_assign = assign_splits(group_keys, args.train, args.val, args.test, args.seed)
            for r in master_rows:
                r["split"] = split_assign[r["timestamp"]]
        else:  # session
            group_keys = sorted({r["session_id"] for r in master_rows})
            split_assign = assign_splits(group_keys, args.train, args.val, args.test, args.seed)
            for r in master_rows:
                r["split"] = split_assign[r["session_id"]]

    # Output header
    out_header = [
        "timestamp", "gesture_label", "trial_id", "session_id",
        "subject_id", "notes", "source_feat_file"
    ]
    if args.split_mode != "none":
        out_header.append("split")
    out_header += feature_header

    # Write master CSV
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=out_header)
        w.writeheader()
        w.writerows(master_rows)

    # Class counts
    class_counter = Counter(r["gesture_label"] for r in master_rows)
    with open(counts_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["gesture_label", "rows"]) 
        for g, n in sorted(class_counter.items()):
            w.writerow([g, n])

    # Optional split files and split summary
    if args.split_mode != "none":
        split_files = {
            "train": out_path.replace(".csv", "_train.csv"),
            "val": out_path.replace(".csv", "_val.csv"),
            "test": out_path.replace(".csv", "_test.csv"),
        }
        split_rows = defaultdict(list)
        for r in master_rows:
            split_rows[r["split"]].append(r)

        for split_name, path in split_files.items():
            with open(path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=out_header)
                w.writeheader()
                w.writerows(split_rows.get(split_name, []))

        print("Split groups:")
        for k, v in sorted(split_assign.items()):
            print(f"  {k} -> {v}")

        print("Split row counts:")
        for split_name in ["train", "val", "test"]:
            print(f"  {split_name}: {len(split_rows.get(split_name, []))}")

    # Console summary
    print(f"Labels rows      : {len(label_rows)}")
    print(f"Master rows      : {len(master_rows)}")
    print(f"Unique trials    : {len(set(r['timestamp'] for r in master_rows))}")
    print(f"Unique sessions  : {len(set(r['session_id'] for r in master_rows))}")
    print(f"Master CSV       : {out_path}")
    print(f"Class counts CSV : {counts_path}")
    print("Class row counts:")
    for g, n in sorted(class_counter.items()):
        print(f"  {g}: {n}")


if __name__ == "__main__":
    main()
