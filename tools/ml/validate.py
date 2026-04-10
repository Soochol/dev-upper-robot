#!/usr/bin/env python3
"""
validate.py — Detect training-serving skew between Python and C features.

Usage:
    python validate.py <raw_csv>

Reads a raw (unlabeled) CSV from the SD card, computes features using
the Python implementation (train.py:compute_features), and outputs the
feature matrix so it can be compared against the MCU's RTT feature log.

For automated comparison:
    1. Enable ML feature RTT logging on the MCU (add to t_ml.c heartbeat)
    2. Capture RTT output to a file
    3. Run this script on the same SD card CSV
    4. Diff the two outputs — differences indicate skew

Tolerance: features should match within 0.1% relative error. Differences
beyond this indicate a bug in the C sliding window implementation.
"""

import sys
import numpy as np
import pandas as pd

# Import feature computation from train.py
from train import compute_features, FEATURE_NAMES, DEFAULT_WINDOW_SIZE


def main():
    if len(sys.argv) < 2:
        print("Usage: python validate.py <raw_csv> [mcu_features_csv]")
        sys.exit(1)

    path = sys.argv[1]
    df = pd.read_csv(path)
    if 'tx100' in df.columns:
        df['tx'] = df['tx100'] / 100.0
        df['ty'] = df['ty100'] / 100.0
        df.drop(columns=['tx100', 'ty100'], inplace=True)

    print(f"Loaded {path}: {len(df)} rows")
    print(f"Computing features (window={DEFAULT_WINDOW_SIZE})...")

    features = compute_features(df, DEFAULT_WINDOW_SIZE)

    # Output features for the first 10 rows after window fill
    start = DEFAULT_WINDOW_SIZE - 1
    print(f"\n--- Python features (rows {start}-{start+9}) ---")
    print(f"{'row':>5s}", end='')
    for name in FEATURE_NAMES:
        print(f"  {name:>14s}", end='')
    print()

    for i in range(start, min(start + 10, len(features))):
        print(f"{i:5d}", end='')
        for j in range(9):
            print(f"  {features[i, j]:14.4f}", end='')
        print()

    # If MCU features CSV is provided, compare
    if len(sys.argv) >= 3:
        mcu_path = sys.argv[2]
        mcu_df = pd.read_csv(mcu_path)
        print(f"\n--- Comparing with MCU features: {mcu_path} ---")

        mcu_feat = mcu_df[FEATURE_NAMES].values
        py_feat = features[start:start + len(mcu_feat)]

        if len(py_feat) != len(mcu_feat):
            print(f"WARNING: row count mismatch: Python={len(py_feat)}, "
                  f"MCU={len(mcu_feat)}")
            n = min(len(py_feat), len(mcu_feat))
            py_feat = py_feat[:n]
            mcu_feat = mcu_feat[:n]

        # Relative error per feature
        denom = np.maximum(np.abs(py_feat), 1e-6)
        rel_err = np.abs(py_feat - mcu_feat) / denom

        print(f"\nMax relative error per feature:")
        for j, name in enumerate(FEATURE_NAMES):
            max_err = rel_err[:, j].max()
            status = "OK" if max_err < 0.001 else "SKEW!"
            print(f"  {name:20s}: {max_err:.6f}  [{status}]")

        total_max = rel_err.max()
        if total_max < 0.001:
            print(f"\nRESULT: All features within 0.1% — no skew detected.")
        else:
            print(f"\nRESULT: SKEW DETECTED (max relative error: {total_max:.6f})")
            print("Check the C implementation in features.c against train.py")

    # Save full feature matrix for manual inspection
    out_path = path.replace('.csv', '_features.csv')
    feat_df = pd.DataFrame(features, columns=FEATURE_NAMES)
    feat_df.to_csv(out_path, index=False)
    print(f"\nFull feature matrix saved to {out_path}")


if __name__ == '__main__':
    main()
