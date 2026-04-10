#!/usr/bin/env python3
"""
auto_label.py — FSR slope + IMU tilt 기반 자동 라벨링.

Usage:
    python auto_label.py <raw_csv>                    # FSR slope만 사용
    python auto_label.py <raw_csv> --use-tilt         # FSR slope + tilt 병합
    python auto_label.py <raw_csv> --visualize        # matplotlib로 결과 시각화

출력: <raw_csv> 옆에 <raw_csv>_auto_labeled.csv 생성.

라벨링 전략:
  Step 1 (FSR slope):
    - FSR slope의 부호로 상승/하강 구간을 식별
    - 양의 slope 구간 → FORCE_UP (1)
    - 음의 slope 구간 → FORCE_DOWN (0)
    - 전환 구간(slope ≈ 0) → unlabeled (-1)

  Step 2 (--use-tilt, optional):
    - tilt_x의 크기로 FSR-only 라벨을 보정
    - tilt_x가 큰데 FSR가 DOWN이면 → 신뢰도 낮음, -1로 변환
    - tilt_x가 작은데 FSR가 UP이면 → 신뢰도 낮음, -1로 변환
"""

import sys
import argparse
import numpy as np
import pandas as pd


def compute_fsr_slope(fsr, window=20):
    """Compute least-squares slope over a sliding window."""
    n = len(fsr)
    slope = np.zeros(n)
    for i in range(window - 1, n):
        w = fsr[i - window + 1:i + 1].astype(float)
        x = np.arange(len(w), dtype=float)
        wn = len(w)
        sum_x = x.sum()
        sum_y = w.sum()
        sum_xy = (x * w).sum()
        sum_x2 = (x * x).sum()
        denom = wn * sum_x2 - sum_x * sum_x
        if abs(denom) < 1e-6:
            slope[i] = 0.0
        else:
            slope[i] = (wn * sum_xy - sum_x * sum_y) / denom
    return slope


def label_by_fsr_slope(fsr_slope, fsr, slope_thresh=100.0, fsr_min=500,
                       idle_fsr_max=6000, idle_slope_max=50.0,
                       plateau_fsr_min=12000):
    """State-machine labeling based on FSR pattern.

    FSR lifecycle:
      1. Steep rise (slope > thresh)        → FORCE_UP start
      2. High plateau (fsr > plateau_min)   → FORCE_UP hold
      3. Steep fall (slope < -thresh)        → FORCE_DOWN
      4. Low flat (fsr < idle_max, flat)     → idle (FORCE_DOWN)

    Args:
        fsr_slope: per-sample FSR slope array
        fsr: raw FSR values
        slope_thresh: minimum |slope| for rise/fall detection
        fsr_min: minimum FSR for any labeling
        idle_fsr_max: FSR below this + flat → idle
        idle_slope_max: |slope| below this → flat
        plateau_fsr_min: FSR above this + not falling → plateau (UP)

    Returns:
        labels: array of -1 (unlabeled), 0 (FORCE_DOWN), 1 (FORCE_UP)
    """
    n = len(fsr_slope)
    labels = np.full(n, -1, dtype=int)

    for i in range(n):
        slope = fsr_slope[i]
        val = fsr[i]

        # 1. Steep rise → FORCE_UP
        if val >= fsr_min and slope > slope_thresh:
            labels[i] = 1
        # 2. High plateau (not falling) → FORCE_UP
        elif val >= plateau_fsr_min and slope > -slope_thresh:
            labels[i] = 1
        # 3. Steep fall → FORCE_DOWN
        elif val >= fsr_min and slope < -slope_thresh:
            labels[i] = 0
        # 4. Low + flat → idle (FORCE_DOWN)
        elif val < idle_fsr_max and abs(slope) < idle_slope_max:
            labels[i] = 0

    return labels


def refine_with_tilt(labels, tilt_x, tilt_up_thresh=-14.0, tilt_down_thresh=-10.0):
    """Refine FSR-based labels using tilt_x.

    Based on training data analysis:
      FORCE_UP (1):   tilt_x mean = -19.4 (large negative, arm tilted up)
      FORCE_DOWN (0): tilt_x mean = -7.7  (small, arm level)

    If tilt disagrees with FSR label, mark as uncertain (-1).

    Args:
        labels: existing label array
        tilt_x: tilt_x_deg values
        tilt_up_thresh: tilt_x below this → consistent with UP
        tilt_down_thresh: tilt_x above this → consistent with DOWN
    """
    refined = labels.copy()
    conflicts = 0

    for i in range(len(labels)):
        if labels[i] == 1 and tilt_x[i] > tilt_down_thresh:
            # FSR says UP but tilt says DOWN → uncertain
            refined[i] = -1
            conflicts += 1
        elif labels[i] == 0 and tilt_x[i] < tilt_up_thresh:
            # FSR says DOWN but tilt says UP → uncertain
            refined[i] = -1
            conflicts += 1

    return refined, conflicts


def smooth_labels(labels, min_run=5):
    """Remove short label runs (noise). Runs shorter than min_run → -1."""
    smoothed = labels.copy()
    n = len(labels)
    i = 0
    while i < n:
        j = i
        while j < n and labels[j] == labels[i]:
            j += 1
        run_len = j - i
        if labels[i] >= 0 and run_len < min_run:
            smoothed[i:j] = -1
        i = j
    return smoothed


def main():
    parser = argparse.ArgumentParser(
        description='Auto-label raw sensor CSV using FSR slope + optional tilt.')
    parser.add_argument('csv', help='Raw CSV file (from SD card)')
    parser.add_argument('--use-tilt', action='store_true',
                        help='Also use tilt_x to refine labels')
    parser.add_argument('--slope-thresh', type=float, default=100.0,
                        help='Min |FSR slope| to assign label (default: 100)')
    parser.add_argument('--fsr-min', type=int, default=500,
                        help='Min FSR value to consider (default: 500)')
    parser.add_argument('--window', type=int, default=20,
                        help='Slope window size (default: 20, matches ML_WINDOW_SIZE)')
    parser.add_argument('--visualize', action='store_true',
                        help='Show matplotlib visualization')
    args = parser.parse_args()

    df = pd.read_csv(args.csv)
    if 'tx100' in df.columns:
        df['tx'] = df['tx100'] / 100.0
        df['ty'] = df['ty100'] / 100.0
    print(f"Loaded {args.csv}: {len(df)} rows")

    fsr = df['fsr'].values.astype(float)

    # Handle NaN
    nan_mask = np.isnan(fsr)
    if nan_mask.any():
        print(f"  Warning: {nan_mask.sum()} NaN FSR values, filling with 0")
        fsr[nan_mask] = 0.0

    # Step 1: FSR slope labeling
    print(f"\nStep 1: FSR slope labeling (window={args.window}, "
          f"thresh={args.slope_thresh}, fsr_min={args.fsr_min})")
    fsr_slope = compute_fsr_slope(fsr, args.window)
    labels = label_by_fsr_slope(fsr_slope, fsr, args.slope_thresh, args.fsr_min)
    labels = smooth_labels(labels, min_run=5)

    n_up = (labels == 1).sum()
    n_down = (labels == 0).sum()
    n_unlabeled = (labels == -1).sum()
    print(f"  FORCE_UP (1):   {n_up}")
    print(f"  FORCE_DOWN (0): {n_down}")
    print(f"  unlabeled (-1): {n_unlabeled}")

    # Step 2: optional tilt refinement
    if args.use_tilt and 'tx' in df.columns:
        print(f"\nStep 2: Tilt refinement")
        tilt_x = df['tx'].values
        labels, conflicts = refine_with_tilt(labels, tilt_x)
        labels = smooth_labels(labels, min_run=5)
        n_up = (labels == 1).sum()
        n_down = (labels == 0).sum()
        n_unlabeled = (labels == -1).sum()
        print(f"  Conflicts resolved: {conflicts}")
        print(f"  FORCE_UP (1):   {n_up}")
        print(f"  FORCE_DOWN (0): {n_down}")
        print(f"  unlabeled (-1): {n_unlabeled}")

    # Output
    df['label'] = labels
    df['fsr_slope'] = fsr_slope  # include for inspection
    out_path = args.csv.replace('.csv', '_auto_labeled.csv')
    df.to_csv(out_path, index=False)
    print(f"\nSaved to {out_path}")

    # Compare with manual labels if they exist
    manual_path = args.csv.replace('.csv', '_labeled.csv')
    try:
        manual = pd.read_csv(manual_path)
        if 'label' in manual.columns:
            m = manual['label'].values
            a = labels[:len(m)]
            # Only compare where both have labels
            both_labeled = (m >= 0) & (a >= 0)
            if both_labeled.sum() > 0:
                agreement = (m[both_labeled] == a[both_labeled]).mean()
                print(f"\n=== Comparison with manual labels ({manual_path}) ===")
                print(f"  Both labeled: {both_labeled.sum()} samples")
                print(f"  Agreement: {agreement:.1%}")
    except FileNotFoundError:
        pass

    if args.visualize:
        try:
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)

            t = (df['ms'].values - df['ms'].values[0]) / 1000.0

            axes[0].plot(t, fsr, 'b-', linewidth=0.5)
            axes[0].set_ylabel('FSR raw')
            axes[0].set_title('Auto-labeling result')

            axes[1].plot(t, fsr_slope, 'g-', linewidth=0.5)
            axes[1].axhline(y=args.slope_thresh, color='r', linestyle='--', alpha=0.5)
            axes[1].axhline(y=-args.slope_thresh, color='r', linestyle='--', alpha=0.5)
            axes[1].set_ylabel('FSR slope')

            if 'tx' in df.columns:
                axes[2].plot(t, df['tx'].values, 'm-', linewidth=0.5)
                axes[2].set_ylabel('tilt_x (deg)')

            # Color-coded labels
            colors = {-1: 'gray', 0: 'blue', 1: 'red'}
            for lbl, color in colors.items():
                mask = labels == lbl
                axes[3].scatter(t[mask], labels[mask], c=color, s=2,
                                label=f'{lbl}')
            axes[3].set_ylabel('label')
            axes[3].set_xlabel('time (s)')
            axes[3].legend()

            plt.tight_layout()
            plt.savefig(out_path.replace('.csv', '_plot.png'), dpi=150)
            print(f"Plot saved to {out_path.replace('.csv', '_plot.png')}")
            plt.close()
        except ImportError:
            print("matplotlib not available, skipping visualization")


if __name__ == '__main__':
    main()
