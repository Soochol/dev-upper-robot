#!/usr/bin/env python3
"""
train.py — Train a Random Forest classifier and export to C via m2cgen.

Usage:
    python train.py labeled_session_001.csv [labeled_session_002.csv ...]

Pipeline:
    1. Load labeled CSV(s)
    2. Compute sliding window features (Python — source of truth)
    3. Train sklearn RandomForestClassifier
    4. Evaluate with cross-validation
    5. Export model to C code via m2cgen → model_rf.c

The generated model_rf.c contains a single function:
    void score(double *input, double *output);
that takes 9 features and outputs 2 class probabilities.
Copy this into Core/Src/app/trigger_ml.c to deploy.
"""

import sys
import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import cross_val_score, StratifiedKFold
from sklearn.metrics import classification_report, confusion_matrix
import m2cgen as m2c

# Must match config.h ML_WINDOW_SIZE and features.c feature order.
DEFAULT_WINDOW_SIZE = 20

# Feature names — must match features.h ml_features_t order.
FEATURE_NAMES = [
    'fsr_raw', 'tilt_x_deg', 'tilt_y_deg', 'ax_g',
    'fsr_mean', 'fsr_slope', 'tilt_x_slope', 'ay_g', 'gyro_mag_mean'
]

# Conversion constants — must match config.h.
IMU_ACCEL_SCALE_G = 2.0 / 32768.0
IMU_GYRO_SCALE_DPS = 2000.0 / 32768.0


def compute_features(df, window_size=DEFAULT_WINDOW_SIZE):
    """Compute the 9-element feature vector for each row.

    This is the Python source of truth. The C implementation in features.c
    must produce identical results. validate.py checks this.
    """
    # Instantaneous features
    fsr = df['fsr'].values.astype(float)
    tx = df['tx'].values.astype(float)
    ty = df['ty'].values.astype(float)

    ax = df['ax'].values * IMU_ACCEL_SCALE_G
    ay = df['ay'].values * IMU_ACCEL_SCALE_G

    gx = df['gx'].values * IMU_GYRO_SCALE_DPS
    gy = df['gy'].values * IMU_GYRO_SCALE_DPS
    gz = df['gz'].values * IMU_GYRO_SCALE_DPS
    gyro_mag = np.sqrt(gx**2 + gy**2 + gz**2)

    n = len(df)
    features = np.zeros((n, 9))

    for i in range(n):
        # Window bounds
        start = max(0, i - window_size + 1)
        win_fsr = fsr[start:i+1]
        win_tx = tx[start:i+1]
        win_gmag = gyro_mag[start:i+1]

        # Least-squares slope over window
        def ls_slope(y):
            if len(y) < 2:
                return 0.0
            x = np.arange(len(y), dtype=float)
            n_ = len(y)
            sum_x = x.sum()
            sum_y = y.sum()
            sum_xy = (x * y).sum()
            sum_x2 = (x * x).sum()
            denom = n_ * sum_x2 - sum_x * sum_x
            if abs(denom) < 1e-6:
                return 0.0
            return (n_ * sum_xy - sum_x * sum_y) / denom

        features[i, 0] = fsr[i]
        features[i, 1] = tx[i]
        features[i, 2] = ty[i]
        features[i, 3] = ax[i]              # raw accel X (g)
        features[i, 4] = win_fsr.mean()
        features[i, 5] = ls_slope(win_fsr)
        features[i, 6] = ls_slope(win_tx)
        features[i, 7] = ay[i]              # raw accel Y (g)
        features[i, 8] = win_gmag.mean()

    return features


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description='Train Random Forest and export to C via m2cgen.')
    parser.add_argument('csvs', nargs='+', help='Labeled CSV files')
    parser.add_argument('--output', default=None,
                        help='Output path for model_rf.c '
                             '(default: Core/Src/app/model_rf.c relative to project root)')
    args = parser.parse_args()

    # Resolve default output path relative to project root.
    if args.output is None:
        import pathlib
        project_root = pathlib.Path(__file__).resolve().parents[2]
        args.output = str(project_root / 'Core' / 'Src' / 'app' / 'model_rf.c')

    # Load and concatenate all labeled CSVs
    dfs = []
    for path in args.csvs:
        df = pd.read_csv(path)
        if 'tx100' in df.columns:
            df['tx'] = df['tx100'] / 100.0
            df['ty'] = df['ty100'] / 100.0
            df.drop(columns=['tx100', 'ty100'], inplace=True)
        if 't_sec' not in df.columns:
            df['t_sec'] = (df['ms'] - df['ms'].iloc[0]) / 1000.0
        dfs.append(df)
        print(f"Loaded {path}: {len(df)} rows")

    df = pd.concat(dfs, ignore_index=True)

    # Filter to labeled rows only
    df = df[df['label'] >= 0].copy()
    print(f"\nTotal labeled rows: {len(df)}")
    print(f"  FORCE_UP (1):   {(df['label'] == 1).sum()}")
    print(f"  FORCE_DOWN (0): {(df['label'] == 0).sum()}")

    if len(df) < 100:
        print("WARNING: Very few labeled samples. Consider collecting more data.")

    # Compute features with data augmentation.
    # FSR scaling simulates different grip strengths.
    # Time stretching simulates faster/slower arm movements.
    FSR_SCALES = [0.85, 1.0, 1.1, 1.2]
    TIME_SCALES = [0.8, 0.9, 1.0, 1.1, 1.2]

    print(f"\nComputing features (window={DEFAULT_WINDOW_SIZE})...")
    print(f"  FSR scales: {FSR_SCALES}")
    print(f"  Time scales: {TIME_SCALES}")

    X_all = []
    y_all = []
    raw_cols = ['fsr', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'tx', 'ty']

    for ts in TIME_SCALES:
        if ts == 1.0:
            df_t = df
        else:
            # Time stretch: resample each contiguous labeled segment
            from scipy.interpolate import interp1d
            n_orig = len(df)
            n_new = int(n_orig * ts)
            x_orig = np.arange(n_orig)
            x_new = np.linspace(0, n_orig - 1, n_new)
            df_t = pd.DataFrame()
            for col in raw_cols:
                if col in df.columns:
                    f = interp1d(x_orig, df[col].values.astype(float),
                                 kind='linear', fill_value='extrapolate')
                    df_t[col] = f(x_new)
            # Labels: nearest-neighbor (no interpolation for discrete labels)
            df_t['label'] = df['label'].values[
                np.round(x_new).astype(int).clip(0, n_orig - 1)]
            df_t['ms'] = np.linspace(df['ms'].iloc[0], df['ms'].iloc[-1],
                                     n_new)

        for fs in FSR_SCALES:
            df_aug = df_t.copy()
            if fs != 1.0:
                df_aug['fsr'] = df_aug['fsr'] * fs

            X_aug = compute_features(df_aug, DEFAULT_WINDOW_SIZE)
            y_aug = df_aug['label'].values

            valid = np.arange(DEFAULT_WINDOW_SIZE - 1, len(X_aug))
            X_all.append(X_aug[valid])
            y_all.append(y_aug[valid])

    X = np.vstack(X_all)
    y = np.concatenate(y_all)
    print(f"Samples after augmentation: {len(X)} "
          f"({len(FSR_SCALES)}x FSR × {len(TIME_SCALES)}x time = "
          f"{len(FSR_SCALES) * len(TIME_SCALES)}x)")

    # Cross-validation
    print("\n--- Cross-validation (5-fold) ---")
    clf = RandomForestClassifier(
        n_estimators=5,
        max_depth=6,
        random_state=42,
        n_jobs=-1
    )
    cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=42)
    scores = cross_val_score(clf, X, y, cv=cv, scoring='accuracy')
    print(f"Accuracy: {scores.mean():.3f} +/- {scores.std():.3f}")
    print(f"Per-fold: {scores}")

    # Final training on all data
    print("\n--- Final training ---")
    clf.fit(X, y)
    y_pred = clf.predict(X)
    print(classification_report(y, y_pred,
                                target_names=['FORCE_DOWN', 'FORCE_UP']))
    print("Confusion matrix:")
    print(confusion_matrix(y, y_pred))

    # Feature importance
    print("\n--- Feature importance ---")
    for name, imp in sorted(zip(FEATURE_NAMES, clf.feature_importances_),
                            key=lambda x: -x[1]):
        print(f"  {name:20s}: {imp:.4f}")

    # Export to C
    print("\n--- Exporting to C (m2cgen) ---")
    c_code = m2c.export_to_c(clf)

    # m2cgen emits compound literals like (double[]){1.0, 0.0} inside score().
    # On Cortex-M3 (no FPU), compound literals with automatic storage duration
    # are stack-allocated. 156 such literals × 16 bytes = 2,496 bytes of stack,
    # which overflows the T_ML task stack (2,048 bytes) and causes a HardFault.
    # Fix: replace with static const arrays that the linker places in .rodata
    # (Flash), costing zero stack at runtime.
    c_code = c_code.replace('(double[]){1.0, 0.0}', '_leaf_down')
    c_code = c_code.replace('(double[]){0.0, 1.0}', '_leaf_up')
    # m2cgen may emit 'inf' for unbounded splits — define it for C.
    c_code = c_code.replace('\ninf\n', '\nINFINITY\n')
    needs_inf = 'inf' in c_code
    static_consts = (
        "#include <math.h>\n"
        "#define inf INFINITY\n\n"
        "static const double _leaf_down[2] = {1.0, 0.0};\n"
        "static const double _leaf_up[2]   = {0.0, 1.0};\n\n"
    ) if needs_inf else (
        "static const double _leaf_down[2] = {1.0, 0.0};\n"
        "static const double _leaf_up[2]   = {0.0, 1.0};\n\n"
    )

    out_path = args.output
    with open(out_path, 'w') as f:
        f.write("/* Auto-generated by train.py — do not edit manually.\n")
        f.write(f" * Model: RandomForest(n_estimators={clf.n_estimators}, "
                f"max_depth={clf.max_depth})\n")
        f.write(f" * Samples: {len(X)}, Features: {len(FEATURE_NAMES)}\n")
        f.write(f" * CV accuracy: {scores.mean():.3f}\n")
        f.write(" *\n")
        f.write(" * Input: double[9] — feature vector (see FEATURE_NAMES)\n")
        f.write(" * Output: double[2] — [P(FORCE_DOWN), P(FORCE_UP)]\n")
        f.write(" */\n\n")
        f.write(static_consts)
        f.write(c_code)

    print(f"Exported to {out_path}")

    # Window size experiment hint
    print("\n--- Hint: try different window sizes ---")
    for ws in [10, 15, 20, 25, 30]:
        X_ws = compute_features(df, ws)
        v = np.arange(ws - 1, len(X_ws))
        X_ws = X_ws[v]
        y_ws = df['label'].values[v]
        s = cross_val_score(clf, X_ws, y_ws, cv=cv, scoring='accuracy')
        print(f"  window={ws:2d}: accuracy={s.mean():.3f} +/- {s.std():.3f}")


if __name__ == '__main__':
    main()
