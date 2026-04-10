#!/usr/bin/env python3
"""
workflow.py — ML training pipeline orchestrator.

Subcommands:
    sd-copy     Copy ml_*.csv from mounted SD card to tools/ml/data/
    label       Interactive labeling (launches visualize.py)
    auto-label  Rule-based automatic labeling
    train       Train model from all labeled data → Core/Src/app/model_rf.c
    validate    Feature skew validation
    full        Full pipeline: sd-copy → label → train → validate
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
DATA_DIR = SCRIPT_DIR / 'data'
MODEL_OUTPUT = PROJECT_ROOT / 'Core' / 'Src' / 'app' / 'model_rf.c'


def find_sd_mount():
    """Scan /media/$USER/ for a FAT partition containing ml_*.csv."""
    user = os.environ.get('USER', 'root')
    media_base = Path(f'/media/{user}')
    if not media_base.exists():
        return None, []

    for mount_point in media_base.iterdir():
        if not mount_point.is_dir():
            continue
        csvs = sorted(mount_point.glob('ml_*.csv'))
        if csvs:
            return mount_point, csvs
    return None, []


def cmd_sd_copy(args):
    """Copy ml_*.csv files from SD card to data directory."""
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    if args.sd_path:
        sd = Path(args.sd_path)
        csvs = sorted(sd.glob('ml_*.csv'))
        if not csvs:
            print(f"ERROR: No ml_*.csv files found in {sd}")
            return 1
    else:
        sd, csvs = find_sd_mount()
        if sd is None:
            print("ERROR: No SD card found in /media/$USER/")
            print("  - Insert SD card into USB card reader")
            print("  - Or specify path: workflow.py sd-copy --sd-path /path/to/sd")
            return 1

    print(f"SD card found: {sd}")
    print(f"Files: {len(csvs)}")

    copied = []
    for csv_path in csvs:
        dest = DATA_DIR / csv_path.name
        if dest.exists():
            print(f"  SKIP (exists): {csv_path.name}")
            continue
        shutil.copy2(csv_path, dest)
        # Quick stats
        with open(dest) as f:
            lines = sum(1 for _ in f) - 1  # subtract header
        print(f"  COPIED: {csv_path.name} ({lines} rows)")
        copied.append(dest)

    if not copied:
        print("No new files to copy.")
    else:
        print(f"\nCopied {len(copied)} file(s) to {DATA_DIR}")
    return 0


def cmd_label(args):
    """Launch visualize.py for interactive labeling."""
    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"ERROR: File not found: {csv_path}")
        return 1

    print(f"Launching interactive labeler for {csv_path.name}")
    print("  - Click-drag on FSR plot to select intervals")
    print("  - Press 'u' for FORCE_UP, 'd' for FORCE_DOWN, 'z' to undo")
    print("  - Close the window when done")

    result = subprocess.run(
        [sys.executable, str(SCRIPT_DIR / 'visualize.py'), str(csv_path)])
    return result.returncode


def cmd_auto_label(args):
    """Apply rule-based labels matching trigger_rule.c logic."""
    import pandas as pd

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"ERROR: File not found: {csv_path}")
        return 1

    df = pd.read_csv(csv_path)
    if 'tx100' in df.columns:
        df['tx'] = df['tx100'] / 100.0
        df['ty'] = df['ty100'] / 100.0
        df.drop(columns=['tx100', 'ty100'], inplace=True)

    # Rule-based labeling (mirrors trigger_rule.c hysteresis logic)
    fsr_up = args.fsr_up
    fsr_down = args.fsr_down
    tilt_up = args.tilt_up
    tilt_down = args.tilt_down
    debounce_rows = int(args.debounce_ms / 50)  # 50ms per row at 20Hz

    labels = []
    state = 0  # 0=FORCE_DOWN, 1=FORCE_UP
    last_transition = -debounce_rows

    for i, row in df.iterrows():
        fsr = row['fsr']
        tilt_x = row.get('tx', 0)

        if state == 0:  # Currently DOWN
            if (i - last_transition) > debounce_rows:
                if fsr > fsr_up or abs(tilt_x) > tilt_up:
                    state = 1
                    last_transition = i
        else:  # Currently UP
            if (i - last_transition) > debounce_rows:
                if fsr < fsr_down and abs(tilt_x) < tilt_down:
                    state = 0
                    last_transition = i
        labels.append(state)

    df['label'] = labels

    out_path = csv_path.with_name(csv_path.stem + '_labeled.csv')
    df.to_csv(out_path, index=False)

    n_up = sum(1 for l in labels if l == 1)
    n_down = sum(1 for l in labels if l == 0)
    print(f"Auto-labeled {csv_path.name}:")
    print(f"  FORCE_UP:   {n_up}")
    print(f"  FORCE_DOWN: {n_down}")
    print(f"  Output: {out_path}")

    if n_up < 50 or n_down < 50:
        print("  WARNING: Very few samples in one class. Consider manual labeling.")

    return 0


def cmd_train(args):
    """Train model from labeled data."""
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    labeled_files = sorted(DATA_DIR.glob('*_labeled.csv'))
    if not labeled_files:
        print("ERROR: No labeled CSV files found in tools/ml/data/")
        print("  Run: workflow.py label <csv> or workflow.py auto-label <csv>")
        return 1

    print(f"Training from {len(labeled_files)} labeled file(s):")
    for f in labeled_files:
        print(f"  {f.name}")

    cmd = [sys.executable, str(SCRIPT_DIR / 'train.py'),
           '--output', str(MODEL_OUTPUT)] + [str(f) for f in labeled_files]

    result = subprocess.run(cmd)
    if result.returncode != 0:
        print("ERROR: Training failed")
        return 1

    if MODEL_OUTPUT.exists():
        print(f"\nModel written to {MODEL_OUTPUT}")
    return 0


def cmd_validate(args):
    """Run feature skew validation."""
    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"ERROR: File not found: {csv_path}")
        return 1

    cmd = [sys.executable, str(SCRIPT_DIR / 'validate.py'), str(csv_path)]
    if args.mcu_features:
        cmd.append(args.mcu_features)

    result = subprocess.run(cmd)
    return result.returncode


def cmd_full(args):
    """Full pipeline: sd-copy → label → train → validate."""
    print("=" * 60)
    print("ML WORKFLOW — Full Pipeline")
    print("=" * 60)

    # Step 1: Copy from SD
    print("\n[1/4] Copying data from SD card...")
    rc = cmd_sd_copy(args)
    if rc != 0:
        return rc

    # Step 2: Label
    unlabeled = []
    for f in sorted(DATA_DIR.glob('ml_*.csv')):
        labeled = f.with_name(f.stem + '_labeled.csv')
        if not labeled.exists():
            unlabeled.append(f)

    if not unlabeled:
        print("\n[2/4] All files already labeled — skipping.")
    else:
        print(f"\n[2/4] Labeling {len(unlabeled)} new file(s)...")
        for csv in unlabeled:
            if args.auto_label:
                args_copy = argparse.Namespace(
                    csv=str(csv),
                    fsr_up=args.fsr_up, fsr_down=args.fsr_down,
                    tilt_up=args.tilt_up, tilt_down=args.tilt_down,
                    debounce_ms=args.debounce_ms)
                rc = cmd_auto_label(args_copy)
            else:
                args_copy = argparse.Namespace(csv=str(csv))
                rc = cmd_label(args_copy)
            if rc != 0:
                return rc

    # Step 3: Train
    print("\n[3/4] Training model...")
    rc = cmd_train(argparse.Namespace())
    if rc != 0:
        return rc

    # Step 4: Validate (use first raw CSV)
    raw_files = sorted(DATA_DIR.glob('ml_*.csv'))
    raw_files = [f for f in raw_files if '_labeled' not in f.name]
    if raw_files:
        print("\n[4/4] Validating features...")
        rc = cmd_validate(argparse.Namespace(csv=str(raw_files[0]),
                                             mcu_features=None))
    else:
        print("\n[4/4] No raw CSV for validation — skipping.")

    print("\n" + "=" * 60)
    print("Pipeline complete.")
    print(f"Model: {MODEL_OUTPUT}")
    print("To deploy: cmake --preset Debug -DML_TRIGGER=ON && build + flash")
    print("=" * 60)
    return 0


def main():
    parser = argparse.ArgumentParser(
        description='ML training pipeline orchestrator')
    sub = parser.add_subparsers(dest='command', required=True)

    # sd-copy
    p = sub.add_parser('sd-copy', help='Copy CSV files from SD card')
    p.add_argument('--sd-path', help='SD card mount point (auto-detect if omitted)')

    # label
    p = sub.add_parser('label', help='Interactive labeling')
    p.add_argument('csv', help='Raw CSV file to label')

    # auto-label
    p = sub.add_parser('auto-label', help='Rule-based automatic labeling')
    p.add_argument('csv', help='Raw CSV file to label')
    p.add_argument('--fsr-up', type=int, default=2000,
                   help='FSR threshold for FORCE_UP (default: 2000)')
    p.add_argument('--fsr-down', type=int, default=500,
                   help='FSR threshold for FORCE_DOWN (default: 500)')
    p.add_argument('--tilt-up', type=float, default=45.0,
                   help='Tilt threshold for FORCE_UP in degrees (default: 45)')
    p.add_argument('--tilt-down', type=float, default=10.0,
                   help='Tilt threshold for FORCE_DOWN in degrees (default: 10)')
    p.add_argument('--debounce-ms', type=int, default=500,
                   help='Debounce period in ms (default: 500)')

    # train
    sub.add_parser('train', help='Train from all labeled data')

    # validate
    p = sub.add_parser('validate', help='Feature skew validation')
    p.add_argument('csv', help='Raw CSV file')
    p.add_argument('mcu_features', nargs='?', help='MCU feature CSV (optional)')

    # full
    p = sub.add_parser('full', help='Full pipeline: sd-copy → label → train → validate')
    p.add_argument('--sd-path', help='SD card mount point')
    p.add_argument('--auto-label', action='store_true',
                   help='Use rule-based auto-labeling instead of interactive')
    p.add_argument('--fsr-up', type=int, default=2000)
    p.add_argument('--fsr-down', type=int, default=500)
    p.add_argument('--tilt-up', type=float, default=45.0)
    p.add_argument('--tilt-down', type=float, default=10.0)
    p.add_argument('--debounce-ms', type=int, default=500)

    args = parser.parse_args()
    sys.exit(globals()[f'cmd_{args.command.replace("-", "_")}'](args))


if __name__ == '__main__':
    main()
