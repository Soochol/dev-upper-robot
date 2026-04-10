#!/usr/bin/env python3
"""
visualize.py — SD card CSV viewer + interval labeling tool.

Usage:
    python visualize.py ml_000.csv

Displays FSR, tilt, accel/gyro time series. Click-drag on the plot to
mark FORCE_UP (green) or FORCE_DOWN (red) intervals. Press 'u' for UP
mode, 'd' for DOWN mode, 'z' to undo last label.

Saves labeled output as: ml_000_labeled.csv
with an extra 'label' column (0=FORCE_DOWN, 1=FORCE_UP).
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import SpanSelector

def load_csv(path):
    """Load MCU CSV. Tilt columns are x100 fixed-point."""
    df = pd.read_csv(path)
    if 'tx100' in df.columns:
        df['tx'] = df['tx100'] / 100.0
        df['ty'] = df['ty100'] / 100.0
        df.drop(columns=['tx100', 'ty100'], inplace=True)
    df['t_sec'] = (df['ms'] - df['ms'].iloc[0]) / 1000.0
    return df

def main():
    if len(sys.argv) < 2:
        print("Usage: python visualize.py <csv_file>")
        sys.exit(1)

    path = sys.argv[1]
    df = load_csv(path)
    df['label'] = -1  # unlabeled

    label_mode = {'value': 1}  # 1=FORCE_UP, 0=FORCE_DOWN
    history = []

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    t = df['t_sec'].values

    # Plot 1: FSR
    axes[0].plot(t, df['fsr'], 'b-', linewidth=0.5)
    axes[0].set_ylabel('FSR (raw)')
    axes[0].set_title(f'{path} — press u=UP mode, d=DOWN mode, z=undo')

    # Plot 2: Tilt
    axes[1].plot(t, df['tx'], 'r-', linewidth=0.5, label='tilt_x')
    axes[1].plot(t, df['ty'], 'g-', linewidth=0.5, label='tilt_y')
    axes[1].set_ylabel('Tilt (deg)')
    axes[1].legend(loc='upper right', fontsize=8)

    # Plot 3: Accel
    axes[2].plot(t, df['ax'], linewidth=0.5, label='ax')
    axes[2].plot(t, df['ay'], linewidth=0.5, label='ay')
    axes[2].plot(t, df['az'], linewidth=0.5, label='az')
    axes[2].set_ylabel('Accel (raw)')
    axes[2].legend(loc='upper right', fontsize=8)

    # Plot 4: Gyro
    axes[3].plot(t, df['gx'], linewidth=0.5, label='gx')
    axes[3].plot(t, df['gy'], linewidth=0.5, label='gy')
    axes[3].plot(t, df['gz'], linewidth=0.5, label='gz')
    axes[3].set_ylabel('Gyro (raw)')
    axes[3].set_xlabel('Time (s)')
    axes[3].legend(loc='upper right', fontsize=8)

    label_spans = []  # track drawn spans for undo

    def on_select(tmin, tmax):
        mask = (df['t_sec'] >= tmin) & (df['t_sec'] <= tmax)
        lbl = label_mode['value']
        df.loc[mask, 'label'] = lbl
        color = 'green' if lbl == 1 else 'red'
        alpha = 0.2
        spans = []
        for ax in axes:
            s = ax.axvspan(tmin, tmax, alpha=alpha, color=color)
            spans.append(s)
        label_spans.append(spans)
        count_up = (df['label'] == 1).sum()
        count_down = (df['label'] == 0).sum()
        fig.suptitle(f'Mode: {"UP" if lbl == 1 else "DOWN"} | '
                     f'UP: {count_up} | DOWN: {count_down} | '
                     f'Unlabeled: {(df["label"] == -1).sum()}',
                     fontsize=10)
        fig.canvas.draw_idle()

    def on_key(event):
        if event.key == 'u':
            label_mode['value'] = 1
            fig.suptitle('Mode: UP (FORCE_UP)', fontsize=10)
            fig.canvas.draw_idle()
        elif event.key == 'd':
            label_mode['value'] = 0
            fig.suptitle('Mode: DOWN (FORCE_DOWN)', fontsize=10)
            fig.canvas.draw_idle()
        elif event.key == 'z' and label_spans:
            # Undo last label
            spans = label_spans.pop()
            for s in spans:
                s.remove()
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect('key_press_event', on_key)

    span = SpanSelector(axes[0], on_select, 'horizontal',
                        useblit=True,
                        props=dict(alpha=0.3, facecolor='yellow'))

    plt.tight_layout()
    plt.show()

    # Save labeled data
    labeled = df[df['label'] >= 0].copy()
    if len(labeled) == 0:
        print("No labels assigned. Nothing saved.")
        return

    out_path = path.replace('.csv', '_labeled.csv')
    labeled.to_csv(out_path, index=False)
    print(f"Saved {len(labeled)} labeled rows to {out_path}")
    print(f"  FORCE_UP:   {(labeled['label'] == 1).sum()}")
    print(f"  FORCE_DOWN: {(labeled['label'] == 0).sum()}")

if __name__ == '__main__':
    main()
