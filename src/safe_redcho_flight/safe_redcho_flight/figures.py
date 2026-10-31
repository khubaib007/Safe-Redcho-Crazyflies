"""All figures for an offline or physical mission, from saved NPZ data."""
import argparse
from pathlib import Path
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    data = np.load(args.trace)
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    t = data['time']
    key = 'measured_position' if 'measured_position' in data else 'synthetic_position'
    p, reference = data[key], data['reference_position']
    fig, ax = plt.subplots(figsize=(7, 6))
    for i in range(6):
        ax.plot(p[:, i, 0], p[:, i, 1], label=f'cf{i+1}')
        ax.scatter(p[0, i, 0], p[0, i, 1], marker='o')
    ax.set(xlabel='World x [m]', ylabel='World y [m]', title='One-metre formation translation')
    ax.axis('equal'); ax.grid(); ax.legend(); fig.tight_layout(); fig.savefig(out/'01_paths.png', dpi=160); plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for i in range(6):
        axes[0].plot(t, p[:, i, 2], label=f'cf{i+1}')
        axes[1].plot(t, np.linalg.norm(p[:, i]-reference[:, i], axis=1))
        axes[2].plot(t, data['velocity_redcho'][:, i, 0, 0])
    for ax in axes: ax.grid()
    axes[0].set_ylabel('Height [m]'); axes[0].legend(ncol=3)
    axes[1].set_ylabel('Tracking error [m]')
    axes[2].set(ylabel='REDCHO vx [m/s]', xlabel='Logical mission time [s]')
    fig.tight_layout(); fig.savefig(out/'02_consensus_tracking.png', dpi=160); plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    pair = np.stack([np.linalg.norm(p[:, i, :2]-p[:, j, :2], axis=1) for i in range(6) for j in range(i)])
    axes[0].plot(t, pair.min(axis=0)); axes[0].set_ylabel('Min measured XY pair [m]')
    for i in range(6): axes[1].plot(t, data['pose_redcho'][:, i, 0, 0])
    axes[1].set(ylabel='REDCHO displacement x [m]', xlabel='Logical mission time [s]')
    for ax in axes: ax.grid()
    fig.tight_layout(); fig.savefig(out/'03_safety_position_consensus.png', dpi=160); plt.close(fig)
    print(out.resolve())
