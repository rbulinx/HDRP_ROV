import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_rows(path):
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def main():
    if len(sys.argv) != 2:
        raise SystemExit("Usage: python tools/plot_wave_parameter_sweep.py <sweep_csv>")

    csv_path = Path(sys.argv[1])
    out_dir = Path.cwd() / "generated_graphs"
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = read_rows(csv_path)
    grouped = defaultdict(list)
    for row in rows:
        grouped[row["field_name"]].append(row)

    for values in grouped.values():
        values.sort(key=lambda r: float(r["multiplier"]))

    stem = csv_path.stem

    # Combined figure
    fig, axes = plt.subplots(3, 1, figsize=(10.8, 11.0), dpi=170, sharex=True)
    colors = ["#0b6e99", "#cc6f00", "#2f8f2f", "#b33c86", "#6c5ce7", "#7f8c8d", "#c0392b"]

    for idx, (field, values) in enumerate(grouped.items()):
        x = [float(r["multiplier"]) for r in values]
        hs13 = [float(r["Hs_top_third_m"]) for r in values]
        fp = [float(r["dominant_frequency_hz"]) for r in values]
        tp = [float(r["dominant_period_s"]) for r in values]
        color = colors[idx % len(colors)]

        axes[0].plot(x, hs13, marker="o", linewidth=1.8, color=color, label=field)
        axes[1].plot(x, fp, marker="o", linewidth=1.8, color=color, label=field)
        axes[2].plot(x, tp, marker="o", linewidth=1.8, color=color, label=field)

    axes[0].set_title("Sensitivity of H1/3 to WaterSurface Parameters")
    axes[0].set_ylabel("H1/3 [m]")
    axes[1].set_title("Sensitivity of Dominant Frequency")
    axes[1].set_ylabel("Dominant Frequency [Hz]")
    axes[2].set_title("Sensitivity of Dominant Period")
    axes[2].set_ylabel("Dominant Period [s]")
    axes[2].set_xlabel("Parameter Multiplier [-]")

    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(frameon=False, fontsize=8, ncol=2)

    fig.tight_layout()
    combined_path = out_dir / f"{stem}_summary.png"
    fig.savefig(combined_path)
    plt.close(fig)

    # Per-parameter figure
    for field, values in grouped.items():
        x = [float(r["multiplier"]) for r in values]
        hs13 = [float(r["Hs_top_third_m"]) for r in values]
        fp = [float(r["dominant_frequency_hz"]) for r in values]
        tp = [float(r["dominant_period_s"]) for r in values]

        fig, axes = plt.subplots(1, 3, figsize=(13.5, 3.9), dpi=170)

        axes[0].plot(x, hs13, marker="o", linewidth=1.8, color="#0b6e99")
        axes[0].set_title(f"{field}: H1/3")
        axes[0].set_xlabel("Multiplier [-]")
        axes[0].set_ylabel("H1/3 [m]")
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(x, fp, marker="o", linewidth=1.8, color="#cc6f00")
        axes[1].set_title(f"{field}: Dominant Frequency")
        axes[1].set_xlabel("Multiplier [-]")
        axes[1].set_ylabel("Frequency [Hz]")
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(x, tp, marker="o", linewidth=1.8, color="#2f8f2f")
        axes[2].set_title(f"{field}: Dominant Period")
        axes[2].set_xlabel("Multiplier [-]")
        axes[2].set_ylabel("Period [s]")
        axes[2].grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(out_dir / f"{stem}_{field}.png")
        plt.close(fig)

    print(combined_path)
    for field in grouped.keys():
        print(out_dir / f"{stem}_{field}.png")


if __name__ == "__main__":
    main()
