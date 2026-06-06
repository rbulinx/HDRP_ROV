import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_rows(path):
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def main():
    if len(sys.argv) < 2:
        raise SystemExit("Usage: python tools/plot_wave_spectrum_comparison_large_text.py <comparison_csv> [output_png]")

    csv_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) >= 3 else csv_path.with_name(csv_path.stem + "_large_text.png")

    rows = read_rows(csv_path)
    freq = [float(r["frequency_hz"]) for r in rows]
    obs = [float(r["observed_norm"]) for r in rows]
    jon = [float(r["jonswap_norm"]) for r in rows]

    peak_idx = max(range(len(obs)), key=lambda i: obs[i])
    fp = freq[peak_idx]

    plt.rcParams.update(
        {
            "font.size": 22,
            "axes.titlesize": 26,
            "axes.labelsize": 24,
            "xtick.labelsize": 20,
            "ytick.labelsize": 20,
            "legend.fontsize": 20,
        }
    )

    fig, ax = plt.subplots(figsize=(12.8, 7.2), dpi=170)
    ax.plot(freq, obs, color="#0b6e99", linewidth=2.8, label="Observed spectrum")
    ax.plot(freq, jon, color="#cc6f00", linewidth=2.8, linestyle="--", label="JONSWAP fit")
    ax.axvline(fp, color="#777777", linewidth=1.6, linestyle=":", label=f"Peak = {fp:.3f} Hz")
    ax.set_title("Observed Spectrum vs. JONSWAP")
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("Normalized spectral energy [-]")
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.08)
    ax.grid(True, alpha=0.28)
    ax.legend(frameon=False, loc="upper right")
    fig.tight_layout()
    fig.savefig(out_path, bbox_inches="tight", pad_inches=0.12)
    plt.close(fig)
    print(out_path)


if __name__ == "__main__":
    main()
