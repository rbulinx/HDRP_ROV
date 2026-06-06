import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_csv_rows(path):
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def compute_hs_top_third(eta):
    mean = sum(eta) / len(eta)
    heights = []
    in_wave = False
    local_min = 0.0
    local_max = 0.0

    for i in range(1, len(eta)):
        prev = eta[i - 1] - mean
        curr = eta[i] - mean

        if (not in_wave) and prev <= 0.0 and curr > 0.0:
            in_wave = True
            local_min = curr
            local_max = curr
            continue

        if not in_wave:
            continue

        local_min = min(local_min, curr)
        local_max = max(local_max, curr)

        if prev <= 0.0 and curr > 0.0:
            heights.append(local_max - local_min)
            local_min = curr
            local_max = curr

    if not heights:
        return 0.0

    heights.sort(reverse=True)
    top_count = max(1, len(heights) // 3)
    return sum(heights[:top_count]) / top_count


def build_jonswap(freq, fp, hs, gamma=3.3):
    g = 9.81
    shape = []

    for f in freq:
        if f <= 0.0:
            shape.append(0.0)
            continue

        sigma = 0.07 if f <= fp else 0.09
        r = math.exp(-((f - fp) ** 2) / (2.0 * sigma * sigma * fp * fp))
        base = (g * g) * ((2.0 * math.pi) ** -4.0) * (f ** -5.0) * math.exp(-1.25 * (fp / f) ** 4.0)
        shape.append(base * (gamma ** r))

    m0_target = (hs / 4.0) ** 2
    m0_shape = 0.0
    for i in range(1, len(freq)):
        df = freq[i] - freq[i - 1]
        m0_shape += 0.5 * (shape[i] + shape[i - 1]) * df

    scale = m0_target / m0_shape if m0_shape > 0 else 1.0
    return [scale * x for x in shape]


def main():
    if len(sys.argv) != 3:
        raise SystemExit("Usage: python tools/plot_wave_observation.py <observation_csv> <spectrum_csv>")

    obs_path = Path(sys.argv[1])
    spec_path = Path(sys.argv[2])
    out_dir = Path.cwd() / "generated_graphs"
    out_dir.mkdir(parents=True, exist_ok=True)

    obs_rows = read_csv_rows(obs_path)
    spec_rows = read_csv_rows(spec_path)

    t = [float(r["time_sec"]) for r in obs_rows]
    eta = [float(r["eta_m"]) for r in obs_rows]
    freq = [float(r["frequency_hz"]) for r in spec_rows]
    energy = [float(r["energy"]) for r in spec_rows]

    mean = sum(eta) / len(eta)
    var = sum((x - mean) ** 2 for x in eta) / len(eta)
    std = math.sqrt(var)
    hs = 4.0 * std
    hs13 = compute_hs_top_third(eta)
    peak_idx = max(range(len(energy)), key=lambda i: energy[i])
    fp = freq[peak_idx]
    tp = 1.0 / fp if fp > 0 else 0.0

    jonswap = build_jonswap(freq, fp, hs)
    obs_norm = [x / max(energy) for x in energy]
    jon_norm = [x / max(jonswap) for x in jonswap]

    stem = obs_path.stem.replace("wave_observation_", "")

    fig, ax = plt.subplots(figsize=(10.5, 4.8), dpi=170)
    ax.plot(t, eta, color="#0b6e99", linewidth=1.0)
    ax.axhline(0.0, color="#666666", linestyle="--", linewidth=0.8)
    ax.set_title("Wave Observation Time Series")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Surface Elevation eta [m]")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / f"wave_observation_{stem}.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10.5, 4.8), dpi=170)
    ax.plot(freq, energy, color="#d17a00", linewidth=1.2)
    ax.set_title("Wave Energy Spectrum")
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("Energy [-]")
    ax.set_xlim(0.0, 2.0)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / f"wave_spectrum_{stem}.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10.8, 5.2), dpi=170)
    ax.plot(freq, obs_norm, color="#0b6e99", linewidth=1.8, label="Observed spectrum")
    ax.plot(freq, jon_norm, color="#cc6f00", linewidth=1.8, linestyle="--", label="JONSWAP fit")
    ax.axvline(fp, color="#888888", linewidth=0.9, linestyle=":", label=f"Peak = {fp:.3f} Hz")
    ax.set_title("Observed Spectrum vs. JONSWAP")
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("Normalized spectral energy [-]")
    ax.set_xlim(0.0, 2.0)
    ax.set_ylim(0.0, 1.08)
    ax.grid(True, alpha=0.28)
    stats = (
        f"Hs = {hs:.3f} m\n"
        f"Hs1/3 = {hs13:.3f} m\n"
        f"fp = {fp:.3f} Hz\n"
        f"Tp = {tp:.2f} s\n"
        f"Samples = {len(eta)}"
    )
    ax.text(
        0.98,
        0.24,
        stats,
        transform=ax.transAxes,
        ha="right",
        va="bottom",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.88, edgecolor="#bbbbbb"),
    )
    ax.legend(frameon=False, loc="upper right")
    fig.tight_layout()
    fig.savefig(out_dir / f"wave_spectrum_comparison_{stem}.png")
    plt.close(fig)

    print(out_dir / f"wave_observation_{stem}.png")
    print(out_dir / f"wave_spectrum_{stem}.png")
    print(out_dir / f"wave_spectrum_comparison_{stem}.png")


if __name__ == "__main__":
    main()
