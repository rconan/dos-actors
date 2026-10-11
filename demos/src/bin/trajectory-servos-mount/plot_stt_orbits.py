# Orbit plot of the segment tip-tilt from the linear optical model (SegmentTipTilt),
# the 7 segment orbits in the same axes, after T_START seconds of simulation
#   python plot_stt_orbits.py <parquet> <out> [sampling frequency, default 1000] [T_START, default 10] [T_END, default: end of run]
import sys
import numpy as np, pyarrow.parquet as pq, matplotlib

matplotlib.use("Agg")
ALPHA = 0.5  # orbit lines transparency
import matplotlib.pyplot as plt

pqf, out = sys.argv[1:3]
fs = int(sys.argv[3]) if len(sys.argv) > 3 else 1000
t_start = float(sys.argv[4]) if len(sys.argv) > 4 else 10.0
t_end = float(sys.argv[5]) if len(sys.argv) > 5 else np.inf
SEG = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7"]
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
plt.rcParams.update({"font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2,
                     "axes.spines.top": False, "axes.spines.right": False})

# [x1,...,x7,y1,...,y7] in radians
stt = np.array(pq.read_table(pqf).column("SegmentTipTilt").to_pylist()) * np.rad2deg(1) * 3600e3
t = np.arange(stt.shape[0]) / fs
mask = (t >= t_start) & (t <= t_end)
x, y = stt[mask, :7], stt[mask, 7:]

fig, ax = plt.subplots(figsize=(7.5, 7), facecolor=SURF)
for s in range(7):
    ax.plot(x[:, s], y[:, s], lw=1.2, color=SEG[s], alpha=ALPHA, label=f"S{s + 1}")
    ax.plot(x[-1, s], y[-1, s], "o", ms=5, color=SEG[s])
ax.axhline(0, color=GRID, lw=0.8, zorder=0); ax.axvline(0, color=GRID, lw=0.8, zorder=0)
ax.set_aspect("equal", adjustable="datalim")
ax.set_facecolor(SURF); ax.grid(color=GRID, lw=0.8)
ax.set_xlabel("Segment tip-tilt x [mas]"); ax.set_ylabel("Segment tip-tilt y [mas]")
leg = ax.legend(ncol=7, frameon=False, loc="lower left", bbox_to_anchor=(0, 1.0), handlelength=1.2, columnspacing=1)
for h in leg.legend_handles:
    h.set_alpha(1)
fig.suptitle(f"Segment tip-tilt orbits, {t[mask][0]:.0f}-{t[mask][-1]:.0f} s; dot: end", color=INK, x=0.02, ha="left")
fig.tight_layout(); fig.savefig(out, dpi=130)
print(f"max |tip-tilt| in {t[mask][0]:g}-{t[mask][-1]:g} s: {np.abs(stt[mask]).max():.2f} mas")
