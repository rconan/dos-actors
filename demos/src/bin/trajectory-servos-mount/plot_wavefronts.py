# Plot of the first and last exit pupil wavefronts saved by the crseo optical model
#   python plot_wavefronts.py <parquet> <out> [pupil size m, default 25.5]
# The outline of the first pupil is drawn over the last wavefront to show the pupil rotation
import sys
import numpy as np, pyarrow.parquet as pq, matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

pqf, out = sys.argv[1:3]
d = float(sys.argv[3]) if len(sys.argv) > 3 else 25.5
INK, INK2, SURF = "#0b0b0b", "#52514e", "#fcfcfb"
plt.rcParams.update({"font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2})
t = pq.read_table(pqf)
frames = [(0, "first"), (t.num_rows - 1, "last")]
fig, ax = plt.subplots(1, 2, figsize=(12, 5.6), facecolor=SURF)
first_mask = None
for a, (k, label) in zip(ax, frames):
    w = np.array(t.column("Wavefront")[k].as_py())
    n = int(np.sqrt(w.size))
    w = w.reshape(n, n) * 1e9  # nm
    mask = w != 0
    if first_mask is None:
        first_mask = mask
    v = np.where(mask, w, np.nan)
    lim = np.nanmax(np.abs(v))
    im = a.imshow(v, origin="lower", cmap="RdBu_r", vmin=-lim, vmax=lim,
                  extent=[-d / 2, d / 2, -d / 2, d / 2])
    if label == "last":
        a.contour(first_mask.astype(float), levels=[0.5], colors=INK, linewidths=0.6,
                  origin="lower", extent=[-d / 2, d / 2, -d / 2, d / 2])
    cb = fig.colorbar(im, ax=a, shrink=0.85)
    cb.set_label("Wavefront [nm]")
    a.set_title(f"{label} wavefront (t = {k + 1} s), rms {np.nanstd(v):.2f} nm", color=INK, loc="left")
    a.set_xlabel("x [m]"); a.set_ylabel("y [m]")
    a.set_facecolor(SURF)
fig.suptitle("Exit pupil wavefronts, GIR-rotated M1 & M2; black: first pupil outline", color=INK, x=0.01, ha="left")
fig.tight_layout(); fig.savefig(out, dpi=130)
