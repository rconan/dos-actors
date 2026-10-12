import sys
import numpy as np, pyarrow.parquet as pq, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
csv, pqf, out = sys.argv[1:4]
fs = int(sys.argv[4]) if len(sys.argv) > 4 else 1000
e = np.rad2deg(np.array(pq.read_table(pqf).column("MountEncoders").to_pylist())) * 3600
n = e.shape[0]
c = np.loadtxt(csv, delimiter=",", skiprows=1)
d = (c[:, 1:] - c[0, 1:]) * 3600  # arcsec offsets from initial pointing
k = np.arange(n); i = k // fs; a = (k % fs) / fs
sp = d[i] + a[:, None] * (d[np.minimum(i + 1, len(d) - 1)] - d[i])
t = k / fs
enc = np.column_stack([e[:, 0:4].mean(1), e[:, 4:10].mean(1), e[:, 10:14].mean(1)])
names = ["Azimuth", "Elevation", "GIR (rotator)"]
BLUE, ORANGE, INK, INK2, GRID = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({"font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK2,
    "xtick.color": INK2, "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False})
fig, ax = plt.subplots(3, 2, figsize=(12, 9), sharex=True, facecolor="#fcfcfb")
for j in range(3):
    l = ax[j, 0]; r = ax[j, 1]
    l.plot(c[:, 0][c[:, 0] <= t[-1]], d[c[:, 0] <= t[-1], j], "o", ms=4, color=BLUE, label="ATP trajectory (1 Hz)")
    l.plot(t, enc[:, j], lw=2, color=ORANGE, label="FEM encoders (mean)")
    r.plot(t, enc[:, j] - sp[:, j], lw=1.5, color=ORANGE)
    l.set_ylabel(f"{names[j]} [arcsec]"); r.set_ylabel(f"{names[j]} error [arcsec]")
    for x in (l, r): x.set_facecolor("#fcfcfb"); x.grid(color=GRID, lw=0.8)
ax[0, 0].set_title("Setpoint and encoders (offset from start)", color=INK, loc="left")
ax[0, 1].set_title("Tracking error (encoders − interpolated setpoint)", color=INK, loc="left")
ax[0, 0].legend(frameon=False)
for x in ax[2]: x.set_xlabel("Time [s]")
fig.suptitle(f"ATP tracking trajectory vs mount encoders, {n / fs:.0f} s at {fs} Hz", color=INK, x=0.01, ha="left")
fig.tight_layout(); fig.savefig(out, dpi=130)
for j in range(3):
    err = enc[:, j] - sp[:, j]
    print(names[j], "error rms after 1 s:", np.sqrt(np.mean(err[fs:]**2)).round(4), "max |err|:", np.abs(err).max().round(3))
