# Image tip-tilt vs mount tracking error (setpoint - average mount encoders)
#   python plot_tiptilt_vs_tracking.py <mount parquet> <optics parquet> <out> [sampling frequency Hz, default 1000] [pixel mas, default 3.93]
# The tip-tilt is the source wavefront gradient pupil average (TipTilt) of the crseo optical model,
# compared to the centroid of the camera frames (Frame<Host>) saved in the same parquet file,
# the optics rate being given by the number of rows of that file.
# crseo TipTilt is ordered [y, x]; the frame is row-major with x along the columns.
# If the telescope pointing error (PointingError, OSS [x, y]) fed to the optical model is saved too,
# the image is offset by -PointingError: it is plotted and removed from the tip-tilt to show
# the tip-tilt from the structure deformations alone.
import sys
import numpy as np, pyarrow.parquet as pq, matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

mount_pq, optics_pq, out = sys.argv[1:4]
fs = float(sys.argv[4]) if len(sys.argv) > 4 else 1000.
pixel = float(sys.argv[5]) if len(sys.argv) > 5 else 3.93
EL = np.radians(45.)  # ATP pointing elevation
MAS = 180 / np.pi * 3600e3
INK, INK2, SURF = "#0b0b0b", "#52514e", "#fcfcfb"
C = ["#2a6fdb", "#d1495b", "#3b9c6e"]
plt.rcParams.update({"font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2})

t = pq.read_table(mount_pq)
sp = np.array(t.column("MountSetPoint").to_pylist())
enc = np.array(t.column("AverageMountEncoders").to_pylist())
err = (sp - enc) * MAS
n = sp.shape[0]
time = np.arange(1, n + 1) / fs
# on-sky tracking error: azimuth scaled by cos(elevation)
sky = np.c_[err[:, 0] * np.cos(EL), err[:, 1]]

ft = pq.read_table(optics_pq)
frames = np.array(ft.column("Frame<Host>").to_pylist())
m = frames.shape[0]
npx = int(np.sqrt(frames.shape[1]))
frames = frames.reshape(m, npx, npx)
rate = n // m
x = np.arange(npx) - (npx - 1) / 2
flux = frames.sum((1, 2))
centroid = np.c_[(frames.sum(1) * x).sum(1), (frames.sum(2) * x).sum(1)] / flux[:, None] * pixel
tt = np.array(ft.column("TipTilt").to_pylist())[:, ::-1] * MAS
pe = -np.array(ft.column("PointingError").to_pylist()) * MAS if "PointingError" in ft.column_names else None
t_tt = np.arange(1, m + 1) * rate / fs
# tracking error at the optics samples
sky_tt = sky[rate - 1::rate][:m]
gir_tt = err[rate - 1::rate, 2][:m]

fig, ax = plt.subplots(3, 1, figsize=(11, 9), facecolor=SURF, sharex=True)
for a in ax:
    a.set_facecolor(SURF)
ax[0].plot(time, sky[:, 0], color=C[0], lw=1, label="azimuth × cos(el)")
ax[0].plot(time, sky[:, 1], color=C[1], lw=1, label="elevation")
ax[0].plot(time, err[:, 2], color=C[2], lw=1, label="GIR (field rotation)")
ax[0].set_ylabel("setpoint − encoders [mas]")
ax[0].set_title("Mount tracking error", color=INK, loc="left")
ax[0].legend(frameon=False)
for i, xy in enumerate("xy"):
    ax[1].plot(t_tt, tt[:, i], ".-", color=C[i], lw=1, ms=3, label=f"TipTilt {xy}")
    ax[1].plot(t_tt, centroid[:, i], color=C[i], lw=0.8, ls=":", label=f"frame centroid {xy}")
    if pe is not None:
        ax[1].plot(t_tt, pe[:, i], color=INK, lw=0.8, ls="--" if i == 0 else "-.", label=f"−PointingError {xy}")
ax[1].set_ylabel("tip-tilt [mas]")
ax[1].set_title(f"Image tip-tilt ({fs / rate:.0f} Hz)", color=INK, loc="left")
ax[1].legend(frameon=False, ncol=2)
ax[2].plot(t_tt, np.hypot(*sky_tt.T), color=INK2, lw=1, label="|on-sky tracking error|")
ax[2].plot(t_tt, np.hypot(*tt.T), color=C[0], lw=1.2, label="|tip-tilt|")
if pe is not None:
    ax[2].plot(t_tt, np.hypot(*(tt - pe).T), color=C[2], lw=1, label="|tip-tilt + PointingError| (structure)")
ax[2].set_yscale("log")
ax[2].set_ylabel("magnitude [mas]")
ax[2].set_xlabel("time [s]")
ax[2].set_title("Magnitudes", color=INK, loc="left")
ax[2].legend(frameon=False)
fig.tight_layout()
fig.savefig(out, dpi=130)

print(f"{m} optics samples at {fs / rate:.0f} Hz, {n} mount samples at {fs:.0f} Hz")
r = [np.corrcoef(tt[:, i], centroid[:, i])[0, 1] for i in range(2)]
print(f"TipTilt vs frame centroid: correlation x {r[0]:.3f} y {r[1]:.3f},"
      f" rms difference {np.std(tt - centroid, 0).round(2)} mas")
for t0 in (0., 2., 5.):
    k = t_tt > t0
    print(f"t > {t0:.0f}s: rms [mas] tracking az.cos(el) {sky_tt[k, 0].std():8.2f} el {sky_tt[k, 1].std():6.2f}"
          f" GIR {gir_tt[k].std():8.2f} | tip-tilt x {tt[k, 0].std():6.2f} y {tt[k, 1].std():6.2f}")
    if pe is not None:
        s = tt[k] - pe[k]
        r = [np.corrcoef(tt[k, i], pe[k, i])[0, 1] for i in range(2)]
        print(f"  -PointingError x {pe[k, 0].std():6.2f} y {pe[k, 1].std():6.2f}"
              f" | structure x {s[:, 0].std():6.2f} y {s[:, 1].std():6.2f}"
              f" | correlation tip-tilt vs -PointingError x {r[0]:.2f} y {r[1]:.2f}")
    else:
        r = np.corrcoef(np.c_[sky_tt[k], gir_tt[k], tt[k]].T)[3:, :3]
        print("  correlation tip-tilt (x, y) vs tracking (az, el, GIR):")
        print("  " + np.array2string(r, precision=2).replace("\n", "\n  "))
