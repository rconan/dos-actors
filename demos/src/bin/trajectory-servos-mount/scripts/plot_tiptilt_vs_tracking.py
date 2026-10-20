# Off-axis image motion vs mount tracking error (setpoint - average mount encoders)
#   python plot_tiptilt_vs_tracking.py <mount parquet> <optics parquet> <out> [sampling frequency Hz, default 1000] [pixel mas, default 3.93] [off-axis arcsec, default 0.25]
# The crseo optical model images a source off-axis along x on a camera in absolute pointing along
# the optical axis: the image is at the source direction seen from the instrument, i.e. the
# off-axis angle rotated by -GIR tracking error (PointingRotationError) and offset by
# -PointingError, plus the tip-tilt from the structure deformations (TipTilt, ordered [y, x]).
# The frame is row-major with x along the columns.
import sys
import numpy as np, pyarrow.parquet as pq, matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

mount_pq, optics_pq, out = sys.argv[1:4]
fs = float(sys.argv[4]) if len(sys.argv) > 4 else 1000.
pixel = float(sys.argv[5]) if len(sys.argv) > 5 else 3.93
off_axis = (float(sys.argv[6]) if len(sys.argv) > 6 else 0.25) * 1e3  # mas
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
pe = np.array(ft.column("PointingError").to_pylist()) * MAS
gir = np.array(ft.column("PointingRotationError").to_pylist())[:, 0]
t_tt = np.arange(1, m + 1) * rate / fs
# image motion from the nominal position
image = centroid - [off_axis, 0.]
# image motion from the field rotation error
rotation = np.c_[(np.cos(gir) - 1) * off_axis, -np.sin(gir) * off_axis]
expected = rotation - pe + tt
# frames with the image within 80% of the field-of-view (the centroid is biased near the edges)
fov = (np.abs(expected + [off_axis, 0.]) < 0.8 * npx / 2 * pixel).all(1)
residual = image - expected
bias = residual[t_tt > 2].mean(0)  # static centroid bias
residual -= bias

fig, ax = plt.subplots(4, 1, figsize=(11, 12), facecolor=SURF, sharex=True)
for a in ax:
    a.set_facecolor(SURF)
ax[0].plot(time, sky[:, 0], color=C[0], lw=1, label="azimuth × cos(el)")
ax[0].plot(time, sky[:, 1], color=C[1], lw=1, label="elevation")
ax[0].plot(time, err[:, 2], color=C[2], lw=1, label="GIR (field rotation)")
ax[0].set_ylabel("setpoint − encoders [mas]")
ax[0].set_title("Mount tracking error", color=INK, loc="left")
ax[0].legend(frameon=False)
for i, xy in enumerate("xy"):
    ax[1].plot(t_tt, np.where(fov, image[:, i] - bias[i], np.nan), ".-", color=C[i], lw=1, ms=3,
               label=f"frame centroid {xy}")
    ax[1].plot(t_tt, -pe[:, i], color=INK, lw=0.8, ls="--" if i == 0 else "-.", label=f"−PointingError {xy}")
ax[1].set_ylabel("image motion [mas]")
ax[1].set_title(f"Off-axis ({off_axis / 1e3:g}\" along x) image motion ({fs / rate:.0f} Hz)", color=INK, loc="left")
ax[1].legend(frameon=False, ncol=2)
ax[2].plot(t_tt, np.hypot(*sky[rate - 1::rate][:m].T), color=INK2, lw=1, label="|on-sky tracking error|")
ax[2].plot(t_tt, np.where(fov, np.hypot(*(image - bias).T), np.nan), color=C[0], lw=1.2, label="|image motion|")
ax[2].plot(t_tt, np.hypot(*tt.T), color=C[2], lw=1, label="|TipTilt| (structure)")
ax[2].plot(t_tt, np.hypot(*rotation.T), color=C[1], lw=1, label="|field rotation error × off-axis angle|")
ax[2].set_yscale("log")
ax[2].set_ylabel("magnitude [mas]")
ax[2].set_title("Magnitudes", color=INK, loc="left")
ax[2].legend(frameon=False)
for i, xy in enumerate("xy"):
    ax[3].plot(t_tt, np.where(fov, residual[:, i], np.nan), color=C[i], lw=1, label=xy)
ax[3].set_ylabel("residual [mas]")
ax[3].set_title("Image motion − (field rotation − PointingError + TipTilt), static bias removed",
                color=INK, loc="left")
ax[3].legend(frameon=False)
ax[-1].set_xlabel("time [s]")
fig.tight_layout()
fig.savefig(out, dpi=130)

print(f"{m} optics samples at {fs / rate:.0f} Hz, {n} mount samples at {fs:.0f} Hz, {fov.sum()} images in the field")
print(f"static centroid bias [mas]: {bias.round(3)}")
for t0 in (0., 2., 5.):
    k = (t_tt > t0) & fov
    print(f"t > {t0:.0f}s: rms [mas] image x {image[k, 0].std():8.3f} y {image[k, 1].std():7.3f}"
          f" | -PointingError x {pe[k, 0].std():8.3f} y {pe[k, 1].std():7.3f}"
          f" | TipTilt x {tt[k, 0].std():6.3f} y {tt[k, 1].std():6.3f}"
          f" | field rotation x {rotation[k, 0].std():.1e} y {rotation[k, 1].std():.1e}"
          f" | residual x {residual[k, 0].std():6.3f} y {residual[k, 1].std():6.3f}")
