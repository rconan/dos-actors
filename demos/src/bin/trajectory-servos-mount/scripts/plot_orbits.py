# Orbit plot of the M1 and M2 segment rigid body motions minus the rigid telescope rotation
#  * each segment orbit (Tx vs Ty and Rx vs Ry) is drawn around the segment center,
#    with the segments laid out as in the OSS (rigid_body.py)
#  * default: the rigid telescope motion is removed as in plot_rbm.py (rigid_body.py)
#  * with "s7" as 4th argument: the rigid body motions of all the segments are transformed
#    from their local frames into the frame of the center segment S7 (rigid_body.py)
#  * left: whole run, right: after the start-up transient (t > T_SETTLE)
import sys
import numpy as np, pyarrow.parquet as pq, matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import rigid_body

pqf, out = sys.argv[1:3]
fs = int(sys.argv[3]) if len(sys.argv) > 3 else 1000
relative_s7 = len(sys.argv) > 4 and sys.argv[4] == "s7"
T_SETTLE = 5.0  # s
table = pq.read_table(pqf)
SEG = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7"]
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
plt.rcParams.update(
    {
        "font.size": 9,
        "axes.edgecolor": INK2,
        "axes.labelcolor": INK2,
        "xtick.color": INK2,
        "ytick.color": INK2,
    }
)


def in_s7_frame(rbm, q):
    """rigid body motions (n, 7, 6) of each segment expressed in the S7 frame"""
    return np.concatenate(
        [
            np.einsum("ij,sjk,nsk->nsi", q[6].T, np.array(q), rbm[:, :, k : k + 3])
            for k in (0, 3)
        ],
        axis=2,
    )


enc = np.array(table.column("MountEncoders").to_pylist())
angles = np.column_stack([enc[:, 0:4].mean(1), enc[:, 4:10].mean(1)])
t = np.arange(enc.shape[0]) / fs
mas = np.rad2deg(1) * 3600e3
windows = [("whole run", t >= 0), (f"t > {T_SETTLE:.0f} s", t > T_SETTLE)]

fig, ax = plt.subplots(4, 2, figsize=(11, 21), facecolor=SURF)
row = 0
for name, (vertices, frames) in rigid_body.MIRRORS.items():
    xy = vertices[:, :2]
    rbm = np.array(table.column(name).to_pylist())
    if relative_s7:
        res = in_s7_frame(rbm.reshape(-1, 7, 6), frames)
    else:
        res = rigid_body.remove_rigid_motion(name, rbm, *angles.T)
    pitch = np.linalg.norm(xy[0])
    for dofs, unit, scale in [((0, 1), "nm", 1e9), ((3, 4), "mas", mas)]:
        for col, (label, mask) in enumerate(windows):
            a = ax[row, col]
            orbit = res[mask][:, :, dofs] * scale  # (n, 7, 2)
            amp = np.abs(orbit).max()
            k = 0.4 * pitch / amp  # largest excursion drawn at 40% of the segment pitch
            for s in range(7):
                a.add_patch(
                    Circle(
                        xy[s],
                        radius=0.48 * pitch,  # segment diameter ~96% of the pitch
                        facecolor="none",
                        edgecolor=GRID,
                        lw=1,
                    )
                )
                a.plot(
                    xy[s, 0] + k * orbit[:, s, 0],
                    xy[s, 1] + k * orbit[:, s, 1],
                    lw=1,
                    color=SEG[s],
                )
                a.plot(*xy[s], "+", color=INK2, ms=6)
                a.text(
                    xy[s, 0],
                    xy[s, 1] - 0.47 * pitch,
                    f"S{s + 1}",
                    ha="center",
                    va="bottom",
                    color=INK2,
                )
            # scale bar
            e = 10 ** np.floor(np.log10(amp))
            bar = max(m * e for m in (1, 2, 5) if m * e <= amp)
            x0, y0 = -1.45 * pitch, -1.45 * pitch
            a.plot([x0, x0 + k * bar], [y0, y0], color=INK, lw=2)
            a.text(x0, y0 + 0.04 * pitch, f"{bar:g} {unit}", color=INK, va="bottom")
            d0, d1 = ("Tx", "Ty") if dofs[0] == 0 else ("Rx", "Ry")
            a.set_title(
                f"{name[:2]} {d0}/{d1}, {label}, max {amp:.3g} {unit}",
                color=INK,
                loc="left",
            )
            a.set_aspect("equal")
            a.set_xlim(-1.6 * pitch, 1.6 * pitch)
            a.set_ylim(-1.6 * pitch, 1.6 * pitch)
            a.set_facecolor(SURF)
            a.set_xlabel("OSS X [m]")
            a.set_ylabel("OSS Y [m]")
            for sp in ("top", "right"):
                a.spines[sp].set_visible(False)
        row += 1
fig.suptitle(
    f"Segment orbits {'in the S7 frame' if relative_s7 else 'minus the rigid telescope rotation'}, {t[-1]:.0f} s at {fs} Hz",
    color=INK,
    x=0.01,
    ha="left",
)
fig.tight_layout(rect=(0, 0, 1, 0.985))
fig.savefig(out, dpi=110)
