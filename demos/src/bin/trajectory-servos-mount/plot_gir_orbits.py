# Orbit plot of the M1 and M2 segments as seen by an observer at the center of the GIR,
# rotating with it
#  * the rigid telescope motion (azimuth & elevation) is removed as in plot_rbm.py:
#    the observer moves with the telescope
#  * the segment motions are transformed from their local frames into the OSS
#    (frame rotations from the geotrans crate)
#  * the OSS is rotated by minus the GIR angle (mean of the GIR encoders) about the
#    OSS z axis: the mirrors are seen spinning around the GIR axis
import sys
import numpy as np, pyarrow.parquet as pq, matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import RegularPolygon

pqf, out = sys.argv[1:3]
fs = int(sys.argv[3]) if len(sys.argv) > 3 else 1000
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


def rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def centers(distance, clocking0):
    """segment centers [m] in the OSS XY plane, segment 7 at the origin"""
    o = np.deg2rad(90 + clocking0 - 60 * np.arange(6))
    return np.vstack(
        [np.column_stack([distance * np.cos(o), distance * np.sin(o)]), [0, 0]]
    )


d = np.deg2rad
mirrors = {
    "M1RigidBodyMotions": (
        centers(8.71, 0),
        [rz(d(-60 * i)) @ rx(d(13.601685)) for i in range(6)] + [np.eye(3)],
    ),
    # M2 S7 frame is flipped w.r.t. the OSS (Rz(pi)Ry(pi)), as found from the rigid telescope rotation
    "M2RigidBodyMotions": (
        centers(1.08774, 180),
        [rz(d(180 - 60 * i)) @ ry(np.pi) @ rx(d(14.777498)) for i in range(6)]
        + [rz(np.pi) @ ry(np.pi)],
    ),
}
enc = np.array(table.column("MountEncoders").to_pylist())
angles = np.column_stack(
    [enc[:, 0:4].mean(1), enc[:, 4:10].mean(1)]
)  # azimuth, elevation [rad]
gir = enc[:, 10:14].mean(1)  # GIR [rad]
t = np.arange(enc.shape[0]) / fs
mas = np.rad2deg(1) * 3600e3

fig, ax = plt.subplots(2, 2, figsize=(11, 11), facecolor=SURF)
for row, (name, (xy, q)) in enumerate(mirrors.items()):
    rbm = np.array(table.column(name).to_pylist())
    coefs, *_ = np.linalg.lstsq(angles, rbm, rcond=None)
    res = (rbm - angles @ coefs).reshape(-1, 7, 6)
    q = np.array(q)
    dt = np.einsum("sjk,nsk->nsj", q, res[:, :, :3])  # OSS translations
    dr = np.einsum("sjk,nsk->nsj", q, res[:, :, 3:])  # OSS rotations
    r0 = np.column_stack([xy, np.zeros(7)])
    spin = np.array([rz(-g) for g in gir])  # OSS -> GIR frame
    # segment displacement and rotation seen from the GIR
    disp = np.einsum("nij,nsj->nsi", spin, r0[None] + dt) - r0[None]
    rot = np.einsum("nij,nsj->nsi", spin, dr)
    rot[:, :, 2] -= gir[:, None]
    pitch = np.linalg.norm(xy[0])
    for col, (orbit, unit, scale, dofs) in enumerate(
        [(disp[:, :, :2], "mm", 1e3, "Tx/Ty"), (rot[:, :, :2], "mas", mas, "Rx/Ry")]
    ):
        a = ax[row, col]
        orbit = orbit * scale
        amp = np.abs(orbit).max()
        k = 0.4 * pitch / amp  # largest excursion drawn at 40% of the segment pitch
        for s in range(7):
            a.add_patch(
                RegularPolygon(
                    xy[s],
                    6,
                    radius=0.5 * pitch,
                    orientation=np.pi / 6,
                    facecolor="none",
                    edgecolor=GRID,
                    lw=1,
                )
            )
            a.plot(
                xy[s, 0] + k * orbit[:, s, 0],
                xy[s, 1] + k * orbit[:, s, 1],
                lw=1.5,
                color=SEG[s],
            )
            a.plot(
                xy[s, 0] + k * orbit[-1, s, 0],
                xy[s, 1] + k * orbit[-1, s, 1],
                "o",
                ms=4,
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
        e = 10 ** np.floor(np.log10(amp))
        bar = max(m * e for m in (1, 2, 5) if m * e <= amp)
        x0, y0 = -1.45 * pitch, -1.45 * pitch
        a.plot([x0, x0 + k * bar], [y0, y0], color=INK, lw=2)
        a.text(x0, y0 + 0.04 * pitch, f"{bar:g} {unit}", color=INK, va="bottom")
        a.set_title(f"{name[:2]} {dofs}, max {amp:.3g} {unit}", color=INK, loc="left")
        a.set_aspect("equal")
        a.set_xlim(-1.6 * pitch, 1.6 * pitch)
        a.set_ylim(-1.6 * pitch, 1.6 * pitch)
        a.set_facecolor(SURF)
        a.set_xlabel("GIR X [m]")
        a.set_ylabel("GIR Y [m]")
        for sp in ("top", "right"):
            a.spines[sp].set_visible(False)
    print(
        f"{name}: final Rz seen from the GIR {np.rad2deg(rot[-1, :, 2]).mean() * 3600:.1f} arcsec, "
        f"final outer segment displacement {np.linalg.norm(disp[-1, :6, :2], axis=1).mean() * 1e3:.2f} mm"
    )
fig.suptitle(
    f"Segment orbits seen from the GIR center (GIR {np.rad2deg(gir[-1]) * 3600:.0f} arcsec), "
    f"{t[-1]:.0f} s at {fs} Hz; dot: end of run",
    color=INK,
    x=0.01,
    ha="left",
)
fig.tight_layout(rect=(0, 0, 1, 0.97))
fig.savefig(out, dpi=110)
