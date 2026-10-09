# Plot the M1 and M2 segment rigid body motions saved in model_data_1.parquet
#  * <out>: the rigid body motions as saved
#  * <out stem>_minus_rigid.png: the same after removing the rigid motion of the whole
#    telescope, the coordinate transformation of the segments by the azimuth and elevation
#    rotations (mean of the encoders) in rigid_body.py
import sys
import numpy as np, pyarrow.parquet as pq, matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rigid_body

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
        "axes.spines.top": False,
        "axes.spines.right": False,
    }
)
dofs = [
    ("Tx", "µm", 1e6),
    ("Ty", "µm", 1e6),
    ("Tz", "µm", 1e6),
    ("Rx", "arcsec", np.rad2deg(1) * 3600),
    ("Ry", "arcsec", np.rad2deg(1) * 3600),
    ("Rz", "arcsec", np.rad2deg(1) * 3600),
]
enc = np.array(table.column("MountEncoders").to_pylist())
# azimuth & elevation angles [rad]: mean of the 4 azimuth and of the 6 elevation encoders
angles = np.column_stack([enc[:, 0:4].mean(1), enc[:, 4:10].mean(1)])


def figure(rbms, dofs, title, out):
    fig, ax = plt.subplots(6, 2, figsize=(12, 13), sharex=True, facecolor=SURF)
    for col, (name, rbm) in enumerate(rbms.items()):
        t = np.arange(rbm.shape[0]) / fs
        for row, (dof, unit, scale) in enumerate(dofs):
            a = ax[row, col]
            for s in range(7):
                a.plot(
                    t,
                    rbm[:, 6 * s + row] * scale,
                    lw=1.5,
                    color=SEG[s],
                    label=f"S{s + 1}",
                )
            a.set_ylabel(f"{dof} [{unit}]")
            a.set_facecolor(SURF)
            a.grid(color=GRID, lw=0.8)
        ax[0, col].set_title(
            name.replace("RigidBodyMotions", " segment rigid body motions"),
            color=INK,
            loc="left",
        )
        ax[-1, col].set_xlabel("Time [s]")
    ax[0, 1].legend(
        ncol=7,
        frameon=False,
        loc="lower right",
        bbox_to_anchor=(1, 1.12),
        handlelength=1.2,
    )
    fig.suptitle(f"{title}, {t[-1]:.0f} s at {fs} Hz", color=INK, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(out, dpi=130)


rbms = {
    name: np.array(table.column(name).to_pylist())  # (n, 7 segments x 6 dofs)
    for name in ["M1RigidBodyMotions", "M2RigidBodyMotions"]
}
figure(rbms, dofs, "M1 & M2 rigid body motions along the ATP trajectory", out)

residuals = {}
for name, rbm in rbms.items():
    residuals[name] = rigid_body.remove_rigid_motion(name, rbm, *angles.T).reshape(rbm.shape)
mas = np.rad2deg(1) * 3600e3
figure(
    residuals,
    [
        ("Tx", "nm", 1e9),
        ("Ty", "nm", 1e9),
        ("Tz", "nm", 1e9),
        ("Rx", "mas", mas),
        ("Ry", "mas", mas),
        ("Rz", "mas", mas),
    ],
    "M1 & M2 rigid body motions minus the rigid telescope rotation",
    out.rsplit(".", 1)[0] + "_minus_rigid.png",
)
