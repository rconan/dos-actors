# GMT M1 & M2 segment geometry and rigid telescope motion
#  * segment local frames and vertices in the OSS from the geotrans crate
#  * the rigid telescope motion is the coordinate transformation of the segment vertices and
#    frames by the rotation about the azimuth and elevation axes, both through the OSS origin,
#    linearized (small angles) as the FEM is linear: the vertex displacement is w x r and the
#    frame rotation is w, with w = azimuth x azimuth axis + elevation x elevation axis:
#      - azimuth axis: (0, sin(30deg), cos(30deg)) in the OSS for the FEM at 30deg zenith angle
#      - elevation axis: -x
import numpy as np

AZIMUTH_AXIS = np.array([0, np.sin(np.deg2rad(30)), np.cos(np.deg2rad(30))])
ELEVATION_AXIS = np.array([-1.0, 0, 0])


def rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def sag(r, radius, k):
    return r**2 / (radius * (1 + np.sqrt(1 - (1 + k) * r**2 / radius**2)))


def vertices(distance, clocking0, height, radius, k):
    """segment vertices [m] in the OSS, segment 7 last"""
    o = np.deg2rad(90 + clocking0 - 60 * np.arange(6))
    z = np.full(6, height + sag(distance, radius, k))
    return np.vstack([np.column_stack([distance * np.cos(o), distance * np.sin(o), z]), [0, 0, height]])


d = np.deg2rad
# segment vertices and local frames (v_oss = Q @ v_local) in the OSS
MIRRORS = {
    "M1RigidBodyMotions": (
        vertices(8.71, 0, 3.9, 36.0, -0.9982857),
        np.array([rz(d(-60 * i)) @ rx(d(13.601685)) for i in range(6)] + [np.eye(3)]),
    ),
    # M2 S7 frame is flipped w.r.t. the OSS (Rz(pi)Ry(pi)), as found from the rigid telescope rotation
    "M2RigidBodyMotions": (
        vertices(1.08774, 180, 3.9 + 20.26247614, -4.1639009, -0.71692784),
        np.array([rz(d(180 - 60 * i)) @ ry(np.pi) @ rx(d(14.777498)) for i in range(6)] + [rz(np.pi) @ ry(np.pi)]),
    ),
}


def telescope_rotation(azimuth, elevation):
    """telescope rotation vectors (n, 3) [rad] in the OSS"""
    return np.outer(azimuth, AZIMUTH_AXIS) + np.outer(elevation, ELEVATION_AXIS)


def rigid_motion(name, azimuth, elevation):
    """rigid body motions (n, 7, 6) of the segments of mirror `name` in their local frames
    for the telescope rotated by `azimuth` and `elevation` [rad]"""
    r, q = MIRRORS[name]
    w_oss = telescope_rotation(azimuth, elevation)
    t_oss = np.cross(w_oss[:, None, :], r[None, :, :])  # vertex displacements
    t = np.einsum("sji,nsj->nsi", q, t_oss)  # OSS -> local frames
    w = np.einsum("sji,nj->nsi", q, w_oss)
    return np.concatenate([t, w], axis=2)


def remove_rigid_motion(name, rbm, azimuth, elevation):
    """rigid body motions (n, 7, 6) minus the rigid telescope motion"""
    return rbm.reshape(-1, 7, 6) - rigid_motion(name, azimuth, elevation)
