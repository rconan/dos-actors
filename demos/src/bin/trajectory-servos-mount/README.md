# trajectory-servos-mount

Integrated model of the GMT servo-mechanisms (mount, M1 and M2 control systems) driven along
the ATP default tracking trajectory, with the segment tip-tilt from the linear optical model
(LOM) and, optionally, the exit pupil wavefront from the crseo optical model.

* **Trajectory:** `atp_trajectory.csv` holds 5 minutes of azimuth, elevation and rotator (GIR)
  angles at 1 Hz. They are computed with astropy (`scripts/atp_trajectory.py`) for the ATP default
  pointing: alt 45°, az 0°, 2018-01-01T04:00:00 UTC, Las Campanas. The rotator follows the change
  in parallactic angle. The mount setpoints are the offsets from the initial pointing, linearly
  interpolated to the mount sampling frequency.
* **Kinematic compensation:** the rigid motion of the telescope as the mount moves is removed
  from all the FEM outputs but the mount encoders. That includes the outputs fed back to the
  M1 and M2 control systems, e.g. the M1 hardpoint forces. The kinematic gain from the mount
  encoders comes from the DC gain of the FEM state space model
  (`DiscreteStateSpace::kinematic_compensation` in `gmt_dos-clients_fem`).
* **Mount feed-forward:** the FEM azimuth, elevation and GIR "rigid body" modes have a small
  stiffness (0.07–0.1 Hz). Holding the mount at an angle therefore takes a drive torque
  proportional to that angle, and over the full trajectory the azimuth drives would saturate
  (800 kN·m at about 242 s). That torque is fed forward to the mount drives, after the drives
  saturation (`feedforward.rs`, `MountTorquesFeedForward` input of `gmt_dos-clients_mount`).
* **Outputs:** the mount encoders, the compensated M1 & M2 rigid body motions and the segment
  tip-tilt are saved to `data/model_data_1.parquet`.

## Folders

| Folder | Content |
|---|---|
| `data/` | simulation data: `model_data_1.parquet`, `model_data_1000.parquet` (crseo) and `model_flowchart.html` |
| `figures/` | plots |
| `scripts/` | Python scripts: trajectory, plots and `rigid_body.py` |

## Quick start

```bash
make build
make run
make plot
```

Any parameter below can be set on the `make` command line, e.g. `make run SIM_DURATION=60`.

## Makefile parameters

| Parameter | Default | Description |
|---|---|---|
| `FEM_REPO` | 20251110_1617_zen_30_M1_202110_FSM_202305_Mount_202305_pier_202411_M1Fans | Path to the FEM 2nd order model (`modal_state_space_model_2ndOrder.zip`). Read at build time (FEM inputs/outputs code generation) and at run time |
| `MOUNT_MODEL` | `MOUNT_FDR_1kHz` | Mount control model: `MOUNT_PDR_8kHz`, `MOUNT_FDR_1kHz`, `MOUNT_FDR_1kHz-az17Hz` or `MOUNT_FDR_8kHz`. It sets the simulation sampling frequency. Build time |
| `SAMPLING_FREQUENCY` | `1000` | Sampling frequency [Hz] used by the plot scripts. Must match `MOUNT_MODEL` |
| `CUDACXX` | `/usr/local/cuda/bin/nvcc` | CUDA compiler for the FEM CUDA solver (`cuda` feature) |
| `SIM_DURATION` | `30` | Simulation duration [s], up to 300, the length of the trajectory. The binary defaults to 300 when it is not set |
| `FLOWCHART` | `dot` | Layout of the model flowchart (`data/model_flowchart.html`): `dot`, `neato` or `fdp` (Graphviz), `elk` (laid out in the browser with elkjs, no Graphviz needed), empty or `none` for no flowchart |
| `STT_START` | `10` | Start time [s] of the segment tip-tilt orbit plot |
| `FEATURES` | `cuda gmt_dos-clients_arrow` | Cargo features, see below |
| `PYTHON` | `python3` | Python interpreter for the trajectory and plot scripts |

## Cargo features

| Feature | Description |
|---|---|
| `cuda` | FEM state space solved on the GPU (`CuStateSpace`). Drop it to use the CPU solver (`ExponentialMatrix`), which is much slower |
| `gmt_dos-clients_arrow` | Data logging to parquet files. Required |
| `gmt_dos-clients_crseo` | Adds the GIR rotation and the crseo optical model, see below |
| `flowchart-replay` | Records the run into the ELK flowchart (`FLOWCHART=elk`) to replay it step by step in the browser. The whole run is recorded unless `FLOWCHART_STEPS` limits the number of recorded steps, e.g. `make run FLOWCHART=elk SIM_DURATION=3 FEATURES="cuda gmt_dos-clients_arrow gmt_dos-clients_crseo flowchart-replay"` |

## Run time environment variables

These are read by the binary. Set them on the `make run` command line, e.g.
`make run KINEMATIC_COMPENSATION=0`.

| Variable | Default | Description |
|---|---|---|
| `SIM_DURATION` | `300` | Simulation duration [s] |
| `TRAJECTORY` | `atp_trajectory.csv` | Path to an alternative trajectory file in the same format: time [s], azimuth, elevation and rotator [deg] |
| `KINEMATIC_COMPENSATION` | on | `0` runs the FEM without kinematic compensation |
| `MOUNT_FEEDFORWARD` | on | `0` turns the mount drives feed-forward off. The azimuth drives then saturate at about 242 s |
| `PUPIL_SAMPLING` | `513` | Pupil sampling of the crseo optical model (`gmt_dos-clients_crseo` feature) |
| `GMT_MODES_PATH` | | Directory of the GMT mirror modes files required by crseo (`gmt_dos-clients_crseo` feature) |

## Optical model (`gmt_dos-clients_crseo` feature)

```bash
GMT_MODES_PATH=<gmtMirrors directory> make run SIM_DURATION=120 FEATURES="cuda gmt_dos-clients_arrow gmt_dos-clients_crseo"
```

* **GIR rotation:** the M1 and M2 rigid body motions are rotated as rigid bodies about the
  optical axis by the GIR angle (`gir_rotation.rs`), as seen by an instrument on the GIR.
* **Optical model:** the crseo optical model ray traces the GMT every second with a J band
  source (1.215 µm) and images it on a camera: λ/(5D) DFT sampling binned 2×2, i.e. a 3.93 mas
  pixel scale over a 128×128 pixel frame (0.503" field-of-view). The exit pupil wavefront, the
  segment tip-tilt and the camera frame (`Frame<Host>`) are saved to
  `data/model_data_1000.parquet`, about 1.5 MB per second at 513×513 pupil sampling.
* **Flowchart:** the block of the optical model shows the GMT pupil (`gmt_pupil.rs`, written to
  `data/gmt_pupil.svg`) with the segments of the assembly highlighted, the assembly being set at
  build time with the `ASSEMBLY` environment variable (e.g. `ASSEMBLY=1,2,7`, all the segments by default).

## Make targets and scripts

| Target / script | Output |
|---|---|
| `make trajectory` (`scripts/atp_trajectory.py`) | `atp_trajectory.csv` |
| `make replay` | embeds the signals of `data/model_data_*.parquet` into the replay page `data/model_flowchart.html` (`flowchart-replay` feature), options in `SCOPE_OPTIONS`, the pixel size of the camera frame in `FRAME_PIXEL` (3.93 mas) and the time of the steps from `SAMPLING_FREQUENCY`, see `gmt_dos-actors_flowchart` |
| `make build` / `make run` | the integrated model and `data/model_data_1.parquet` (and `data/model_data_1000.parquet` with crseo) |
| `make plot` | all the plots below, in `figures/`, from `data/model_data_1.parquet` |
| `scripts/plot_encoders.py` | `trajectory_vs_encoders.png`: trajectory vs mount encoders, and tracking error |
| `scripts/plot_rbm.py` | `rigid_body_motions.png` (and `_minus_rigid.png`): M1 and M2 rigid body motions |
| `scripts/plot_orbits.py` | `rbm_orbits.png`, `rbm_orbits_s7.png` (4th argument `s7`): segment orbits in the pupil |
| `scripts/plot_gir_orbits.py` | `rbm_orbits_gir.png`: segment orbits seen from the GIR |
| `scripts/plot_stt_orbits.py` | `segment_tiptilt_orbits.png`: segment tip-tilt orbits from `STT_START`, with an optional end time as 5th argument |
| `scripts/plot_wavefronts.py` | first and last wavefronts from `data/model_data_1000.parquet` (crseo): `python3 scripts/plot_wavefronts.py data/model_data_1000.parquet figures/wavefronts_first_last.png` |

`scripts/plot_rbm.py`, `scripts/plot_orbits.py` and `scripts/plot_gir_orbits.py` use `scripts/rigid_body.py` to remove the
rigid telescope motion themselves. They were written for uncompensated rigid body motions. With
the kinematic compensation on, use them on a run with `KINEMATIC_COMPENSATION=0`.
