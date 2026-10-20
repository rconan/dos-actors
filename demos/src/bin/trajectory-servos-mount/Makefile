# ATP tracking trajectory driving the GMT mount (trajectory-servos-mount)
#
#   make trajectory  compute atp_trajectory.csv with astropy (scripts/atp_trajectory.py)
#   make build       build the integrated model
#   make run         run the integrated model, saves data/model_data_1.parquet
#   make plot        plot (in figures/) the trajectory against the mount encoders, the M1 & M2 rigid body motions
#                    and the segment orbits, and the segment tip-tilt orbits

# FEM 2nd order model
FEM_REPO ?= /media/rconan/b79153ac-7908-4e0f-9d29-fdfb542910ca/20251110_1617_zen_30_M1_202110_FSM_202305_Mount_202305_pier_202411_M1Fans
# mount control model: MOUNT_PDR_8kHz, MOUNT_FDR_1kHz, MOUNT_FDR_1kHz-az17Hz or MOUNT_FDR_8kHz
MOUNT_MODEL ?= MOUNT_FDR_1kHz
# mount sampling frequency [Hz], must match MOUNT_MODEL
SAMPLING_FREQUENCY ?= 1000
# CUDA compiler for the FEM CUDA solver
CUDACXX ?= /usr/local/cuda/bin/nvcc
# simulation duration [s], up to 300 (length of the trajectory)
SIM_DURATION ?= 30
# flowchart layout of the model: dot, neato, fdp (Graphviz) or elk (laid out in the browser), empty: no flowchart
FLOWCHART ?= dot
# start time [s] of the segment tip-tilt orbit plot
STT_START ?= 10
# cargo features: drop cuda to use the CPU FEM solver
FEATURES ?= cuda gmt_dos-clients_arrow
# flowchart-scope options of make replay, e.g. --signals MountEncoders,Wavefront --max-size 128 --steps 0..10000 --shape 'Frame<Host>=128x128'
SCOPE_OPTIONS ?=
# pixel size of the camera frame (Frame<Host>) of the crseo optical model, for the field of view in the frame scope
FRAME_PIXEL ?= 3.93mas

BIN = trajectory-servos-mount
PYTHON ?= python3
export FEM_REPO MOUNT_MODEL CUDACXX SIM_DURATION FLOWCHART

.PHONY: trajectory build run plot replay

trajectory:
	$(PYTHON) scripts/atp_trajectory.py atp_trajectory.csv

build:
	cargo build --release -p demos --features "$(FEATURES)" --bin $(BIN)

run:
	cargo run --release -p demos --features "$(FEATURES)" --bin $(BIN)

plot:
	mkdir -p figures
	$(PYTHON) scripts/plot_encoders.py atp_trajectory.csv data/model_data_1.parquet figures/trajectory_vs_encoders.png $(SAMPLING_FREQUENCY)
	$(PYTHON) scripts/plot_rbm.py data/model_data_1.parquet figures/rigid_body_motions.png $(SAMPLING_FREQUENCY)
	$(PYTHON) scripts/plot_orbits.py data/model_data_1.parquet figures/rbm_orbits.png $(SAMPLING_FREQUENCY)
	$(PYTHON) scripts/plot_orbits.py data/model_data_1.parquet figures/rbm_orbits_s7.png $(SAMPLING_FREQUENCY) s7
	$(PYTHON) scripts/plot_gir_orbits.py data/model_data_1.parquet figures/rbm_orbits_gir.png $(SAMPLING_FREQUENCY)
	$(PYTHON) scripts/plot_stt_orbits.py data/model_data_1.parquet figures/segment_tiptilt_orbits.png $(SAMPLING_FREQUENCY) $(STT_START)

# embeds the signals of the parquet files into the replay page data/model_flowchart.html (run with the feature flowchart-replay)
replay:
	cargo run --release -p gmt_dos-actors_flowchart --features scope --bin flowchart-scope -- data/model_flowchart.html $(wildcard data/model_data_*.parquet) --sampling-frequency $(SAMPLING_FREQUENCY) --pixel 'Frame<Host>=$(FRAME_PIXEL)' $(SCOPE_OPTIONS)
