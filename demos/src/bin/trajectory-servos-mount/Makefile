# ATP tracking trajectory driving the GMT mount (trajectory-servos-mount)
#
#   make trajectory  compute atp_trajectory.csv with astropy
#   make build       build the integrated model
#   make run         run the integrated model, saves model_data_1.parquet
#   make plot        plot the trajectory against the mount encoders, the M1 & M2 rigid body motions
#                    and the segment orbits

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
# cargo features: drop cuda to use the CPU FEM solver
FEATURES ?= cuda gmt_dos-clients_arrow

BIN = trajectory-servos-mount
PYTHON ?= python3
export FEM_REPO MOUNT_MODEL CUDACXX SIM_DURATION

.PHONY: trajectory build run plot

trajectory:
	$(PYTHON) atp_trajectory.py atp_trajectory.csv

build:
	cargo build --release -p demos --features "$(FEATURES)" --bin $(BIN)

run:
	cargo run --release -p demos --features "$(FEATURES)" --bin $(BIN)

plot:
	$(PYTHON) plot_encoders.py atp_trajectory.csv model_data_1.parquet trajectory_vs_encoders.png $(SAMPLING_FREQUENCY)
	$(PYTHON) plot_rbm.py model_data_1.parquet rigid_body_motions.png $(SAMPLING_FREQUENCY)
	$(PYTHON) plot_orbits.py model_data_1.parquet rbm_orbits.png $(SAMPLING_FREQUENCY)
	$(PYTHON) plot_orbits.py model_data_1.parquet rbm_orbits_s7.png $(SAMPLING_FREQUENCY) s7
	$(PYTHON) plot_gir_orbits.py model_data_1.parquet rbm_orbits_gir.png $(SAMPLING_FREQUENCY)
