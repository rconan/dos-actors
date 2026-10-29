#include <cublas_v2.h>

struct mode_state_space {
  double x0;
  double x1;
  double a0;
  double a1;
  double a2;
  double a3;
  double b2;
  double b3;
};
struct state_space {
  int n_mode, n_input, n_output;
  double *d_i2m, *d_m2o, *d_u, *d_v, *d_x0, *d_y, *d_dcg;
  cublasHandle_t handle;
  cudaStream_t stream;
  mode_state_space *d_mss;
  void build(int n_mode, mode_state_space *mss, int n_input, double *i2m,
             int n_output, double *m2o);
  void dc_gain_compensator(double *dcg);
  void free();
  void step(double *u, double *y);
};
