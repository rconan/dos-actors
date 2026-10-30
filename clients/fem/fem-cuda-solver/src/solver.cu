#include "solver.hpp"
#include <cstdio>
#include <cstdlib>

// abort with the failing call, its location and the error message
#define CUDA_CHECK(call)                                                       \
  do {                                                                         \
    cudaError_t err = (call);                                                  \
    if (err != cudaSuccess) {                                                  \
      fprintf(stderr, "fem-cuda-solver: %s failed at %s:%d: %s\n", #call,      \
              __FILE__, __LINE__, cudaGetErrorString(err));                    \
      abort();                                                                 \
    }                                                                          \
  } while (0)
#define CUBLAS_CHECK(call)                                                     \
  do {                                                                         \
    cublasStatus_t status = (call);                                            \
    if (status != CUBLAS_STATUS_SUCCESS) {                                     \
      fprintf(stderr, "fem-cuda-solver: %s failed at %s:%d: %s\n", #call,      \
              __FILE__, __LINE__, cublasGetStatusString(status));              \
      abort();                                                                 \
    }                                                                          \
  } while (0)

// state equation`
__global__ void step_kernel(mode_state_space *mss, double *v, double *y,
                            int n) {
  int i;
  double x0, x1;
  i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    x0 = mss[i].x0;
    x1 = mss[i].x1;
    // x0 <- a0 x0 + a2 x1 + b2 vi
    mss[i].x0 = mss[i].a0 * x0 + mss[i].a2 * x1 + mss[i].b2 * v[i];
    // x1 <- a1 x0 + a3 x1 + b3 vi
    mss[i].x1 = mss[i].a1 * x0 + mss[i].a3 * x1 + mss[i].b3 * v[i];
    y[i] = mss[i].x0;
  }
}

void state_space::build(int n_mode_, mode_state_space *mss, int n_input_,
                        double *i2m, int n_output_, double *m2o) {

  n_mode = n_mode_;
  n_input = n_input_;
  n_output = n_output_;
  d_dcg = NULL;
  CUDA_CHECK(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking));
  CUBLAS_CHECK(cublasCreate(&handle));
  CUBLAS_CHECK(cublasSetStream(handle, stream));
  CUDA_CHECK(cudaMalloc(&d_mss, n_mode * sizeof(mode_state_space)));
  CUDA_CHECK(cudaMemcpyAsync(d_mss, mss, n_mode * sizeof(mode_state_space),
                             cudaMemcpyHostToDevice, stream));
  CUDA_CHECK(cudaMalloc(&d_i2m, n_mode * n_input * sizeof(double)));
  CUDA_CHECK(cudaMemcpyAsync(d_i2m, i2m, n_mode * n_input * sizeof(double),
                             cudaMemcpyHostToDevice, stream));
  CUDA_CHECK(cudaMalloc(&d_m2o, n_mode * n_output * sizeof(double)));
  CUDA_CHECK(cudaMemcpyAsync(d_m2o, m2o, n_mode * n_output * sizeof(double),
                             cudaMemcpyHostToDevice, stream));
  CUDA_CHECK(cudaMalloc(&d_u, n_input * sizeof(double)));
  // the DC gain compensation is applied to the previous input, zero at start
  CUDA_CHECK(cudaMemsetAsync(d_u, 0, n_input * sizeof(double), stream));
  CUDA_CHECK(cudaMalloc(&d_v, n_mode * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_x0, n_mode * sizeof(double)));
  CUDA_CHECK(cudaMalloc(&d_y, n_output * sizeof(double)));
  CUDA_CHECK(cudaStreamSynchronize(stream));
}
void state_space::dc_gain_compensator(double *dcg) {
  CUDA_CHECK(cudaMalloc(&d_dcg, n_output * n_input * sizeof(double)));
  CUDA_CHECK(cudaMemcpyAsync(d_dcg, dcg, n_output * n_input * sizeof(double),
                             cudaMemcpyHostToDevice, stream));
  CUDA_CHECK(cudaStreamSynchronize(stream));
}
void state_space::free() {
  CUDA_CHECK(cudaStreamSynchronize(stream));
  CUBLAS_CHECK(cublasDestroy(handle));
  CUDA_CHECK(cudaStreamDestroy(stream));
  CUDA_CHECK(cudaFree(d_mss));
  CUDA_CHECK(cudaFree(d_i2m));
  CUDA_CHECK(cudaFree(d_m2o));
  CUDA_CHECK(cudaFree(d_u));
  CUDA_CHECK(cudaFree(d_v));
  CUDA_CHECK(cudaFree(d_x0));
  CUDA_CHECK(cudaFree(d_y));
  if (d_dcg != NULL)
    CUDA_CHECK(cudaFree(d_dcg));
}
void state_space::step(double *u, double *y) {
  double alpha = 1.0;
  double beta = 0.0;
  dim3 block(256); // or whatever block size you want
  dim3 grid((n_mode + block.x - 1) / block.x); // ceil(n/block.x)

  // y = Dc u, with u the input of the previous step (still in d_u)
  if (d_dcg != NULL) {
    CUBLAS_CHECK(cublasDgemv(handle, CUBLAS_OP_N, n_output, n_input, &alpha,
                             d_dcg, n_output, d_u, 1, &beta, d_y, 1));
  }

  CUDA_CHECK(cudaMemcpyAsync(d_u, u, n_input * sizeof(double),
                             cudaMemcpyHostToDevice, stream));
  // v = Bu
  CUBLAS_CHECK(cublasDgemv(handle, CUBLAS_OP_T, n_input, n_mode, &alpha, d_i2m,
                           n_input, d_u, 1, &beta, d_v, 1));

  // update state equation
  step_kernel<<<grid, block, 0, stream>>>(d_mss, d_v, d_x0, n_mode);
  CUDA_CHECK(cudaGetLastError());

  // y = Cx0 (+ Dc u)
  if (d_dcg != NULL)
    beta = 1.0;
  CUBLAS_CHECK(cublasDgemv(handle, CUBLAS_OP_N, n_output, n_mode, &alpha,
                           d_m2o, n_output, d_x0, 1, &beta, d_y, 1));

  CUDA_CHECK(cudaMemcpyAsync(y, d_y, n_output * sizeof(double),
                             cudaMemcpyDeviceToHost, stream));
  CUDA_CHECK(cudaStreamSynchronize(stream));
}
