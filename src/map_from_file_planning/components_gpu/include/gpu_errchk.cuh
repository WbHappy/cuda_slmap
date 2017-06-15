#ifndef GPU_ERRCHK_CUH_
#define GPU_ERRCHK_CUH_

#include <stdio.h>
#include <cuda.h>
#include <driver_types.h>

#define gpuErrchk(ans) {gpuAssert((ans), __FILE__, __LINE__, true); }

void gpuAssert(cudaError_t code, const char *file, int line, bool abort);

#endif
