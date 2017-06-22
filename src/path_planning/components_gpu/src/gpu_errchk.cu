#include "../include/gpu_errchk.cuh"

void gpuAssert(cudaError_t code, const char *file, int line, bool abort)
{
    if(code != cudaSuccess)
    {
        fprintf(stderr, "GPU Assert: %s \nIn file: %s, line: %d \n", cudaGetErrorString(code), file, line);
        if(abort) exit(code);
    }
}
