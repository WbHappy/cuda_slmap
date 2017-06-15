#include "../include/gpu_map_ui16.cuh"

__global__ void fillValueKernel(uint16_t* data, const uint16_t fill_value)
{
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    int idy = blockDim.y * blockIdx.y + threadIdx.y;
    int tid = idy + idx * gridDim.y * blockDim.y;

    data[tid] = fill_value;
}

__global__ void drawCircleKernel(uint16_t* data, const uint16_t fill_value, const int pose_x, const int pose_y, const float radius)
{
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    int idy = blockDim.y * blockIdx.y + threadIdx.y;
    int tid = idy + idx * gridDim.y * blockDim.y;

    int dist_x = (idx - pose_x);
    int dist_y = (idy - pose_y);

    float dist_from_center = sqrtf((float)dist_x*dist_x + (float)dist_y*dist_y);

    if(dist_from_center <= radius)
    {
        data[tid] = fill_value;
    }

}

__global__ void drawBordersKernel(uint16_t* data, const uint16_t fill_value, const int thickness)
{
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    int idy = blockDim.y * blockIdx.y + threadIdx.y;
    int tid = idy + idx * gridDim.y * blockDim.y;

    if(
        idx < thickness ||
        idx > gridDim.x * blockDim.x - thickness ||
        idy < thickness ||
        idy > gridDim.y * blockDim.y - thickness
    )
    {
        data[tid] = fill_value;
    }

}

GpuMapUI16::GpuMapUI16()
{
    size_x = 0;
    size_y = 0;
}

GpuMapUI16::GpuMapUI16(int size_x, int size_y)
{
    this->resize(size_x, size_y);
}


GpuMapUI16::GpuMapUI16(int size_x, int size_y, const uint16_t fill_value)
{
    this->resize(size_x, size_y);
    this->fill(fill_value);
}


void GpuMapUI16::allocate(int size_x, int size_y)
{
    this->size_x = size_x;
    this->size_y = size_y;

    gpuErrchk( cudaMalloc((void**)&this->data, size_x * size_y * sizeof(uint16_t)) );
}


void GpuMapUI16::resize(int size_x, int size_y)
{
    release();
    allocate(size_x, size_y);
}


void GpuMapUI16::fill(const uint16_t fill_value)
{
    int block_x = 32;
    int block_y = 32;

    int grid_x = (size_x + block_x - 1) / block_x;
    int grid_y = (size_y + block_y - 1) / block_y;
    dim3 grid(grid_x, grid_y, 1);
    dim3 block(block_x, block_y, 1);

    fillValueKernel<<< grid, block >>> (this->data, fill_value);
    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaDeviceSynchronize() );

}

void GpuMapUI16::drawCircle(const uint16_t fill_value, const int pose_x, const int pose_y, const float radius)
{
    int block_x = 32;
    int block_y = 32;

    int grid_x = (size_x + block_x - 1) / block_x;
    int grid_y = (size_y + block_y - 1) / block_y;
    dim3 grid(grid_x, grid_y, 1);
    dim3 block(block_x, block_y, 1);

    drawCircleKernel<<< grid, block >>> (this->data, fill_value, pose_x, pose_y, radius);
    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaDeviceSynchronize() );

}


void GpuMapUI16::drawBorders(const uint16_t fill_value, const int thickness)
{
    int block_x = 32;
    int block_y = 32;

    int grid_x = (size_x + block_x - 1) / block_x;
    int grid_y = (size_y + block_y - 1) / block_y;
    dim3 grid(grid_x, grid_y, 1);
    dim3 block(block_x, block_y, 1);

    drawBordersKernel<<< grid, block >>> (this->data, fill_value, thickness);
    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaDeviceSynchronize() );

}

void GpuMapUI16::release()
{
    gpuErrchk( cudaFree(this->data) );
}
