#ifndef GPU_PATH_CUH_
#define GPU_PATH_CUH_

struct GpuPathPoint
{
    uint16_t x;
    uint16_t y;
    uint16_t length;
    int16_t avrg_cost;

    __device__ inline uint32_t cost()
    {
        return length * avrg_cost;
        // return length * avrg_cost * avrg_cost / 1024;
    }
};

struct GpuPath
{
    GpuPathPoint p[GPU_PATH_MAX_SIZE];

    int total_size;

    int total_cost;
};

#endif
