#ifndef GPU_PATH_CUH_
#define GPU_PATH_CUH_

struct GpuPathPoint
{
    uint16_t x;
    uint16_t y;
    uint32_t cost;
};

struct GpuPath
{
    GpuPathPoint p[GPU_PATH_MAX_SIZE];

    int total_size;

    int total_cost;
};

#endif
