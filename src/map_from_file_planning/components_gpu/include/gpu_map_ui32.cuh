#ifndef GPU_MAP_UI32_CUH_
#define GPU_MAP_UI32_CUH_

#include <cuda.h>
#include "gpu_errchk.cuh"

class GpuMapUI32
{
public:
    int size_x;
    int size_y;

    uint32_t* data;

public:

    GpuMapUI32();
    GpuMapUI32(int size_x, int size_y);
    GpuMapUI32(int size_x, int size_y, const uint32_t fill_value);

    void allocate(int size_x, int size_y);
    void resize(int size_x, int size_y);
    void fill(const uint32_t fill_value);
    void drawCircle(const uint32_t fill_value, const int pose_x, const int pose_y, const float radius);
    void drawBorders(const uint32_t fill_value, const int thickness);
    void release();

    int size(){return size_x * size_y;}

};

#endif
