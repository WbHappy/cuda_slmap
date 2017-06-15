#ifndef GPU_MAP_I16_CUH_
#define GPU_MAP_I16_CUH_

#include <cuda.h>
#include "gpu_errchk.cuh"

class GpuMapI16
{
public:
    int size_x;
    int size_y;

    int16_t* data;

public:

    GpuMapI16();
    GpuMapI16(int size_x, int size_y);
    GpuMapI16(int size_x, int size_y, const int16_t fill_value);

    void allocate(int size_x, int size_y);
    void resize(int size_x, int size_y);
    void fill(const int16_t fill_value);
    void drawCircle(const int16_t fill_value, const int pose_x, const int pose_y, const float radius);
    void drawBorders(const int16_t fill_value, const int thickness);
    void release();

    int size(){return size_x * size_y;}

};

#endif
