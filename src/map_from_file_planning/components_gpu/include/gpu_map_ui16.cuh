#ifndef GPU_MAP_UI16_CUH_
#define GPU_MAP_UI16_CUH_

#include <cuda.h>
#include "gpu_errchk.cuh"

class GpuMapUI16
{
public:
    int size_x;
    int size_y;

    uint16_t* data;

public:

    GpuMapUI16();
    GpuMapUI16(int size_x, int size_y);
    GpuMapUI16(int size_x, int size_y, const uint16_t fill_value);

    void allocate(int size_x, int size_y);
    void resize(int size_x, int size_y);
    void fill(const uint16_t fill_value);
    void drawCircle(const uint16_t fill_value, const int pose_x, const int pose_y, const float radius);
    void drawBorders(const uint16_t fill_value, const int thickness);
    void release();

    int size(){return size_x * size_y;}

};

#endif
