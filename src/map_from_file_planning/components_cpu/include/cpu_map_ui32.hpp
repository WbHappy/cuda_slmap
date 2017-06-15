#ifndef CPU_MAP_UI32_CUH_
#define CPU_MAP_UI32_CUH_

#include "common.hpp"
#include "opencv2/opencv.hpp"

class CpuMapUI32
{
public:
    int size_x;
    int size_y;

    uint32_t* data;

    cv::Mat img;

public:

    CpuMapUI32();
    CpuMapUI32(int size_x, int size_y);
    CpuMapUI32(int size_x, int size_y, const uint32_t fill_value);

    void allocate(int size_x, int size_y);
    void resize(int size_x, int size_y);
    void fill(const uint32_t fill_value);
    void release();

    int size(){return size_x * size_y;}

    void display(std::string win_name);

    void info();
};

#endif
