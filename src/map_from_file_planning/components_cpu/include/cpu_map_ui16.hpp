#ifndef CPU_MAP_UI16_CUH_
#define CPU_MAP_UI16_CUH_

#include "common.hpp"
#include "opencv2/opencv.hpp"

class CpuMapUI16
{
public:
    int size_x;
    int size_y;

    uint16_t* data;

    cv::Mat img;

public:

    CpuMapUI16();
    CpuMapUI16(int size_x, int size_y);
    CpuMapUI16(int size_x, int size_y, const uint16_t fill_value);

    void allocate(int size_x, int size_y);
    void resize(int size_x, int size_y);
    void fill(const uint16_t fill_value);
    void release();

    int size(){return size_x * size_y;}

    void display(std::string win_name);

    void info();
};

#endif
