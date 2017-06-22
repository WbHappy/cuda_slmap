#ifndef CPU_MAP_I16_CUH_
#define CPU_MAP_I16_CUH_

#include "common.hpp"
#include "opencv2/opencv.hpp"

class CpuMapI16
{
public:
    int size_x;
    int size_y;

    int16_t* data;

    cv::Mat img;

public:

    CpuMapI16();
    CpuMapI16(int size_x, int size_y);
    CpuMapI16(int size_x, int size_y, const int16_t fill_value);

    void allocate(int size_x, int size_y);
    void resize(int size_x, int size_y);
    void fill(const int16_t fill_value);
    void release();

    int size(){return size_x * size_y;}

    void display(std::string win_name);

    void drawEpiosde(std::string win_name, const uint8_t brightness, const int x1, const int y1, const int x2, const int y2);

    void info();
};

#endif
