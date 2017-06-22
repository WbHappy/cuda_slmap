#ifndef MY_GPU_TYPES_CUH_
#define MY_GPU_TYPES_CUH_

#include <cuda.h>

struct Point3F32
{
    float x;
    float y;
    float z;
};

struct Point2F32
{
    float x;
    float y;
};

struct Point2I32
{
    int x;
    int y;
};

struct Point2UI16
{
    uint16_t x;
    uint16_t y;
};

struct HTMatrixLidarCPU
{
    float m_0;
    float m_1;
    float m_3;

    float m_4;
    float m_5;
    float m_7;

    float m_8;
    float m_9;
    float m_11;
};

#endif
