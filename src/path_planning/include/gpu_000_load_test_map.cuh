#ifndef GPU_000_LOAD_TEST_MAP_CUH_
#define GPU_000_LOAD_TEST_MAP_CUH_

#include "_robot_planner_maps.cuh"
#include "_ros_buffor.hpp"

#include "components_cpu.hpp"
#include "components_gpu.cuh"

#include <cuda.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/package.h>

class GpuLoadTestMap
{
    _RobotPlannerMaps *_rpm;
    _ROSBuffor *_ros;

    cv::Mat cv_cmap;
    cv::Mat cv_hmap;


public:
    GpuLoadTestMap(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros);

    void readHeightMapFromFile8(std::string path);
    void readHeightMapFromFile16(std::string path);

    void readCostMapFromFile8(std::string path);
    void readCostMapFromFile16(std::string path);

    void allocateMaps();

    void copyInputToDevice();

    void display();

};

#endif
