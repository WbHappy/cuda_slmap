#ifndef GPU_001_LIDAR_MAPPING_CUH_
#define GPU_001_LIDAR_MAPPING_CUH_

#include "_robot_planner_maps.cuh"
#include "_ros_buffor.hpp"

#include "components_cpu.hpp"
#include "components_gpu.cuh"


#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cuda.h>

// CPU function that calculates direct kinematics matrix
// of full transformation from world to lidar
HTMatrix dkWorldToLidar(
    double tx,
    double ty,
    double tz,
    double qx,
    double qy,
    double qz,
    double qw,
    double th2,
    const double a1,
    const double d2,
    const double al3);

// CPU function that calculates direct kinematics matrix
// of simplified transformation from world to lidar
// Simplified matrix has only 9 fields used in later calulations
HTMatrixLidarCPU dkWorldToLidarReduced(
    double tx,
    double ty,
    double tz,
    double qx,
    double qy,
    double qz,
    double qw,
    double th2,
    const double a1,
    const double d2,
    const double al3);

class GpuLidarMapping
{
    _RobotPlannerMaps *_rpm;
    _ROSBuffor *_ros;

    float* dev_dk_matrix;       // gpu memory for direct kinematics matrix (world to Lidar)
    float* dev_laser_scan;      // gpu memory for laser scan

    HTMatrixLidarCPU dk_cpu;    // cpu matrix of direct kinematics (world to Lidar)

public:
    int laser_rays;         // Number of lidar's laser rays

    float angle_min;        // Angle of first scan form the rigth
    float angle_max;        // Angle of first scan from the left

    float dk_a1;    // Offset from rover center to LiDAR tower Z-axis
    float dk_d2;    // Height from rover center to LiDAR scanner
    float dk_al3;   // Angle of LiDAR tilt in its Y-axis

    // needed on the begin, to estimate height of area under rover, which cannot be mapped without moving
    int init_circle_height;     // Initial height of circle - should be equal to height from rover center to bottom of wheel (negative number)
    float init_circle_radius;   // Radius of start circle within which pixels are set to init_circle_height value

public:
    GpuLidarMapping(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros);

    void allocateMemory(int laser_rays, float angle_min, float angle_max);
    void freeMemory();

    void drawInitialHeightmapCircle();

    void copyInputToDevice();
    void executeKernel();
    void copyOutputToHost();

    void display();
};

// GPU function to calculate angle of scan depending on ID of GPU Thread
__device__ inline float calcLaserAngle(
    int laser_rays,
    float angle_min,
    float angle_max,
    int tid);


// GPU function to calculate direct kinematics from World to Scan,
// with given kinematics from World to Lidar
__device__ inline Point3F32 dkLidarToScan(
    const HTMatrixLidarCPU *dk_cpu,
    float th5,
    float a5);

// GPU function to transform given point from Real World into point on GPU Map
__device__ inline Point2I32 pointWorldToMap(
    float point_x,
    float point_y,
    float map_orient,
    float map_scale,
    float map_offset_pix);

// GPU Kernel to used for LiDAR Mapping -> Creates raw heightmap of terrain
__global__ void lidarMappingKernel(
    float* laser_scan,
    const HTMatrixLidarCPU dk_cpu,
    const int laser_rays,
    const float angle_min,
    const float angle_max,
    int16_t* heightmap,
    const int map_x,
    const int map_y,
    const int height_scale,
    const int map_scale,
    const float map_orient,
    const float map_offset_pix,
    float* debug);


#endif
