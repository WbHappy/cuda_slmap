#ifndef GPU_003_PATH_PLANNING_CUH_
#define GPU_003_PATH_PLANNING_CUH_

#include "_robot_planner_maps.cuh"
#include "_ros_buffor.hpp"

#include "components_cpu.hpp"
#include "components_gpu.cuh"

#include <cuda.h>

#define PLANNER_EPISODE_DIVISIONS 1     // Number of new points created during division (1 -> episode is divided on 2 parts)

class GpuPathPlanning
{
    _RobotPlannerMaps *_rpm;
    _ROSBuffor *_ros;

    GpuPath* dev_path;                 // GPU memory for storing paths
    GpuPath* host_path;                // CPU memory for storing paths

    float* dev_debug;
    float* host_debug;

public:
    int planner_max_iteration;          // Maximum number of planning iterations (divide, optimize, mutate)

    int planner_concurrent_paths;       // Number of concurent paths calculated - equal to number of CUDA blocs

    int planner_threads_per_path;       // Number of threads used in one path - equal to number of divisions/mutations per episode/point

    int planner_cost_sampling;          // Distance between consecutive points taken to episode cost calculation in pixels

    int planner_min_division_length;    // Episodes under this length will not be divided


public:
    GpuPathPlanning(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros);

    void allocateMemory();

    void copyInputToDevice();
    void executeKernel();
    void copyOutputToHost();

    void display();
};

// GPU function to copy path between two pointers
// Use single thread to copy points
__device__ inline void copyPath_Singlethread(
    GpuPath *new_path,
    GpuPath *old_path);

// GPU function to copy path between two pointers
// Use multiple threads in on block to copy path
__device__ inline void copyPath_Multithread(
    GpuPath *new_path,
    GpuPath *old_path,
    int sid,
    int threads_no);

// GPU function that generates pseudo-random point
__device__ inline GpuPathPoint generateRandomPoint(
    const GpuPathPoint *p1,
    const GpuPathPoint *p2,
    int ep_div_no,
    int ep_div_total,
    int sid,
    int threads_no);

__device__ inline void addPathPoints_Multithread(
    GpuPath *new_path,
    GpuPath *old_path,
    GpuPathPoint *new_points,
    int sid,
    int threads_no,
    int new_points_index,
    float *dev_debug);

// Calcualtes cost of traveling via episode
__device__ inline int calcEpisodeCost_Singlethread(
    int16_t *costmap,
    const int map_x,
    const int map_y,
    const GpuPathPoint p1,
    const GpuPathPoint p2,
    const int sampling);

__device__ inline void divideEpisode_Multithread(
    int16_t *costmap,
    const int map_x,
    const int map_y,
    const GpuPathPoint p1,
    const GpuPathPoint p2,
    const int sampling,
    const int sid,
    const int threads_no);

__global__ void pathPlanningKernel(
    GpuPath* dev_paths,
    int16_t *costmap,
    const int map_x,
    const int map_y,
    const GpuPathPoint odom,
    const GpuPathPoint goal,
    const int sampling,
    const int max_iteration,
    const int min_division_length,
    float *dev_debug);


#endif
