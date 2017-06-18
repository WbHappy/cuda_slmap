#ifndef GPU_003_PATH_PLANNING_CUH_
#define GPU_003_PATH_PLANNING_CUH_

#include "_robot_planner_maps.cuh"
#include "_ros_buffor.hpp"

#include "components_cpu.hpp"
#include "components_gpu.cuh"

#include <cuda.h>
#include <curand.h>
#include <curand_kernel.h>

#include <sys/time.h>

class GpuPathPlanning
{
    _RobotPlannerMaps *_rpm;
    _ROSBuffor *_ros;

    GpuPath* dev_path;                 // GPU memory for storing paths
    GpuPath* host_path;                // CPU memory for storing paths

    timeval host_time;

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
    GpuPath *path_output,
    GpuPath *path_input);

// GPU function to copy path between two pointers
// Use multiple threads in on block to copy path
__device__ inline void copyPath_Multithread(
    GpuPath *path_output,
    GpuPath *path_input,
    int sid,
    int threads_no);

// GPU function that generates pseudo-random point
// Reference point pose is betweem two points
__device__ inline GpuPathPoint generateDividePoint_Singlethread(
    int16_t *costmap,
    const GpuPathPoint *p1,
    const GpuPathPoint *p2,
    const int std_dev,
    const int map_x,
    const int map_y,
    const int sid,
    const int threads_no,
    const int sampling,
    curandState_t *curand_state);

// GPU function that generates pseudo-random point
// Reference point pose is inside point
__device__ inline GpuPathPoint generateMutatePoint_Singlethread(
    int16_t *costmap,
    const GpuPathPoint *p1,
    const GpuPathPoint *p2,
    const int std_dev,
    const int map_x,
    const int map_y,
    const int sid,
    const int threads_no,
    const int sampling,
    curandState_t *curand_state);

__device__ inline void addPathPoints_Multithread(
    GpuPath *path_output,
    GpuPath *path_input,
    GpuPathPoint *new_points,
    int sid,
    int threads_no,
    int new_points_index);

// Calcualtes cost of traveling via episode
__device__ inline int16_t calcEpisodeAvrgCost_Singlethread(
    int16_t *costmap,
    const int map_x,
    const int map_y,
    const GpuPathPoint *p1,
    const GpuPathPoint *p2,
    const int sampling);

// Calcualtes cost of traveling via episode
__device__ inline uint16_t calcEpisodeLength_Singlethread(
    const GpuPathPoint *p1,
    const GpuPathPoint *p2);

//
__device__ inline void updatePathCost_Multithread(
    GpuPath *path_input,
    int sid,
    int threads_no
);

//aqq
__device__ inline int findCheapestThreadPath(
    uint32_t *new_points_costs,
    int sid,
    int threads_no);


//aqq
__device__ inline void dividePath_Multithread(
    int16_t *costmap,
    const int map_x,
    const int map_y,
    GpuPath *path_input,
    GpuPath *path_output,
    GpuPathPoint *new_points_buff,
    uint32_t *new_points_costs,
    curandState_t *curand_state,
    const int std_dev,
    const int sampling,
    const int sid,
    const int threads_no,
    float* dev_debug);

//aqq
__device__ inline void mutatePath_Multithread(
    int16_t *costmap,
    const int map_x,
    const int map_y,
    GpuPath *path_input,
    GpuPath *path_output,
    GpuPathPoint *new_points_buff,
    uint32_t *new_points_costs,
    curandState_t *curand_state,
    const int std_dev,
    const int sampling,
    const int sid,
    const int threads_no,
    float* dev_debug);

//aqq
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
    const int global_seed,
    float *dev_debug);

#endif
