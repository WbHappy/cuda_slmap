#ifndef _ROBOT_PLANNER_MAPS_CUH_
#define _ROBOT_PLANNER_MAPS_CUH_


#include "components_cpu.hpp"
#include "components_gpu.cuh"

#include "opencv2/opencv.hpp"

class _RobotPlannerMaps
{
public:
// DEVICE MEMORY
    GpuMapI16 dev_heightmap;
    GpuMapI16 dev_costmap;

    float* dev_debug;


// HOST MEMORY

    CpuMapI16 host_heightmap;
    CpuMapI16 host_costmap;

    float* host_debug;

// HOST VARIABLES & PARAMETERS

    int robot_onmap_x;
    int robot_onmap_y;

    int goal_onmap_x;
    int goal_onmap_y;

    float map_orient;       // Clock counter-wise angle between East and vector from Start to End points
    int map_scale;        // One meter in real world is equal to this number in map pixel position (X,Y)
    int height_scale;     // One meter in real world is equal to this number in map pixel value (Z)

    int map_pow2_divider;   // Map size must be divisible by this number
    int map_meters_offset;     // Minimum number of fields between Start/Stop points and edge of map
    int map_offset_pix;

public:

    _RobotPlannerMaps();
    ~_RobotPlannerMaps();

    void allocateConstSizeMemory();
    void allocateMaps(float x_deviation_meters, float y_deviation_meters);

    void freeConstSizeMemory();
    void freeMaps();

    void resizeMaps(float x_deviation_meters, float y_deviation_meters);

    void updateRobotPoseOnMap(float point_x, float point_y);
    void updateGoalPoseOnMap(float point_x, float point_y);

    void setCustomRobotPoseOnMap(int point_x, int point_y);
    void setCustomGoalPoseOnMap(int point_x, int point_y);

    void cudaDebugInfo();
};

#endif
