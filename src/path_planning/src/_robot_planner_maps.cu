#include "../include/_robot_planner_maps.cuh"

_RobotPlannerMaps::_RobotPlannerMaps()
{
    dev_heightmap = GpuMapI16();
    dev_costmap = GpuMapI16();
}


_RobotPlannerMaps::~_RobotPlannerMaps()
{
    freeMaps();
}


void _RobotPlannerMaps::allocateMaps(float target_east, float target_north)
{
    target_east *= map_scale;
    target_north *= map_scale;

    map_orient = atan2(target_north, target_east);
    map_offset_pix = roundUpTo(map_pow2_divider, map_meters_offset*map_scale);


    int dist_xy = (int) (sqrt(target_east*target_east + target_north*target_north));
    int map_size_x = roundUpTo(map_pow2_divider, dist_xy) + 2 * roundUpTo(map_pow2_divider, map_offset_pix);
    int map_size_y = 2 * roundUpTo(map_pow2_divider, map_offset_pix);


    printf("angle form East = %f\n", map_orient);
    printf("dist_xy = %d\n", dist_xy);
    printf("map_size_x = %d\n", map_size_x);
    printf("map_size_y = %d\n", map_size_y);

    dev_heightmap.allocate(map_size_y, map_size_x);
    dev_costmap.allocate(map_size_y, map_size_x);

    dev_heightmap.fill(UNKNOWN);
    dev_costmap.fill(0);


    host_heightmap.allocate(map_size_y, map_size_x);
    host_costmap.allocate(map_size_y, map_size_x);

    host_heightmap.fill(127);
    host_costmap.fill(127);


// DEBUG
    gpuErrchk(cudaMalloc((void**)&dev_debug, 1024 * sizeof(float)) );
    host_debug = (float*)malloc(1024 * sizeof(float));

}


void _RobotPlannerMaps::freeMaps()
{
    dev_heightmap.release();
    dev_costmap.release();

    host_heightmap.release();
    host_costmap.release();

    gpuErrchk( cudaFree(dev_debug) );
}



void _RobotPlannerMaps::resizeMaps(float target_east, float target_north)
{
    freeMaps();
    allocateMaps(target_east, target_north);
}

// Mirrored from __device__ pointWorldToMap()
void _RobotPlannerMaps::updateRobotPoseOnMap(float point_x, float point_y)
{
    float point_orient = atan2f(point_y, point_x);
    float point_dist = sqrtf(point_x*point_x + point_y*point_y);

    this->robot_onmap_x = (int) (sinf(map_orient - point_orient) * point_dist * map_scale + map_offset_pix);
    this->robot_onmap_y = (int) (cosf(map_orient - point_orient) * point_dist * map_scale + map_offset_pix);
}

void _RobotPlannerMaps::updateGoalPoseOnMap(float point_x, float point_y)
{
    float point_orient = atan2f(point_y, point_x);
    float point_dist = sqrtf(point_x*point_x + point_y*point_y);

    this->goal_onmap_x = (int) (sinf(map_orient - point_orient) * point_dist * map_scale + map_offset_pix);
    this->goal_onmap_y = (int) (cosf(map_orient - point_orient) * point_dist * map_scale + map_offset_pix);
}


void _RobotPlannerMaps::setCustomRobotPoseOnMap(int point_x, int point_y)
{
    this->robot_onmap_x = point_x;
    this->robot_onmap_y = point_y;
}

void _RobotPlannerMaps::setCustomGoalPoseOnMap(int point_x, int point_y)
{
    this->goal_onmap_x = point_x;
    this->goal_onmap_y = point_y;
}


void _RobotPlannerMaps::cudaDebugInfo()
{

    gpuErrchk( cudaMemcpy(host_debug, dev_debug, 1024 * sizeof(float), cudaMemcpyDeviceToHost) );

    printf("==== CUDA DEBUG ====\n");
    for(int i = 0; i < 1024; i++)
    {
        printf("%f\n", host_debug[i]);
    }
    printf("====================\n");
}
