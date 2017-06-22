#ifndef GPU_002_COST_MAPPING_CUH_
#define GPU_002_COST_MAPPING_CUH_

#include "_robot_planner_maps.cuh"
#include "_ros_buffor.hpp"

#include "components_cpu.hpp"
#include "components_gpu.cuh"


#include <cuda.h>

class GpuCostMapping
{
    _RobotPlannerMaps *_rpm;
    _ROSBuffor *_ros;


public:
    float cmap_refresh_radius_meters;  // Radius of area, within which costmap is refreshed in single iteration
    int cmap_refresh_radius_pix;

    int cost_mask_radius;   // "Radius" of square making mask for pixel cost calculation (for 31x31 square "Radius" will be 16)
    int unknown_field_cost;   // Cost of empty pixel

    int costmap_borders_value; // Value assigned to borders of costmap - it should be high enought to not allow rover to travell there

public:
    GpuCostMapping(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros);

    void drawInitialCostmapBorders();

    void copyInputToDevice();
    void executeKernel();
    void copyOutputToHost();

    void display();
};


// GPU Function to "Localize" position of GPU Thread on Global GPU Map.
// It assumes that Rover's position is the middle of GPU Grid
__device__ inline Point2I32 threadOnGlobalMap(int idx, int idy, int pose_x, int pose_y, int cmap_refresh_radius_pix);

// GPU Functions to support shared memory filling pattern
// Transforms position of GPU Thread on Global GPU Map
// to corresponding position in sector 1,2,3,4 on Global GPU Map (not a mistake)
__device__ inline int threadGlobalSector1(Point2I32 pix, int block_x, int block_y, int map_y, int cost_mask_radius);
__device__ inline int threadGlobalSector2(Point2I32 pix, int block_x, int block_y, int map_y, int cost_mask_radius);
__device__ inline int threadGlobalSector3(Point2I32 pix, int block_x, int block_y, int map_y, int cost_mask_radius);
__device__ inline int threadGlobalSector4(Point2I32 pix, int block_x, int block_y, int map_y, int cost_mask_radius);



// GPU Functions to support shared memory filling pattern
// Transforms position of GPU Thread in Block (local thread)
// to corresponding index in sector 1,2,3,4 of GPU Shared Memory
__device__ inline int threadSharedSector1(int sidx, int sidy, int block_x, int block_y, int shared_dim_y);
__device__ inline int threadSharedSector2(int sidx, int sidy, int block_x, int block_y, int shared_dim_y);
__device__ inline int threadSharedSector3(int sidx, int sidy, int block_x, int block_y, int shared_dim_y);
__device__ inline int threadSharedSector4(int sidx, int sidy, int block_x, int block_y, int shared_dim_y);


// GPU Function to load parts of map to GPU Shared Memory in every block
// Depending on thread pose, it can store form 1 to 4 pixels in GPU Shared Memory
__device__ inline void loadMapToShared(
    const int16_t* heightmap,
    int16_t* costmap,
    int16_t* smem_hmap,
    const int cost_mask_radius,
    const int sidx,
    const int sidy,
    const int shared_dim_x,
    const int shared_dim_y,
    const Point2I32 pix,
    const int block_x,
    const int block_y,
    const int map_y,
    const int cmap_refresh_radius_pix);


// GPU Function to calculate COST OF TRAVEL through pixel
// It uses heightmap assigned to Shared Memory,
// and caluclate variance form mask of given size
// Unknown fields are added to cost with given value
__device__ void calcCostVariance(
    const int16_t *smem_hmap,
    int16_t* costmap,
    const int cost_mask_radius,
    const Point2I32 pix,
    const int pix_id,
    const int map_x,
    const int map_y,
    const int stid,
    const int sidx,
    const int sidy,
    const int shared_dim_y,
    const int unknown_field_cost);

// GPU Kernel to used for calculating cost map
// It is updated only in region close to rover, with given radius
__global__ void costMappingKernel(
    const int16_t *heightmap,
    int16_t* costmap,
    const int map_x,
    const int map_y,
    const int pose_x,
    const int pose_y,
    const int shared_dim_x,
    const int shared_dim_y,
    const int cmap_refresh_radius_pix,
    const int cost_mask_radius,
    const int unknown_field_cost);

#endif
