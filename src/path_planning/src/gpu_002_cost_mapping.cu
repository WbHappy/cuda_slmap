#include "../include/gpu_002_cost_mapping.cuh"


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
                            const int unknown_field_cost)
{
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    int idy = blockDim.y * blockIdx.y + threadIdx.y;
    int tid = idy + idx * gridDim.y * blockDim.y;

    int sidx = threadIdx.x;
    int sidy = threadIdx.y;
    int stid = sidx * blockDim.y + sidy;


    // TRANSFORMING LOCAL THREAD IDX TO PIXEL IDX IN GLOABAL MAP FRAME
    // (FOR THIS PIXEL COST WILL BE CALCULATED)
    Point2I32 pix = threadOnGlobalMap(idx, idy, pose_x, pose_y, cmap_refresh_radius_pix);

    // INDEX OF PIXEL IN GLOBAL MAP
    int pix_id = pix.y + pix.x * map_y;


    // SHARED MEMORY FOR STORAGE PART OF HEIGHTMAP NEEDED FOR THIS BLOCK
    // ARRAY IS ALLOCATED WITH DYNAMIC SIZE (NOT KNOWN AT COMPILATION - extern)
    extern __shared__ int16_t smem_hmap[];

    loadMapToShared(
                heightmap,
                costmap,
                smem_hmap,
                cost_mask_radius,
                sidx,
                sidy,
                shared_dim_x,
                shared_dim_y,
                pix,
                blockDim.x,
                blockDim.y,
                map_y,
                cmap_refresh_radius_pix
    );

    __syncthreads();

    // CHECKING IF SCAN POINT IS INSIDE GPU MAP, NOT CLOSER FROM EDGE THAN "cost_mask_radius"
    if(
        pix.x >= cost_mask_radius &&
        pix.x <= map_x - cost_mask_radius &&
        pix.y >= cost_mask_radius &&
        pix.y <= map_y - cost_mask_radius
    )
    {

        // CHECKING IF SCAN POINT IS INSIDE RADIUS OF COSTMAP REFREESHING
        int dist_x = (pix.x - pose_x);
        int dist_y = (pix.y - pose_y);

        float dist_from_center = sqrtf((float)dist_x*dist_x + (float)dist_y*dist_y);

        if(dist_from_center <= cmap_refresh_radius_pix)
        {
            // CALCULATING NEW POINT OF COSTMAP
            calcCostVariance(
                    smem_hmap,
                    costmap,
                    cost_mask_radius,
                    pix,
                    pix_id,
                    map_x,
                    map_y,
                    stid,
                    sidx,
                    sidy,
                    shared_dim_y,
                    unknown_field_cost
            );
        }
    }

}


// GPU Function to "Localize" position of GPU Thread on Global GPU Map.
// It assumes that Rover's position is the middle of GPU Grid
__device__ inline Point2I32 threadOnGlobalMap(int idx, int idy, int pose_x, int pose_y, int cmap_refresh_radius_pix)
{
    Point2I32 pix;

    pix.x = pose_x - cmap_refresh_radius_pix + idx;
    pix.y = pose_y - cmap_refresh_radius_pix + idy;

    return pix;
}

// GPU Functions to support shared memory filling pattern
// Transforms position of GPU Thread on Global GPU Map
// to corresponding position in sector 1,2,3,4 on Global GPU Map (not a mistake)
__device__ inline int threadGlobalSector1(Point2I32 pix, int block_x, int block_y, int map_y, int cost_mask_radius)
{
    Point2I32 point;

    point.x = pix.x - cost_mask_radius;
    point.y = pix.y - cost_mask_radius;

    return point.x * map_y + point.y;
}
__device__ inline int threadGlobalSector2(Point2I32 pix, int block_x, int block_y, int map_y, int cost_mask_radius)
{
    Point2I32 point;

    point.x = pix.x - cost_mask_radius + block_x;
    point.y = pix.y - cost_mask_radius;

    return point.x * map_y + point.y;
}
__device__ inline int threadGlobalSector3(Point2I32 pix, int block_x, int block_y, int map_y, int cost_mask_radius)
{
    Point2I32 point;

    point.x = pix.x - cost_mask_radius;
    point.y = pix.y - cost_mask_radius + block_y;

    return point.x * map_y + point.y;
}
__device__ inline int threadGlobalSector4(Point2I32 pix, int block_x, int block_y, int map_y, int cost_mask_radius)
{
    Point2I32 point;

    point.x = pix.x - cost_mask_radius + block_x;
    point.y = pix.y - cost_mask_radius + block_y;

    return point.x * map_y + point.y;
}


// GPU Functions to support shared memory filling pattern
// Transforms position of GPU Thread in Block (local thread)
// to corresponding index in sector 1,2,3,4 of GPU Shared Memory
__device__ inline int threadSharedSector1(int sidx, int sidy, int block_x, int block_y, int shared_dim_y)
{
    return sidx * shared_dim_y + sidy;
}
__device__ inline int threadSharedSector2(int sidx, int sidy, int block_x, int block_y, int shared_dim_y)
{
    return (sidx + block_x) * shared_dim_y + sidy;
}
__device__ inline int threadSharedSector3(int sidx, int sidy, int block_x, int block_y, int shared_dim_y)
{
    return sidx * shared_dim_y + (sidy + block_y);
}
__device__ inline int threadSharedSector4(int sidx, int sidy, int block_x, int block_y, int shared_dim_y)
{
    return (sidx + block_x) * shared_dim_y + (sidy + block_y);
}


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
                                const int cmap_refresh_radius_pix
)
{
    // Offset used to decide, if thread will assign values to sectors 2,3,4
    int offset = 2 * cost_mask_radius - 2;

    // Global indexes of pixels
    int gid_1 = threadGlobalSector1(pix, block_x, block_y, map_y, cost_mask_radius);
    int gid_2 = threadGlobalSector2(pix, block_x, block_y, map_y, cost_mask_radius);
    int gid_3 = threadGlobalSector3(pix, block_x, block_y, map_y, cost_mask_radius);
    int gid_4 = threadGlobalSector4(pix, block_x, block_y, map_y, cost_mask_radius);

    // Corresponding indexes of pixels in Shared Memory
    int sid_1 = threadSharedSector1(sidx, sidy, block_x, block_y, shared_dim_y);
    int sid_2 = threadSharedSector2(sidx, sidy, block_x, block_y, shared_dim_y);
    int sid_3 = threadSharedSector3(sidx, sidy, block_x, block_y, shared_dim_y);
    int sid_4 = threadSharedSector4(sidx, sidy, block_x, block_y, shared_dim_y);

    // Filling Sector 1
    smem_hmap[sid_1] = heightmap[gid_1];

    // Filling Sector 2
    if(sidx < offset)
    {
        smem_hmap[sid_2] = heightmap[gid_2];
    }

    // Filling Sector 3
    if(sidy < offset)
    {
        smem_hmap[sid_3] = heightmap[gid_3];
    }

    // Filling Sector 4
    if(sidx < offset && sidy < offset)
    {
        smem_hmap[sid_4] = heightmap[gid_4];
    }

}


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
                            const int unknown_field_cost
)
{
    float avrg = 0;

    int mask_dim = (2 * cost_mask_radius) - 1;

    int smem_idx = sidx * shared_dim_y + sidy;

    int known_heights_conuter = 0;
    int unknown_heights_conuter = 0;


    // Calculating average value of known points
    for(int x = 0; x < mask_dim; x++)
    {
        for(int y = 0; y < mask_dim; y++)
        {
            if(smem_hmap[smem_idx + x*shared_dim_y + y] != UNKNOWN)
            {
                avrg += (float) smem_hmap[smem_idx + x*shared_dim_y + y];
                known_heights_conuter++;
            }
            else
            {
                unknown_heights_conuter++;
            }
        }
    }
    avrg /= (float) known_heights_conuter;

    // Calculating variance of known points
    float variance = 0;
    for(int x = 0; x < mask_dim; x++)
    {
        for(int y = 0; y < mask_dim; y++)
        {
            if(smem_hmap[smem_idx + x*shared_dim_y + y] != UNKNOWN)
            {
                float diff = (float) (avrg - smem_hmap[smem_idx + x*shared_dim_y + y]);
                variance += diff * diff;
            }
        }
    }

    variance = sqrtf(variance);

    // Adding cost of unknown points
    int pixel_cost = variance + unknown_heights_conuter * unknown_field_cost;

    costmap[pix_id] = (int16_t) (pixel_cost);
}



GpuCostMapping::GpuCostMapping(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros)
{
    this->_rpm = _rpm;
    this->_ros = _ros;
}


void GpuCostMapping::drawInitialCostmapBorders()
{
    _rpm->dev_costmap.drawBorders(costmap_borders_value, cost_mask_radius);
}




void GpuCostMapping::copyInputToDevice()
{

}





void GpuCostMapping::executeKernel()
{
    int block_x = 32;
    int block_y = 32;

    int size_x = 2*cmap_refresh_radius_pix;
    int size_y = 2*cmap_refresh_radius_pix;

    int grid_x = (size_x + block_x - 1) / block_x;
    int grid_y = (size_y + block_y - 1) / block_y;
    dim3 grid(grid_x, grid_y, 1);
    dim3 block(block_x, block_y, 1);

    int shared_dim_x = block_x + 2 * cost_mask_radius - 2;
    int shared_dim_y = block_y + 2 * cost_mask_radius - 2;
    int shared_size = shared_dim_x * shared_dim_y * sizeof(int16_t);

    costMappingKernel <<< grid, block, shared_size >>> (
        _rpm->dev_heightmap.data,
        _rpm->dev_costmap.data,
        _rpm->dev_costmap.size_x,
        _rpm->dev_costmap.size_y,
        _rpm->robot_onmap_x,
        _rpm->robot_onmap_y,
        shared_dim_x,
        shared_dim_y,
        this->cmap_refresh_radius_pix,
        this->cost_mask_radius,
        this->unknown_field_cost);

        gpuErrchk( cudaPeekAtLastError() );
        gpuErrchk( cudaDeviceSynchronize() );

}





void GpuCostMapping::copyOutputToHost()
{
    gpuErrchk( cudaMemcpy(_rpm->host_costmap.data, _rpm->dev_costmap.data, _rpm->dev_costmap.size() * sizeof(int16_t), cudaMemcpyDeviceToHost) );
}






void GpuCostMapping::display()
{
    _rpm->host_costmap.display("costmap");
}
