#include "../include/gpu_003_path_planning.cuh"

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
                            float *dev_debug
)
{
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    int idy = blockDim.y * blockIdx.y + threadIdx.y;
    int tid = idy + idx * gridDim.y * blockDim.y;

    int bid = blockIdx.x * gridDim.y + blockIdx.y;
    int sid = threadIdx.x * blockDim.y + threadIdx.y;

    int threads_no = blockDim.x * blockDim.y;

    // Creating new path with starting point, ending point and cost
    __shared__ GpuPath path;
    path.p[0] = odom;
    path.p[1] = goal;
    path.total_size = 2;

    // Temporary path
    __shared__ GpuPathPoint new_points[1];
    new_points[0].x = 70;
    new_points[0].y = 90;

    int new_points_index = 1;

    __shared__ GpuPath result_path;

    // divideEpisode_Multithread(costmap, map_x, map_y, path.p[0], path.p[1], sampling, sid, threads_no);

    addPathPoints_Multithread(
        &result_path,
        &path,
        new_points,
        sid,
        threads_no,
        new_points_index,
        dev_debug);

    __syncthreads();

    if(sid == 0){
        result_path.p[0].cost = 0;
        result_path.p[1].cost = calcEpisodeCost_Singlethread(costmap, map_x, map_y, result_path.p[0], result_path.p[1], sampling);
        result_path.p[2].cost = calcEpisodeCost_Singlethread(costmap, map_x, map_y, result_path.p[1], result_path.p[2], sampling);
        result_path.total_cost = result_path.p[1].cost + result_path.p[2].cost;
        result_path.total_size = 3;

        // Divide episode

        // copyPath_Singlethread(&dev_paths[bid], &result_path);

        // Copy result path to global memory
    }
    copyPath_Multithread(&dev_paths[bid], &result_path, sid, threads_no);

}

__device__ inline float sqrf (float x)
{
    return x * x;
}

__device__ inline void copyPath_Singlethread(
                            GpuPath *new_path,
                            GpuPath *old_path)
{
    int total_size = old_path->total_size;

    new_path->total_size = total_size;
    new_path->total_cost = old_path->total_cost;

    // Index of first point to copy in this thread
    int point_index = 0;

    // Copying points from old_path to new_path.
    while(point_index < total_size)
    {
        new_path->p[point_index] = old_path->p[point_index];
        point_index += 1;
    }
}

__device__ inline void copyPath_Multithread(
                            GpuPath *new_path,
                            GpuPath *old_path,
                            int sid,
                            int threads_no)
{
    int total_size = old_path->total_size;

    if(sid == 0)
    {
        new_path->total_size = total_size;
        new_path->total_cost = old_path->total_cost;
    }

    // Index of first point to copy in this thread
    int point_index = sid;

    // Copying points from old_path to new_path.
    while(point_index < total_size)
    {
        new_path->p[point_index] = old_path->p[point_index];
        point_index += threads_no;
    }
}

__device__ inline GpuPathPoint generateRandomPoint(
                            const GpuPathPoint *p1,
                            const GpuPathPoint *p2,
                            int ep_div_no,
                            int ep_div_total,
                            int sid,
                            int threads_no)
{
    GpuPathPoint random_point;
    random_point.x = ( p1->x*(ep_div_no+1) + p2->x*(ep_div_total-ep_div_no ) / (ep_div_total+1) ) + 2*sid - threads_no;
    random_point.y = ( p1->y*(ep_div_no+1) + p2->y*(ep_div_total-ep_div_no ) / (ep_div_total+1) ) + 2*sid - threads_no;



    return random_point;

}

__device__ inline void addPathPoints_Multithread(
                                GpuPath *new_path,
                                GpuPath *old_path,
                                GpuPathPoint *new_points,
                                int sid,
                                int threads_no,
                                int new_points_index,
                                float *dev_debug
                            )
{
    // Index of last point do move
    int old_total_size = old_path->total_size;
    __syncthreads();

    int new_points_number = PLANNER_EPISODE_DIVISIONS;

    // Index of first point to copy in this thread
    int point_index = sid;

    // Copying points from old_path to new_path.
    // Points with indexes higher or equal to new_points_index are moved by offset new_points_number
    while(point_index < old_total_size)
    {

        if(point_index < new_points_index)
        {
            new_path->p[point_index] = old_path->p[point_index];
        }
        else
        {
            new_path->p[point_index + new_points_number] = old_path->p[point_index];
        }

        point_index += threads_no;
    }

    // Adding new points to new path
    if(sid < new_points_number)
    {
        new_path->p[new_points_index + sid] = new_points[sid];
    }

    // Save new total cost.
    // if(sid == threads_no - 2){  new_path->total_cost +=  1;  }

    // Save new total points number.
    if(sid == 0){   new_path->total_size += new_points_number; }

}


// Calcualtes cost of traveling via episode
__device__ inline int calcEpisodeCost_Singlethread(
                                int16_t *costmap,
                                const int map_x,
                                const int map_y,
                                const GpuPathPoint p1,
                                const GpuPathPoint p2,
                                const int sampling
                            )
{
    // Totoal cost of traveling through this episode
    int total_cost = 0;

    // Distance between two points - length of episode
    int dist_x = p2.x - p1.x;
    int dist_y = p2.y - p1.y;
    float dist = sqrtf(sqrf(dist_x) + sqrf(dist_y));

    // Number of samples taken from episode
    int samples_num = (int)ceilf(dist / sampling);

    // Adding cost for each consecutive point
    GpuPathPoint sampling_point;
    for(int i = 1; i <= samples_num; i++)       // i = 1 - Avoiding taking the same points 2 times to caluclation in diffrent episodes
    {
        sampling_point.x = p1.x + (int)roundf(dist_x * i / samples_num);
        sampling_point.y = p1.y + (int)roundf(dist_y * i / samples_num);

        total_cost += costmap[sampling_point.x * map_y + sampling_point.y];
        costmap[sampling_point.x * map_y + sampling_point.y] = 16000;               // DRAWING POINTS ON MAP!!!!
    }

    return total_cost;
}


__device__ inline void divideEpisode_Multithread(
                            int16_t *costmap,
                            const int map_x,
                            const int map_y,
                            const GpuPathPoint p1,
                            const GpuPathPoint p2,
                            const int sampling,
                            const int sid,
                            const int threads_no
)
{
    // Array size is + 2, becouse we want to add p1 and p2 on the begin and end
    GpuPathPoint new_episode_points[PLANNER_EPISODE_DIVISIONS + 2];

    // Filling new_episode_points with points
    new_episode_points[0] = p1;
    for(int i = 1; i < PLANNER_EPISODE_DIVISIONS + 1; i++)
    {
        new_episode_points[i] = generateRandomPoint(&p1, &p2, i, PLANNER_EPISODE_DIVISIONS, sid, threads_no);
    }
    new_episode_points[PLANNER_EPISODE_DIVISIONS + 1] = p2;

    // Calculating costs of new episode
    for(int i = 1; i < PLANNER_EPISODE_DIVISIONS + 2; i++)
    {
        new_episode_points[i].cost = calcEpisodeCost_Singlethread(costmap, map_x, map_y, new_episode_points[i-1], new_episode_points[i],sampling);
    }

}

GpuPathPlanning::GpuPathPlanning(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros)
{
    this->_rpm = _rpm;
    this->_ros = _ros;
}

void GpuPathPlanning::allocateMemory()
{

    gpuErrchk( cudaMalloc((void**)&dev_path, planner_concurrent_paths * sizeof(GpuPath)) );
    host_path = (GpuPath*) malloc(planner_concurrent_paths * sizeof(GpuPath));

    gpuErrchk( cudaMalloc((void**)&dev_debug, 32*sizeof(float)) );
    host_debug = (float*) malloc(32*sizeof(float));

    gpuErrchk( cudaDeviceSynchronize() );
}

void GpuPathPlanning::copyInputToDevice()
{

}

void GpuPathPlanning::executeKernel()
{

    GpuPathPoint robot_onmap;
    robot_onmap.x = _rpm->robot_onmap_x;
    robot_onmap.y = _rpm->robot_onmap_y;

    GpuPathPoint goal_onmap;
    goal_onmap.x = _rpm->goal_onmap_x;
    goal_onmap.y = _rpm->goal_onmap_y;

    // pathPlanningKernel<<<planner_concurrent_paths, planner_threads_per_path>>>(
    pathPlanningKernel<<<1, 1>>>(
                            dev_path,
                            _rpm->dev_costmap.data,
                            _rpm->dev_costmap.size_x,
                            _rpm->dev_costmap.size_y,
                            robot_onmap,
                            goal_onmap,
                            planner_cost_sampling,
                            planner_max_iteration,
                            planner_min_division_length,
                            dev_debug
    );
    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaDeviceSynchronize() );

}

void GpuPathPlanning::copyOutputToHost()
{
    gpuErrchk( cudaMemcpy(host_path, dev_path, planner_concurrent_paths * sizeof(GpuPath), cudaMemcpyDeviceToHost) );
    gpuErrchk( cudaMemcpy(host_debug, dev_debug, 32 * sizeof(float), cudaMemcpyDeviceToHost) );

    gpuErrchk( cudaDeviceSynchronize() );

}

void GpuPathPlanning::display()
{
    printf("===========\n");
    printf("host path 0\n\n");

    for(int i = 0; i < host_path[0].total_size; i++)
    {
        printf("    point: %d\n", i);
        printf("    x: %d\n", host_path[0].p[i].x);
        printf("    y: %d\n", host_path[0].p[i].y);
        printf("    cost: %d\n\n", host_path[0].p[i].cost);
    }
    printf("    total size: %d\n", host_path[0].total_size);
    printf("    total cost: %d\n", host_path[0].total_cost);

    printf("===========\n");
    printf("debug:\n");
    for(int i = 0; i < 32; i++)
    {
        printf("    debug %d: %f\n", i, host_debug[i]);
    }
    printf("===========\n");

}
