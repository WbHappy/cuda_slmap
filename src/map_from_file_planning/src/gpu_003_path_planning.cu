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
    const int min_episode_length,
    const int global_seed,
    float *dev_debug
)
{
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    int idy = blockDim.y * blockIdx.y + threadIdx.y;
    int tid = idy + idx * gridDim.y * blockDim.y;

    int bid = blockIdx.x * gridDim.y + blockIdx.y;
    int sid = threadIdx.x * blockDim.y + threadIdx.y;

    // int threads_no = blockDim.x * blockDim.y;
    int threads_no = PLANNER_THREADS_PER_PATH;

    uint32_t local_seed = global_seed + tid * 0xffff;

    curandState_t curand_state;
    curand_init(local_seed, 0, 0, &curand_state);

    // Shared memory allocation
    __shared__ GpuPath initial_path;
    __shared__ GpuPath divided_path;
    // __shared__ GpuPath mutated_path;
    __shared__ GpuPathPoint new_points_buff[GPU_PATH_MAX_SIZE];
    __shared__ uint32_t points_costs_div[PLANNER_THREADS_PER_PATH];
    __shared__ uint32_t points_costs_mut[PLANNER_THREADS_PER_PATH];

    if(sid == 0)
    {
        initial_path.p[0] = odom;
        initial_path.p[1] = goal;
        initial_path.p[1].avrg_cost = calcEpisodeAvrgCost_Singlethread(costmap, map_x, map_y, &initial_path.p[0], &initial_path.p[1], sampling);
        initial_path.p[1].length = calcEpisodeLength_Singlethread(&initial_path.p[0], &initial_path.p[1]);

        initial_path.total_size = 2;
        initial_path.total_cost = 0;

        divided_path.total_size = 0;
        divided_path.total_cost = 0;
    }

    for(int i = 0; i < PLANNER_MAX_ITERATIONS; i++)
    {
        int std_dev_div = 128 / (i+1) / (i+1) + 8;  // Currently UNUSED!
        int std_dev_mut = 32 / (i+1) / (i+1) + 2;   // Currently UNUSED!

        dividePath_Multithread(
            costmap,
            map_x,
            map_y,
            &divided_path,
            &initial_path,
            new_points_buff,
            points_costs_div,
            &curand_state,
            min_episode_length,
            std_dev_div,
            sampling,
            sid,
            threads_no,
            dev_debug);

        __syncthreads();

        copyPath_Multithread(&initial_path, &divided_path, sid, threads_no);

        __syncthreads();

        mutatePath_Multithread(
            costmap,
            map_x,
            map_y,
            &divided_path,
            &initial_path,
            new_points_buff,
            points_costs_mut,
            &curand_state,
            min_episode_length,
            std_dev_mut,
            sampling,
            sid,
            threads_no,
            dev_debug);

        __syncthreads();

        copyPath_Multithread(&initial_path, &divided_path, sid, threads_no);

        __syncthreads();

    }

    updatePathCost_Multithread(
        &divided_path,
        sid,
        threads_no);

    copyPath_Multithread(&dev_paths[bid], &divided_path, sid, threads_no);
}

// GPU function to perform one iteration of path division
// It takes path_input, divide it, and save result path to path_output
// Each episode is divided separatley by each threads concurently
// Best episode division is chosen at the end
__device__ inline void dividePath_Multithread(
    int16_t *costmap,
    const int map_x,
    const int map_y,
    GpuPath *path_output,
    GpuPath *path_input,
    GpuPathPoint *new_points_buff,
    uint32_t *new_points_costs,
    curandState_t *curand_state,
    const int min_episode_length,
    const int std_dev,
    const int sampling,
    const int sid,
    const int threads_no,
    float* dev_debug
)
{

    if(sid == 0)
    {
        path_output->p[0] = path_input->p[0];
        path_output->total_size = (path_input->total_size - 1) * (PLANNER_EPISODE_DIVISIONS + 1) + 1;
    }

    // Iterating through path episodes
    for(int i = 0; i < path_input->total_size - 1; i++)
    {
        uint32_t thread_cost = 0;


        // Generating random points
        // First and last episode point are added on the start and end
        GpuPathPoint thread_points[PLANNER_EPISODE_DIVISIONS + 2];

        thread_points[0] = path_input->p[i]; // Assign first point
        for(int j = 0; j < PLANNER_EPISODE_DIVISIONS; j++)
        {
            if(sid == 0)
            {
                thread_points[j+1] =  generateDividePointLine_Singlethread(             // Assign all random points
                    costmap,
                    &path_input->p[i],
                    &path_input->p[i+1],
                    min_episode_length,
                    j,
                    map_x,
                    map_y,
                    sid,
                    threads_no,
                    sampling,
                    curand_state);
            }else{
                // RANDOMIZING FROM LAST POINT (MIGHT BE RANDOMIZED) TO EPISODE END
                thread_points[j+1] =  generateDividePointRandom_Singlethread(             // Assign all random points
                    costmap,
                    &thread_points[j],
                    &path_input->p[i+1],
                    min_episode_length,
                    std_dev,
                    map_x,
                    map_y,
                    sid,
                    threads_no,
                    sampling,
                    curand_state);
            }
        }
        thread_points[PLANNER_EPISODE_DIVISIONS + 1] = path_input->p[i+1]; // Assign last point
        __syncthreads();


        // Calculating cost of episode
        for(int j = 0; j < PLANNER_EPISODE_DIVISIONS + 1; j++)
        {
             thread_points[j + 1].avrg_cost = calcEpisodeAvrgCost_Singlethread(costmap, map_x, map_y, &thread_points[j], &thread_points[j+1], sampling);
             thread_points[j + 1].length = calcEpisodeLength_Singlethread(&thread_points[j], &thread_points[j+1]);
             thread_cost += thread_points[j + 1].cost();
        }
        new_points_costs[sid] = thread_cost;
        __syncthreads();


        dev_debug[sid] = thread_cost;
        dev_debug[31] = findCheapestThreadPath(new_points_costs, sid, threads_no);


        // Chosing thread with lowest cost
        int best_thread = findCheapestThreadPath(new_points_costs, sid, threads_no);
        if(sid == best_thread)
        {
            for(int j = 0; j < PLANNER_EPISODE_DIVISIONS + 1; j++)
            {
                int output_idx = (PLANNER_EPISODE_DIVISIONS + 1) * i + j + 1;
                path_output->p[output_idx] = thread_points[j+1];
            }
        }
    }
}

// GPU function to perform one iteration of path mutation
__device__ inline void mutatePath_Multithread(
    int16_t *costmap,
    const int map_x,
    const int map_y,
    GpuPath *path_output,
    GpuPath *path_input,
    GpuPathPoint *new_points_buff,
    uint32_t *new_points_costs,
    curandState_t *curand_state,
    const int min_episode_length,
    const int std_dev,
    const int sampling,
    const int sid,
    const int threads_no,
    float* dev_debug
)
{

    if(sid == 0)
    {
        path_output->p[0] = path_input->p[0];
        path_output->total_size = path_input->total_size;
        path_output->p[path_output->total_size-1] = path_input->p[path_input->total_size-1];
    }

    // Iterating through path internal points
    for(int i = 0; i < path_input->total_size - PLANNER_EPISODE_MUTATIONS - 1; i++)
    {
        uint32_t thread_cost = 0;


        // Generating random points
        // First and last episode point are added on the start and end
        GpuPathPoint thread_points[PLANNER_EPISODE_MUTATIONS + 2];

        thread_points[0] = path_input->p[i]; // Assign first point
        for(int j = 0; j < PLANNER_EPISODE_MUTATIONS; j++)
        {
            if(sid == 0)
            {
                thread_points[j + 1] = path_input->p[i + j + 1];
            }else{
                thread_points[j + 1] = generateMutatePoint_Singlethread(
                    costmap,
                    &path_input->p[i + j],
                    &path_input->p[i + j + 1],
                    min_episode_length,
                    std_dev,
                    map_x,
                    map_y,
                    sid,
                    threads_no,
                    sampling,
                    curand_state);  // Assign all random points
            }
        }
        thread_points[PLANNER_EPISODE_MUTATIONS + 1] = path_input->p[i + PLANNER_EPISODE_MUTATIONS + 1]; // Assign last point

        __syncthreads();


        // Calculating cost of episode
        for(int j = 0; j < PLANNER_EPISODE_MUTATIONS + 1; j++)
        {
            thread_points[j + 1].avrg_cost = calcEpisodeAvrgCost_Singlethread(costmap, map_x, map_y, &thread_points[j], &thread_points[j+1], sampling);
            thread_points[j + 1].length = calcEpisodeLength_Singlethread(&thread_points[j], &thread_points[j+1]);
            thread_cost += thread_points[j + 1].cost();
        }

        new_points_costs[sid] = thread_cost;
        __syncthreads();


        // dev_debug[sid] = thread_cost;
        // dev_debug[31] = findCheapestThreadPath(new_points_costs, sid, threads_no);


        // Chosing thread with lowest cost
        int best_thread = findCheapestThreadPath(new_points_costs, sid, threads_no);
        if(sid == best_thread)
        {
            for(int j = 0; j < PLANNER_EPISODE_MUTATIONS + 1; j++)
            {
                path_output->p[i + j + 1] = thread_points[j+1];

            }
        }
    }
}

__device__ inline void updatePathCost_Multithread(
    GpuPath *path_input,
    int sid,
    int threads_no
)
{
    int point_index = sid;
    int path_size = path_input->total_size;
    uint32_t thread_cost = 0;

    while(point_index < path_size)
    {
        thread_cost += path_input->p[point_index].cost();
        point_index += threads_no;
    }

    __syncthreads();

    __shared__ int cost_array[PLANNER_THREADS_PER_PATH];
    cost_array[sid] = thread_cost;
    int max_sid = PLANNER_THREADS_PER_PATH / 2;

    while(max_sid > 0 && sid < max_sid)
    {

        cost_array[sid] = cost_array[sid] + cost_array[sid + max_sid];

        __syncthreads();

        max_sid /= 2;
    }
    __syncthreads();

    if(sid == 0)
    {
        path_input->total_cost = cost_array[0];
    }
}

__device__ inline int findCheapestThreadPath(uint32_t *new_points_costs, int sid, int threads_no)
{
    __shared__ int sid_array[PLANNER_THREADS_PER_PATH];
    sid_array[sid] = sid;

    int max_sid = PLANNER_THREADS_PER_PATH / 2;

    while(max_sid > 0 && sid < max_sid)
    {
        if(new_points_costs[sid_array[sid]] < new_points_costs[sid_array[sid + max_sid]])
        {
            // sid_array[sid] = sid_array[sid];  DO NOTHING
        }else{
            sid_array[sid] = sid_array[sid + max_sid];
        }
        __syncthreads();

        max_sid /= 2;
    }
    __syncthreads();

    return sid_array[0];

}

__device__ inline float sqrf (float x)
{
    return x * x;
}

__device__ inline void copyPath_Singlethread(
    GpuPath *path_output,
    GpuPath *path_input)
{
    int total_size = path_input->total_size;

    path_output->total_size = total_size;
    path_output->total_cost = path_input->total_cost;

    // Index of first point to copy in this thread
    int point_index = 0;

    // Copying points from path_output to path_input.
    while(point_index < total_size)
    {
        path_output->p[point_index] = path_input->p[point_index];
        point_index += 1;
    }
}

__device__ inline void copyPath_Multithread(
    GpuPath *path_output,
    GpuPath *path_input,
    int sid,
    int threads_no)
{
    int total_size = path_input->total_size;

    if(sid == 0)
    {
        path_output->total_size = total_size;
        path_output->total_cost = path_input->total_cost;
    }

    // Index of first point to copy in this thread
    int point_index = sid;

    // Copying points from path_output to path_output.
    while(point_index < total_size)
    {
        path_output->p[point_index] = path_input->p[point_index];
        point_index += threads_no;
    }
}

__device__ inline GpuPathPoint generateDividePointRandom_Singlethread(
    int16_t *costmap,
    const GpuPathPoint *p1,
    const GpuPathPoint *p2,
    const int min_episode_length,
    const int std_dev,
    const int map_x,
    const int map_y,
    const int sid,
    const int threads_no,
    const int sampling,
    curandState_t *curand_state)
{
    GpuPathPoint random_point;

    // UNIFORM DISTRIBUTION
    // random_point.x = curand(curand_state) % map_x;
    // random_point.y = curand(curand_state) % map_y;

    int std_dev_len = p2->length / DIVISION_STD_DEV_DIVIDER;

    // NORMAL DISTRIBUTION
    int i = 0;
    do
    {
        random_point.x =( p1->x + p2->x ) / 2 + (int)(curand_normal(curand_state) * std_dev_len);
        random_point.y =( p1->y + p2->y ) / 2 + (int)(curand_normal(curand_state) * std_dev_len);

        if(random_point.x < 0) random_point.x = 0;
        if(random_point.x >= map_x) random_point.x = map_x - 1;
        if(random_point.y < 0) random_point.y = 0;
        if(random_point.y >= map_y) random_point.y = map_y - 1;

        random_point.length = calcEpisodeLength_Singlethread(p1, p2);

        i++;
        if(i == PLANNER_MAX_RERANDOMING)
        {
            random_point.avrg_cost = 32000;
            return random_point;
        }

    }while(random_point.length < min_episode_length);

    random_point.avrg_cost = calcEpisodeAvrgCost_Singlethread(costmap, map_x, map_y, p1, p2, sampling);

    return random_point;
}

__device__ inline GpuPathPoint generateDividePointLine_Singlethread(
    int16_t *costmap,
    const GpuPathPoint *p1,
    const GpuPathPoint *p2,
    const int min_episode_length,
    const int line_index,
    const int map_x,
    const int map_y,
    const int sid,
    const int threads_no,
    const int sampling,
    curandState_t *curand_state)
{
    GpuPathPoint random_point;

    int weight_1 = line_index + 1;
    int weight_2 = PLANNER_EPISODE_DIVISIONS - line_index;
    int weight_total = weight_1 + weight_2;

    // LINEAR DIVISION
    int i = 0;
    do
    {
        random_point.x =( p1->x*weight_1 + p2->x*weight_2 ) / weight_total;
        random_point.y =( p1->y*weight_1 + p2->y*weight_2 ) / weight_total;

        if(random_point.x < 0) random_point.x = 0;
        if(random_point.x >= map_x) random_point.x = map_x - 1;
        if(random_point.y < 0) random_point.y = 0;
        if(random_point.y >= map_y) random_point.y = map_y - 1;

        random_point.length = calcEpisodeLength_Singlethread(p1, p2);

        i++;
        if(i == PLANNER_MAX_RERANDOMING)
        {
            random_point.avrg_cost = 32000;
            return random_point;
        }

    }while(random_point.length < min_episode_length);

    random_point.avrg_cost = calcEpisodeAvrgCost_Singlethread(costmap, map_x, map_y, p1, p2, sampling);

    return random_point;
}

__device__ inline GpuPathPoint generateMutatePoint_Singlethread(
    int16_t *costmap,
    const GpuPathPoint *p1,
    const GpuPathPoint *p2,
    const int min_episode_length,
    const int std_dev,
    const int map_x,
    const int map_y,
    const int sid,
    const int threads_no,
    const int sampling,
    curandState_t *curand_state)
{
    GpuPathPoint random_point;

    // UNIFORM DISTRIBUTION
    // random_point.x = curand(curand_state) % map_x;
    // random_point.y = curand(curand_state) % map_y;

    int std_dev_len = p2->length / MUTATION_STD_DEV_DIVIDER;

    // NORMAL DISTRIBUTION
    int i = 0;
    do
    {
        random_point.x = p2->x + (int)(curand_normal(curand_state) * std_dev_len);
        random_point.y = p2->y + (int)(curand_normal(curand_state) * std_dev_len);

        if(random_point.x < 0) random_point.x = 0;
        if(random_point.x >= map_x) random_point.x = map_x - 1;
        if(random_point.y < 0) random_point.y = 0;
        if(random_point.y >= map_y) random_point.y = map_y - 1;

        random_point.length = calcEpisodeLength_Singlethread(p1, p2);

        i++;
        if(i == PLANNER_MAX_RERANDOMING)
        {
            random_point.avrg_cost = 32000;
            return random_point;
        }

    }while(random_point.length < min_episode_length);

    random_point.avrg_cost = calcEpisodeAvrgCost_Singlethread(costmap, map_x, map_y, p1, p2, sampling);

    return random_point;

}

__device__ inline void addPathPoints_Multithread(
    GpuPath *path_output,
    GpuPath *path_input,
    GpuPathPoint *new_points,
    int sid,
    int threads_no,
    int new_points_index)
{
    // Index of last point do move
    int old_total_size = path_input->total_size;
    __syncthreads();

    int new_points_number = PLANNER_EPISODE_DIVISIONS;

    // Index of first point to copy in this thread
    int point_index = sid;

    // Copying points from path_input to path_output.
    // Points with indexes higher or equal to new_points_index are moved by offset new_points_number
    while(point_index < old_total_size)
    {

        if(point_index < new_points_index)
        {
            path_output->p[point_index] = path_input->p[point_index];
        }
        else
        {
            path_output->p[point_index + new_points_number] = path_input->p[point_index];
        }

        point_index += threads_no;
    }

    // Adding new points to new path
    if(sid < new_points_number)
    {
        path_output->p[new_points_index + sid] = new_points[sid];
    }

    // Save new total cost.
    // if(sid == threads_no - 2){  path_output->total_cost +=  1;  }

    // Save new total points number.
    if(sid == 0){   path_output->total_size += new_points_number; }

}


// Calcualtes cost of traveling via episode
__device__ inline int16_t calcEpisodeAvrgCost_Singlethread(
    int16_t *costmap,
    const int map_x,
    const int map_y,
    const GpuPathPoint *p1,
    const GpuPathPoint *p2,
    const int sampling
)
{
    // Totoal cost of traveling through this episode
    int avrg_cost = 0;

    // Distance between two points - length of episode
    int dist_x = p2->x - p1->x;
    int dist_y = p2->y - p1->y;
    float dist = sqrtf(sqrf(dist_x) + sqrf(dist_y));

    // Number of samples taken from episode
    int samples_num = (int)ceilf(dist / sampling);

    // Adding cost for each consecutive point
    GpuPathPoint sampling_point;
    for(int i = 1; i <= samples_num; i++)       // i = 1 - Avoiding taking the same points 2 times to caluclation in diffrent episodes
    {
        sampling_point.x = p1->x + (int)roundf(dist_x * i / samples_num);
        sampling_point.y = p1->y + (int)roundf(dist_y * i / samples_num);

        avrg_cost += costmap[sampling_point.x * map_y + sampling_point.y];
        // costmap[sampling_point.x * map_y + sampling_point.y] = 16000;               // DRAWING POINTS ON MAP!!!!
    }

    return avrg_cost / samples_num;
}

// Calcualtes cost of traveling via episode
__device__ inline uint16_t calcEpisodeLength_Singlethread(
    const GpuPathPoint *p1,
    const GpuPathPoint *p2
)
{
    int32_t x = p1->x - p2->x;
    int32_t y = p1->y - p2->y;
    return (uint16_t) sqrtf(x*x + y*y);

}


GpuPathPlanning::GpuPathPlanning(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros)
{
    this->_rpm = _rpm;
    this->_ros = _ros;
}

void GpuPathPlanning::allocateMemory()
{
    // PATH
    gpuErrchk( cudaMalloc((void**)&dev_path, planner_concurrent_paths * sizeof(GpuPath)) );
    host_path = (GpuPath*) malloc(planner_concurrent_paths * sizeof(GpuPath));

    // DEBUG
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
    robot_onmap.length = 0;
    robot_onmap.avrg_cost = 0;

    GpuPathPoint goal_onmap;
    goal_onmap.x = _rpm->goal_onmap_x;
    goal_onmap.y = _rpm->goal_onmap_y;
    goal_onmap.length = 0;
    goal_onmap.avrg_cost = 0;

    // Seed for cuRand
    gettimeofday(&host_time, 0);
    uint32_t global_seed = host_time.tv_sec + host_time.tv_usec;
    // uint32_t global_seed = 1;

    pathPlanningKernel<<<planner_concurrent_paths, PLANNER_THREADS_PER_PATH>>>(
                            dev_path,
                            _rpm->dev_costmap.data,
                            _rpm->dev_costmap.size_x,
                            _rpm->dev_costmap.size_y,
                            robot_onmap,
                            goal_onmap,
                            planner_cost_sampling,
                            planner_max_iteration,
                            planner_min_episode_length,
                            global_seed,
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

    for(int j = 0; j < host_path[0].total_size -1; j++)
    {
        _rpm->host_costmap.drawEpiosde(
            "costmap",
            128,
            host_path[0].p[j].x,
            host_path[0].p[j].y,
            host_path[0].p[j+1].x,
            host_path[0].p[j+1].y);
    }


    for(int i = 0; i < planner_concurrent_paths; i++)
    {
        printf("===========\n");
        printf("host path %d\n\n", i);

        for(int j = 0; j < host_path[i].total_size; j++)
        {
            printf("    point: %d\n", j);
            printf("    x: %d\n", host_path[i].p[j].x);
            printf("    y: %d\n", host_path[i].p[j].y);
            printf("    avrg_cost: %d\n", host_path[i].p[j].avrg_cost);
            printf("    length: %d\n\n", host_path[i].p[j].length);
        }
        printf("    total size: %d\n", host_path[i].total_size);
        printf("    total cost: %d\n", host_path[i].total_cost);
        printf("===========\n");
    }

    printf("debug:\n");
    for(int i = 0; i < 32; i++)
    {
        printf("    debug %d: %f\n", i, host_debug[i]);
    }
    printf("===========\n");

}
