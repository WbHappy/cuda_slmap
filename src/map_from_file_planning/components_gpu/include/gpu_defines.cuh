#ifndef GPU_DEFINES_CUH_
#define GPU_DEFINES_CUH_

#define PLANNER_EPISODE_DIVISIONS 1     // Number of new points created during division (1 -> episode is divided on 2 parts)
#define PLANNER_EPISODE_MUTATIONS 1     // Number of points mutated together
#define PLANNER_THREADS_PER_PATH 32

#define PLANNER_MAX_ITERATIONS 6

#define PLANNER_MAX_RERANDOMING 5

#define DIVISION_STD_DEV_DIVIDER 2
#define MUTATION_STD_DEV_DIVIDER 4

#define GPU_PATH_MAX_SIZE 128

#endif
