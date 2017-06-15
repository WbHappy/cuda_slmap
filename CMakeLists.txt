cmake_minimum_required(VERSION 2.8.3)
project(cuda_slmap)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_30,code=sm_30)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -w")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    cv_bridge
    image_transport
)

find_package(CUDA REQUIRED)

find_package(OpenCV 3.0.0 REQUIRED)

find_package(Boost)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package()

cuda_add_executable(map_from_file_planning
    src/map_from_file_planning/src/main.cpp
    src/map_from_file_planning/src/_robot_planner_maps.cu
    src/map_from_file_planning/src/gpu_000_load_test_map.cu
    src/map_from_file_planning/src/gpu_001_lidar_mapping.cu
    src/map_from_file_planning/src/gpu_002_cost_mapping.cu
    src/map_from_file_planning/src/gpu_003_path_planning.cu
    src/map_from_file_planning/components_cpu/src/cpu_map_i16.cpp
    src/map_from_file_planning/components_cpu/src/cpu_map_ui16.cpp
    src/map_from_file_planning/components_cpu/src/cpu_map_ui32.cpp
    src/map_from_file_planning/components_cpu/src/ht_matrix.cpp
    src/map_from_file_planning/components_gpu/src/gpu_errchk.cu
    src/map_from_file_planning/components_gpu/src/gpu_map_i16.cu
    src/map_from_file_planning/components_gpu/src/gpu_map_ui16.cu
    src/map_from_file_planning/components_gpu/src/gpu_map_ui32.cu
    )
target_link_libraries(map_from_file_planning ${catkin_LIBRARIES})
