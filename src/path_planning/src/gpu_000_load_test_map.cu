#include "../include/gpu_000_load_test_map.cuh"

GpuLoadTestMap::GpuLoadTestMap(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros)
{
    this->_rpm = _rpm;
    this->_ros = _ros;
}

void GpuLoadTestMap::readHeightMapFromFile8(std::string path)
{

    // Read map from file located inside package
    std::string package = ros::package::getPath("cuda_slmap");
    this->cv_hmap = cv::imread(package + "/" + path , CV_LOAD_IMAGE_ANYDEPTH);

    // Copy map to cpu_map object
    _rpm->host_heightmap.allocate(cv_hmap.rows, cv_hmap.cols);
    int map_size =  cv_hmap.rows * cv_hmap.cols;
    for(int i = 0; i < map_size; i++)
    {
        int16_t data;
        data = cv_hmap.data[i];
        data = data << 8;
        data -= 32768;              // UInt16 to Int16

        _rpm->host_heightmap.data[i] = data;
    }

    // Copy map to gpu_map object
    _rpm->dev_heightmap.allocate(cv_hmap.rows, cv_hmap.cols);

    gpuErrchk( cudaMemcpy(_rpm->dev_heightmap.data, _rpm->host_heightmap.data, map_size*sizeof(int16_t), cudaMemcpyHostToDevice) );

}

void GpuLoadTestMap::readHeightMapFromFile16(std::string path)
{

    // Read map from file located inside package
    std::string package = ros::package::getPath("cuda_slmap");
    this->cv_hmap = cv::imread(package + "/" + path , CV_LOAD_IMAGE_ANYDEPTH);

    // Copy map to cpu_map object
    _rpm->host_heightmap.allocate(cv_hmap.rows, cv_hmap.cols);
    int map_size =  cv_hmap.rows * cv_hmap.cols;
    for(int i = 0; i < map_size; i++)
    {
        int16_t data;
        data = cv_hmap.data[2*i+1]; // High byte
        data = data << 8;
        data += cv_hmap.data[2*i];  // Low byte
        data -= 32768;              // UInt16 to Int16

        _rpm->host_heightmap.data[i] = data;
    }

    // Copy map to gpu_map object
    _rpm->dev_heightmap.allocate(cv_hmap.rows, cv_hmap.cols);

    gpuErrchk( cudaMemcpy(_rpm->dev_heightmap.data, _rpm->host_heightmap.data, map_size*sizeof(int16_t), cudaMemcpyHostToDevice) );

}

void GpuLoadTestMap::readCostMapFromFile8(std::string path)
{

    // Read map from file located inside package
    std::string package = ros::package::getPath("cuda_slmap");
    this->cv_cmap = cv::imread(package + "/" + path , CV_LOAD_IMAGE_GRAYSCALE);

    // Copy map to cpu_map object
    _rpm->host_costmap.allocate(cv_cmap.rows, cv_cmap.cols);
    int map_size =  cv_cmap.rows * cv_cmap.cols;
    for(int i = 0; i < map_size; i++)
    {
        int16_t data;
        data = cv_cmap.data[i]; // High byte
        data = data << 7;       // Divide by 2 included - range only form 0 to 32k


        _rpm->host_costmap.data[i] = data;
    }

    // Copy map to gpu_map object
    _rpm->dev_costmap.allocate(cv_cmap.rows, cv_cmap.cols);

    gpuErrchk( cudaMemcpy(_rpm->dev_costmap.data, _rpm->host_costmap.data, map_size*sizeof(int16_t), cudaMemcpyHostToDevice) );

}


void GpuLoadTestMap::readCostMapFromFile16(std::string path)
{

    // Read map from file located inside package
    std::string package = ros::package::getPath("cuda_slmap");
    this->cv_cmap = cv::imread(package + "/" + path , CV_LOAD_IMAGE_ANYDEPTH);

    // Copy map to cpu_map object
    _rpm->host_costmap.allocate(cv_cmap.rows, cv_cmap.cols);
    int map_size =  cv_cmap.rows * cv_cmap.cols;
    for(int i = 0; i < map_size; i++)
    {
        int16_t data;
        data = cv_cmap.data[2*i+1] / 2;     // High byte
        data = data << 8;
        data += cv_cmap.data[2*i] / 2 ;     // Low byte

        _rpm->host_costmap.data[i] = data;
    }

    // Copy map to gpu_map object
    _rpm->dev_costmap.allocate(cv_cmap.rows, cv_cmap.cols);

    gpuErrchk( cudaMemcpy(_rpm->dev_costmap.data, _rpm->host_costmap.data, map_size*sizeof(int16_t), cudaMemcpyHostToDevice) );

}


void GpuLoadTestMap::display()
{
    if(_rpm->host_costmap.size() > 0)
    {
        _rpm->host_costmap.display("costmap");
    }
    if(_rpm->host_heightmap.size() > 0)
    {
        _rpm->host_heightmap.display("heightmap");
    }

}
