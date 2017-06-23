#include "../include/gpu_001_lidar_mapping.cuh"

// GPU Kernel to used for LiDAR Mapping -> Creates raw heightmap of terrain
__global__ void lidarMappingKernel(
                            float* laser_scan,
                            const HTMatrixLidarCPU dk_cpu,
                            const int laser_rays,
                            const float angle_min,
                            const float angle_max,
                            int16_t* heightmap,
                            const int map_x,
                            const int map_y,
                            const int height_scale,
                            const int map_scale,
                            const float map_orient,
                            const float map_offset_pix,
                            float* debug)
{
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    // DISTANCE MEASURED BY LASER SCAN
    float a5 = laser_scan[tid];

    // ANGLE FROM MIDDLE OF SCANNING AREA
    float th5 = calcLaserAngle(laser_rays, angle_min, angle_max, tid);

    // GLOBAL POSITION OF POINT ON THE END OF THE SCAN
    Point3F32 point_world = dkLidarToScan(&dk_cpu, th5, a5);

    // POSITION OF SCAN POINT ON GPU HEIGHTMAP
    Point2I32 point_map = pointWorldToMap(point_world.x, point_world.y, map_orient, map_scale, map_offset_pix);

    // CHECKING IF SCAN POINT IS INSIDE GPU MAP
    if(point_map.x >=0 && point_map.x < map_x && point_map.y >=0 && point_map.y < map_y)
    {
        // ASSIGN NEW POINT TO GPU MAP
        heightmap[point_map.x * map_y + point_map.y] = (int16_t) (point_world.z * height_scale);
    }

}



// GPU function to calculate angle of scan depending on ID of GPU Thread
__device__ inline float calcLaserAngle(int laser_rays, float angle_min, float angle_max, int tid)
{
    return ((float)tid+0.5)/laser_rays*(angle_max - angle_min) + angle_min;
}

// GPU function to calculate direct kinematics from World to Scan,
// with given kinematics from World to Lidar
__device__ inline Point3F32 dkLidarToScan(const HTMatrixLidarCPU* dk_cpu, float th5, float a5)
{

    Point3F32 point;

    // A_GPU_14
    point.x = a5*dk_cpu->m_0*cos(th5) + a5*dk_cpu->m_1*sin(th5) + dk_cpu->m_3;

    // A_GPU_24
    point.y = a5*dk_cpu->m_4*cos(th5) + a5*dk_cpu->m_5*sin(th5) + dk_cpu->m_7;

    // A_GPU_34
    point.z = a5*dk_cpu->m_8*cos(th5) + a5*dk_cpu->m_9*sin(th5) + dk_cpu->m_11;

    return point;

}

// GPU function to transform given point from Real World into point on GPU Map
__device__ inline Point2I32 pointWorldToMap(float world_pose_x, float world_pose_y, float map_orient, float map_scale, float map_offset_pix)
{

    Point2I32 map_pose;
    float point_orient = atan2f(world_pose_y, world_pose_x);
    float point_dist = sqrtf(world_pose_x*world_pose_x + world_pose_y*world_pose_y);

    map_pose.x = (int) (sinf(map_orient - point_orient) * point_dist * map_scale + map_offset_pix);
    map_pose.y = (int) (cosf(map_orient - point_orient) * point_dist * map_scale + map_offset_pix);

    return map_pose;
}


GpuLidarMapping::GpuLidarMapping(_RobotPlannerMaps *_rpm, _ROSBuffor *_ros)
{
    this->_rpm = _rpm;
    this->_ros = _ros;
}



void GpuLidarMapping::allocateMemory(int laser_rays, float angle_min, float angle_max)
{
    this->laser_rays = laser_rays;
    this->angle_min = angle_min;
    this->angle_max = angle_max;

    gpuErrchk(cudaMalloc((void**)&dev_laser_scan, laser_rays * sizeof(float)) );
    gpuErrchk(cudaMalloc((void**)&dev_dk_matrix, 16 * sizeof(double)) );

}


void GpuLidarMapping::freeMemory()
{
    gpuErrchk( cudaFree(dev_dk_matrix) );
    gpuErrchk( cudaFree(dev_laser_scan) );

}


void GpuLidarMapping::drawInitialHeightmapCircle()
{
    _rpm->dev_heightmap.drawCircle(init_circle_height, _rpm->map_offset_pix, _rpm->map_offset_pix, init_circle_radius);
}



void GpuLidarMapping::copyInputToDevice()
{
    // Copying laser scan to GPU
    gpuErrchk( cudaMemcpy(this->dev_laser_scan, &_ros->laser_scan.ranges[0], this->laser_rays * sizeof(float), cudaMemcpyHostToDevice) );

}


void GpuLidarMapping::executeKernel()
{

    // CPU part
    this->dk_cpu = dkWorldToLidarReduced(
                    _ros->odom.pose.pose.position.x,
                    _ros->odom.pose.pose.position.y,
                    _ros->odom.pose.pose.position.z,
                    _ros->odom.pose.pose.orientation.x,
                    _ros->odom.pose.pose.orientation.y,
                    _ros->odom.pose.pose.orientation.z,
                    _ros->odom.pose.pose.orientation.w,
                    _ros->lidar_pose.data,
                    this->dk_a1,
                    this->dk_d2,
                    this->dk_al3);

    // GPU part
    lidarMappingKernel <<< this->laser_rays, 1 >>> (
                    this->dev_laser_scan,
                    this->dk_cpu,
                    this->laser_rays,
                    this->angle_min,
                    this->angle_max,
                    _rpm->dev_heightmap.data,
                    _rpm->dev_heightmap.size_x,
                    _rpm->dev_heightmap.size_y,
                    _rpm->height_scale,
                    _rpm->map_scale,
                    _rpm->map_orient,
                    _rpm->map_offset_pix,
                    _rpm->dev_debug);

    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaDeviceSynchronize() );

}

void GpuLidarMapping::copyOutputToHost()
{
    gpuErrchk( cudaMemcpy(_rpm->host_heightmap.data, _rpm->dev_heightmap.data, _rpm->dev_heightmap.size() * sizeof(int16_t), cudaMemcpyDeviceToHost) );
}


void GpuLidarMapping::display()
{
    _rpm->host_heightmap.display("heightmap");
}


// CPU function to calculate direct kinematics form World to Lidar.
// Return full Homogenous Transformatin Matrix
HTMatrix dkWorldToLidar(
    double tx,
    double ty,
    double tz,
    double qx,
    double qy,
    double qz,
    double qw,
    double th2,
    const double a1,
    const double d2,
    const double al3
)
{

    OctaveVariable <double> TX(tx);
    OctaveVariable <double> TY(ty);
    OctaveVariable <double> TZ(tz);
    OctaveVariable <double> QX(qx);
    OctaveVariable <double> QY(qy);
    OctaveVariable <double> QZ(qz);
    OctaveVariable <double> QW(qw);
    OctaveVariable <double> TH2(th2);
    OctaveVariable <double> A1(a1);
    OctaveVariable <double> D2(d2);
    OctaveVariable <double> AL3(al3);

    OctaveVariable <double> TMP;

    HTMatrix dk_cpu;

    // A_CPU_11
    TMP =  -(2*QW*QY + 2*QX*QZ)*sin(AL3) + (-2*QW*QZ + 2*QX*QY)*(sin(TH2)*cos(AL3) + sin(TH2)/266709378811357127073829389436900 - cos(TH2)*cos(AL3)/16331239353195370 + cos(TH2)/16331239353195370) + (-2*QY->*2 - 2*QZ->*2 + 1)*(sin(TH2)*cos(AL3)/16331239353195370 - sin(TH2)/16331239353195370 + cos(TH2)*cos(AL3) + cos(TH2)/266709378811357127073829389436900);
    dk_cpu.m[0][0] = TMP.data;


    // A_CPU_12
    TMP =  (2*QW*QY + 2*QX*QZ)*sin(AL3)/16331239353195370 + (-2*QW*QZ + 2*QX*QY)*(-sin(TH2)*cos(AL3)/16331239353195370 + sin(TH2)/16331239353195370 + cos(TH2)*cos(AL3)/266709378811357127073829389436900 + cos(TH2)) + (-2*QY->*2 - 2*QZ->*2 + 1)*(-sin(TH2)*cos(AL3)/266709378811357127073829389436900 - sin(TH2) - cos(TH2)*cos(AL3)/16331239353195370 + cos(TH2)/16331239353195370);
    dk_cpu.m[0][1] = TMP.data;


    // A_CPU_13
    TMP =  (2*QW*QY + 2*QX*QZ)*cos(AL3) + (-2*QW*QZ + 2*QX*QY)*(sin(TH2)*sin(AL3) - sin(AL3)*cos(TH2)/16331239353195370) + (sin(TH2)*sin(AL3)/16331239353195370 + sin(AL3)*cos(TH2))*(-2*QY->*2 - 2*QZ->*2 + 1);
    dk_cpu.m[0][2] = TMP.data;


    // A_CPU_14
    TMP =  TX + A1*(-2*QY->*2 - 2*QZ->*2 + 1) + D2*(2*QW*QY + 2*QX*QZ);
    dk_cpu.m[0][3] = TMP.data;


    // A_CPU_21
    TMP =  -(-2*QW*QX + 2*QY*QZ)*sin(AL3) + (2*QW*QZ + 2*QX*QY)*(sin(TH2)*cos(AL3)/16331239353195370 - sin(TH2)/16331239353195370 + cos(TH2)*cos(AL3) + cos(TH2)/266709378811357127073829389436900) + (-2*QX->*2 - 2*QZ->*2 + 1)*(sin(TH2)*cos(AL3) + sin(TH2)/266709378811357127073829389436900 - cos(TH2)*cos(AL3)/16331239353195370 + cos(TH2)/16331239353195370);
    dk_cpu.m[1][0] = TMP.data;


    // A_CPU_22
    TMP =  (-2*QW*QX + 2*QY*QZ)*sin(AL3)/16331239353195370 + (2*QW*QZ + 2*QX*QY)*(-sin(TH2)*cos(AL3)/266709378811357127073829389436900 - sin(TH2) - cos(TH2)*cos(AL3)/16331239353195370 + cos(TH2)/16331239353195370) + (-2*QX->*2 - 2*QZ->*2 + 1)*(-sin(TH2)*cos(AL3)/16331239353195370 + sin(TH2)/16331239353195370 + cos(TH2)*cos(AL3)/266709378811357127073829389436900 + cos(TH2));
    dk_cpu.m[1][1] = TMP.data;


    // A_CPU_23
    TMP =  (-2*QW*QX + 2*QY*QZ)*cos(AL3) + (2*QW*QZ + 2*QX*QY)*(sin(TH2)*sin(AL3)/16331239353195370 + sin(AL3)*cos(TH2)) + (sin(TH2)*sin(AL3) - sin(AL3)*cos(TH2)/16331239353195370)*(-2*QX->*2 - 2*QZ->*2 + 1);
    dk_cpu.m[1][2] = TMP.data;


    // A_CPU_24
    TMP =  TY + A1*(2*QW*QZ + 2*QX*QY) + D2*(-2*QW*QX + 2*QY*QZ);
    dk_cpu.m[1][3] = TMP.data;


    // A_CPU_31
    TMP =  (2*QW*QX + 2*QY*QZ)*(sin(TH2)*cos(AL3) + sin(TH2)/266709378811357127073829389436900 - cos(TH2)*cos(AL3)/16331239353195370 + cos(TH2)/16331239353195370) + (-2*QW*QY + 2*QX*QZ)*(sin(TH2)*cos(AL3)/16331239353195370 - sin(TH2)/16331239353195370 + cos(TH2)*cos(AL3) + cos(TH2)/266709378811357127073829389436900) - (-2*QX->*2 - 2*QY->*2 + 1)*sin(AL3);
    dk_cpu.m[2][0] = TMP.data;


    // A_CPU_32
    TMP =  (2*QW*QX + 2*QY*QZ)*(-sin(TH2)*cos(AL3)/16331239353195370 + sin(TH2)/16331239353195370 + cos(TH2)*cos(AL3)/266709378811357127073829389436900 + cos(TH2)) + (-2*QW*QY + 2*QX*QZ)*(-sin(TH2)*cos(AL3)/266709378811357127073829389436900 - sin(TH2) - cos(TH2)*cos(AL3)/16331239353195370 + cos(TH2)/16331239353195370) + (-2*QX->*2 - 2*QY->*2 + 1)*sin(AL3)/16331239353195370;
    dk_cpu.m[2][1] = TMP.data;


    // A_CPU_33
    TMP =  (2*QW*QX + 2*QY*QZ)*(sin(TH2)*sin(AL3) - sin(AL3)*cos(TH2)/16331239353195370) + (-2*QW*QY + 2*QX*QZ)*(sin(TH2)*sin(AL3)/16331239353195370 + sin(AL3)*cos(TH2)) + (-2*QX->*2 - 2*QY->*2 + 1)*cos(AL3);
    dk_cpu.m[2][2] = TMP.data;


    // A_CPU_34
    TMP =  TZ + A1*(-2*QW*QY + 2*QX*QZ) + D2*(-2*QX->*2 - 2*QY->*2 + 1);
    dk_cpu.m[2][3] = TMP.data;


    // A_CPU_41
    dk_cpu.m[3][0] =  0;


    // A_CPU_42
    dk_cpu.m[3][1] =  0;


    // A_CPU_43
    dk_cpu.m[3][2] =  0;


    // A_CPU_44
    dk_cpu.m[3][3] =  1;


    return dk_cpu;

}


// CPU function to calculate direct kinematics form World to Lidar.
// Return reduced Homogenous Transformatin Matrix (9 instead of 16 elements)
HTMatrixLidarCPU dkWorldToLidarReduced(
    double tx,
    double ty,
    double tz,
    double qx,
    double qy,
    double qz,
    double qw,
    double th2,
    const double a1,
    const double d2,
    const double al3
)
{

        OctaveVariable <double> TX(tx);
        OctaveVariable <double> TY(ty);
        OctaveVariable <double> TZ(tz);
        OctaveVariable <double> QX(qx);
        OctaveVariable <double> QY(qy);
        OctaveVariable <double> QZ(qz);
        OctaveVariable <double> QW(qw);
        OctaveVariable <double> TH2(th2);
        OctaveVariable <double> A1(a1);
        OctaveVariable <double> D2(d2);
        OctaveVariable <double> AL3(al3);

        OctaveVariable <double> TMP;

        HTMatrixLidarCPU dk_cpu;

        // A_CPU_11
        TMP =  -(2*QW*QY + 2*QX*QZ)*sin(AL3) + (-2*QW*QZ + 2*QX*QY)*(sin(TH2)*cos(AL3) + sin(TH2)/266709378811357127073829389436900 - cos(TH2)*cos(AL3)/16331239353195370 + cos(TH2)/16331239353195370) + (-2*QY->*2 - 2*QZ->*2 + 1)*(sin(TH2)*cos(AL3)/16331239353195370 - sin(TH2)/16331239353195370 + cos(TH2)*cos(AL3) + cos(TH2)/266709378811357127073829389436900);
        dk_cpu.m_0 = TMP.data;


        // A_CPU_12
        TMP =  (2*QW*QY + 2*QX*QZ)*sin(AL3)/16331239353195370 + (-2*QW*QZ + 2*QX*QY)*(-sin(TH2)*cos(AL3)/16331239353195370 + sin(TH2)/16331239353195370 + cos(TH2)*cos(AL3)/266709378811357127073829389436900 + cos(TH2)) + (-2*QY->*2 - 2*QZ->*2 + 1)*(-sin(TH2)*cos(AL3)/266709378811357127073829389436900 - sin(TH2) - cos(TH2)*cos(AL3)/16331239353195370 + cos(TH2)/16331239353195370);
        dk_cpu.m_1 = TMP.data;


        // A_CPU_14
        TMP =  TX + A1*(-2*QY->*2 - 2*QZ->*2 + 1) + D2*(2*QW*QY + 2*QX*QZ);
        dk_cpu.m_3 = TMP.data;


        // A_CPU_21
        TMP =  -(-2*QW*QX + 2*QY*QZ)*sin(AL3) + (2*QW*QZ + 2*QX*QY)*(sin(TH2)*cos(AL3)/16331239353195370 - sin(TH2)/16331239353195370 + cos(TH2)*cos(AL3) + cos(TH2)/266709378811357127073829389436900) + (-2*QX->*2 - 2*QZ->*2 + 1)*(sin(TH2)*cos(AL3) + sin(TH2)/266709378811357127073829389436900 - cos(TH2)*cos(AL3)/16331239353195370 + cos(TH2)/16331239353195370);
        dk_cpu.m_4 = TMP.data;


        // A_CPU_22
        TMP =  (-2*QW*QX + 2*QY*QZ)*sin(AL3)/16331239353195370 + (2*QW*QZ + 2*QX*QY)*(-sin(TH2)*cos(AL3)/266709378811357127073829389436900 - sin(TH2) - cos(TH2)*cos(AL3)/16331239353195370 + cos(TH2)/16331239353195370) + (-2*QX->*2 - 2*QZ->*2 + 1)*(-sin(TH2)*cos(AL3)/16331239353195370 + sin(TH2)/16331239353195370 + cos(TH2)*cos(AL3)/266709378811357127073829389436900 + cos(TH2));
        dk_cpu.m_5 = TMP.data;


        // A_CPU_24
        TMP =  TY + A1*(2*QW*QZ + 2*QX*QY) + D2*(-2*QW*QX + 2*QY*QZ);
        dk_cpu.m_7 = TMP.data;


        // A_CPU_31
        TMP =  (2*QW*QX + 2*QY*QZ)*(sin(TH2)*cos(AL3) + sin(TH2)/266709378811357127073829389436900 - cos(TH2)*cos(AL3)/16331239353195370 + cos(TH2)/16331239353195370) + (-2*QW*QY + 2*QX*QZ)*(sin(TH2)*cos(AL3)/16331239353195370 - sin(TH2)/16331239353195370 + cos(TH2)*cos(AL3) + cos(TH2)/266709378811357127073829389436900) - (-2*QX->*2 - 2*QY->*2 + 1)*sin(AL3);
        dk_cpu.m_8 = TMP.data;


        // A_CPU_32
        TMP =  (2*QW*QX + 2*QY*QZ)*(-sin(TH2)*cos(AL3)/16331239353195370 + sin(TH2)/16331239353195370 + cos(TH2)*cos(AL3)/266709378811357127073829389436900 + cos(TH2)) + (-2*QW*QY + 2*QX*QZ)*(-sin(TH2)*cos(AL3)/266709378811357127073829389436900 - sin(TH2) - cos(TH2)*cos(AL3)/16331239353195370 + cos(TH2)/16331239353195370) + (-2*QX->*2 - 2*QY->*2 + 1)*sin(AL3)/16331239353195370;
        dk_cpu.m_9 = TMP.data;


        // A_CPU_34
        TMP =  TZ + A1*(-2*QW*QY + 2*QX*QZ) + D2*(-2*QX->*2 - 2*QY->*2 + 1);
        dk_cpu.m_11 = TMP.data;

        return dk_cpu;
}
