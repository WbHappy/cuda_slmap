#ifndef ROS_BUFFOR_HPP_
#define ROS_BUFFOR_HPP_

#include "components_ros.hpp"
#include "components_gpu.cuh"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>

class _ROSBuffor
{
public:
    // SUBSCRIBED
    geometry_msgs::PoseStamped goal;

    nav_msgs::Odometry odom;

    std_msgs::Float64 lidar_pose;

    sensor_msgs::LaserScan laser_scan;

    // PUBLISHED
    nav_msgs::Path path;

    geometry_msgs::PoseStamped path_point;

public:
    _ROSBuffor(){
        path.header.seq = 0;
        path.header.frame_id = "/odom";

        path_point.header.seq = 0;
        path_point.header.frame_id = "/odom";

    }

    void updatePath(GpuPath *host_path)
    {
        path.header.stamp = ros::Time::now();
        path_point.header.stamp = path.header.stamp;

        path.poses.clear();
        for(int i = 0; i < host_path->total_size; i++)
        {

            path_point.header.seq = i;
            path_point.pose.position.x = host_path->p[i].x;
            path_point.pose.position.y = host_path->p[i].y;

            path.poses.push_back(path_point);

        }

        path.header.seq++;
    }

    void debugInfo()
    {
        ROS_INFO("GOAL: {%f %f %f}", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

        ROS_INFO("ODOM: {%f %f %f | %f %f %f %f}", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);

        ROS_INFO("LIDAR POSE: {%f}", lidar_pose.data);

        ROS_INFO("LASER SCAN:");
        printf("{");
        for( int i = 0; i < laser_scan.ranges.size(); i++)
        {
            printf(" %f", laser_scan.ranges[i]);
        }
        printf(" }\n");
    }
};


#endif
