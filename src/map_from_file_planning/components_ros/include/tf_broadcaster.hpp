#ifndef TF_BROADCASTER_HPP_
#define TF_BROADCASTER_HPP_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

class TfBroadcaster{
    ros::NodeHandle *nh;
    nav_msgs::Odometry *msg_odom;

    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;

public:
     TfBroadcaster(ros::NodeHandle *nh, nav_msgs::Odometry *msg_odom){
         this->nh = nh;
         this->msg_odom = msg_odom;
         this->odom_trans.header.frame_id = "odom";
         this->odom_trans.child_frame_id = "base_link";
     }

     void updateTf(){
         this->odom_trans.header.seq = msg_odom->header.seq;
         this->odom_trans.header.stamp = msg_odom->header.stamp;

         this->odom_trans.transform.translation.x = msg_odom->pose.pose.position.x;
         this->odom_trans.transform.translation.y = msg_odom->pose.pose.position.y;
         this->odom_trans.transform.translation.z = msg_odom->pose.pose.position.z;

         this->odom_trans.transform.rotation.x = msg_odom->pose.pose.orientation.x;
         this->odom_trans.transform.rotation.y = msg_odom->pose.pose.orientation.y;
         this->odom_trans.transform.rotation.z = msg_odom->pose.pose.orientation.z;
         this->odom_trans.transform.rotation.w = msg_odom->pose.pose.orientation.w;

         odom_broadcaster.sendTransform(odom_trans);
     }
};
#endif
