
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <ros/console.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl/io/pcd_io.h>
#include <boost/lexical_cast.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <stdlib.h>
#include <string>
#include "hdl_msgs/getPCloudSweep.h"
#include <stdio.h>
#include <nav_msgs/Odometry.h>

bool isOdom=false;
hdl_msgs::getPCloudSweep call_srv;
ros::ServiceClient request_pcl_srv_;

void hdlSlamCloudCallback(const nav_msgs::OdometryConstPtr& msg)
{
    isOdom = true;
    if(request_pcl_srv_.call(call_srv)){
    ROS_INFO("Request PointCloud velodyne");
    }else{
        ROS_WARN("Request PointCloud velodyne - failed");
    } 
}

int main(int argc, char** argv){

    ros::init(argc, argv, "request_cloud");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");
    
    
    ROS_INFO("\033[1;32m---->\033[0m Request cloud every sec.");

    ros::Rate loop_rate(1);
    call_srv.request.num=1;
    request_pcl_srv_ = node.serviceClient<hdl_msgs::getPCloudSweep>("request_pcloud_sweep");
    ros::Subscriber sub_hdl_slam_odom_= node.subscribe("/odom", 1, hdlSlamCloudCallback);

    if(request_pcl_srv_.call(call_srv)){
    ROS_INFO("Request PointCloud velodyne");
    }else{
        ROS_WARN("Request PointCloud velodyne - failed");
    } 
    while(ros::ok()) 
    { 

          
        if(isOdom==true)
        {
            ROS_INFO("IsOdom");
            isOdom = false;

        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}