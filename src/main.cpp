#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <time.h>
#include <string>
#include <sstream>
#include <exception>

int main(int argc, char **argv) {
    ros::init(argc, argv, "quadtree_node");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("quadtree_node - starts.");
   
    try {  
   
    }
    catch (std::exception& e) {
        ROS_ERROR(e.what());
    }
   
    ROS_INFO_STREAM("quadtree_node - TERMINATED.");
    return -1;
}