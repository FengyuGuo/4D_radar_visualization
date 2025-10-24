#include <ros/ros.h>
#include "radar_to_pc2/radar_to_pc2_converter.h"

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "radar_to_pc2_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    ROS_INFO("Starting radar_to_pc2_node");
    
    try {
        // Create converter instance
        RadarToPC2Converter converter(private_nh);
        
        // Spin to process callbacks
        ros::spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in radar_to_pc2_node: %s", e.what());
        return -1;
    }
    catch (...) {
        ROS_ERROR("Unknown exception in radar_to_pc2_node");
        return -1;
    }
    
    ROS_INFO("radar_to_pc2_node shutting down");
    return 0;
}
