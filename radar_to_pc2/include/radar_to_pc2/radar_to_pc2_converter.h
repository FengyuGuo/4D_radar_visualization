#ifndef RADAR_TO_PC2_CONVERTER_H
#define RADAR_TO_PC2_CONVERTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <msgs_radar/RadarScanExtended.h>
#include <msgs_radar/RadarTargetExtended.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class RadarToPC2Converter
{
public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     */
    RadarToPC2Converter(ros::NodeHandle& nh);
    
    /**
     * @brief Destructor
     */
    ~RadarToPC2Converter();

private:
    /**
     * @brief Callback function for radar scan messages
     * @param msg Radar scan extended message
     */
    void radarScanCallback(const msgs_radar::RadarScanExtended::ConstPtr& msg);
    
    /**
     * @brief Convert radar scan to point cloud
     * @param radar_msg Input radar scan message
     * @param cloud_msg Output point cloud message
     */
    void convertRadarToPointCloud(const msgs_radar::RadarScanExtended::ConstPtr& radar_msg, 
                                  sensor_msgs::PointCloud2& cloud_msg, visualization_msgs::MarkerArray& marker_array);

    // ROS node handle
    ros::NodeHandle nh_;
    
    // Subscriber for radar data
    ros::Subscriber radar_sub_;
    
    // Publisher for point cloud data
    ros::Publisher cloud_pub_;
    
    // Publisher for visualization markers
    ros::Publisher marker_pub_, reset_marker_pub_;
    
    // Parameters
    std::string radar_topic_;
    std::string cloud_topic_;
    std::string output_frame_;
};

#endif // RADAR_TO_PC2_CONVERTER_H
