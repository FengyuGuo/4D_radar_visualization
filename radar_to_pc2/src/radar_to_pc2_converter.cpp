#include "radar_to_pc2/radar_to_pc2_converter.h"

RadarToPC2Converter::RadarToPC2Converter(ros::NodeHandle& nh)
    : nh_(nh)
{
    // Read parameters
    nh_.param<std::string>("radar_topic", radar_topic_, "/radar_scan");
    nh_.param<std::string>("cloud_topic", cloud_topic_, "/radar_pointcloud");
    nh_.param<std::string>("output_frame", output_frame_, "radar");
    
    ROS_INFO("Radar topic: %s", radar_topic_.c_str());
    ROS_INFO("Cloud topic: %s", cloud_topic_.c_str());
    ROS_INFO("Output frame: %s", output_frame_.c_str());
    
    // Initialize subscriber and publisher
    radar_sub_ = nh_.subscribe(radar_topic_, 10, &RadarToPC2Converter::radarScanCallback, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic_, 10);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("radar_marker_array", 10);
    reset_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("radar_marker", 1, true);
    
    ROS_INFO("RadarToPC2Converter initialized successfully");
}

RadarToPC2Converter::~RadarToPC2Converter()
{
    ROS_INFO("RadarToPC2Converter destroyed");
}

void RadarToPC2Converter::radarScanCallback(const msgs_radar::RadarScanExtended::ConstPtr& msg)
{
    ROS_DEBUG("Received radar scan with %zu targets", msg->targets.size());
    
    // Create point cloud message
    sensor_msgs::PointCloud2 cloud_msg;
    visualization_msgs::MarkerArray marker_array;
    
    // Convert radar scan to point cloud
    convertRadarToPointCloud(msg, cloud_msg, marker_array);
    
    // Publish the point cloud
    cloud_pub_.publish(cloud_msg);
    // Publish reset marker to clear previous markers
    static ros::Time last_reset_time = ros::Time::now();
    if ((ros::Time::now() - last_reset_time).toSec() > 1.0) { // publish reset every 1 seconds
        last_reset_time = ros::Time::now();
        visualization_msgs::Marker reset_marker;
        reset_marker.action = visualization_msgs::Marker::DELETEALL;
        reset_marker.header = cloud_msg.header;
        reset_marker_pub_.publish(reset_marker);
    }
    // Publish the visualization markers
    marker_pub_.publish(marker_array);
}

void RadarToPC2Converter::convertRadarToPointCloud(const msgs_radar::RadarScanExtended::ConstPtr& radar_msg, 
                                                   sensor_msgs::PointCloud2& cloud_msg, visualization_msgs::MarkerArray& marker_array)
{
    // Set up the point cloud header
    cloud_msg.header = radar_msg->header;
    cloud_msg.header.frame_id = output_frame_;

    // Set up the visualization marker
    marker_array.markers.clear();
    
    // Set up point cloud fields
    cloud_msg.fields.clear();
    cloud_msg.fields.resize(7);
    
    // Position fields (x, y, z)
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;
    
    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;
    
    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;
    
    // Additional radar-specific fields
    cloud_msg.fields[3].name = "velocity";
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[3].count = 1;
    
    cloud_msg.fields[4].name = "snr";
    cloud_msg.fields[4].offset = 16;
    cloud_msg.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[4].count = 1;
    
    cloud_msg.fields[5].name = "power";
    cloud_msg.fields[5].offset = 20;
    cloud_msg.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[5].count = 1;
    
    cloud_msg.fields[6].name = "rcs";
    cloud_msg.fields[6].offset = 24;
    cloud_msg.fields[6].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[6].count = 1;
    
    // Set point cloud properties
    cloud_msg.is_bigendian = false;
    cloud_msg.point_step = 28; // 7 fields * 4 bytes each
    cloud_msg.width = radar_msg->targets.size();
    cloud_msg.height = 1;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.is_dense = true;
    
    // Resize data array
    cloud_msg.data.resize(cloud_msg.row_step);

    float min_rcs = std::numeric_limits<float>::max();
    float max_rcs = std::numeric_limits<float>::lowest();

    float rcs_range_min = -60.0f; // Example minimum RCS value for marker scaling
    float rcs_range_max = 50.0f;   // Example maximum RCS value for marker scaling

    float power_range_min = 50.0f; // Example minimum power value for marker scaling
    float power_range_max = 110.0f;    // Example maximum power value for marker scaling

    float velocity_range_min = -20.0f; // Example minimum velocity value for marker scaling
    float velocity_range_max = 20.0f;   // Example maximum velocity value for marker scaling

    static int max_points_size = 0;
    if (radar_msg->targets.size() > max_points_size) {
        max_points_size = radar_msg->targets.size();
        ROS_INFO("New max radar points size: %d", max_points_size);
    }

    // This is where you would convert from spherical coordinates (range, azimuth, elevation)
    // to Cartesian coordinates (x, y, z) and fill the point cloud data
    for (size_t i = 0; i < radar_msg->targets.size(); ++i) {
        const auto& target = radar_msg->targets[i];
        float range = target.range;
        float azimuth = target.azimuth;
        float elevation = target.elevation;

        // Convert spherical to Cartesian coordinates
        float x = range * cos(elevation) * cos(azimuth);
        float y = range * cos(elevation) * sin(azimuth);
        float z = range * sin(elevation);

        // Fill the point cloud data
        memcpy(&cloud_msg.data[i * cloud_msg.point_step], &x, sizeof(float));
        memcpy(&cloud_msg.data[i * cloud_msg.point_step + 4], &y, sizeof(float));
        memcpy(&cloud_msg.data[i * cloud_msg.point_step + 8], &z, sizeof(float));
        memcpy(&cloud_msg.data[i * cloud_msg.point_step + 12], &target.velocity, sizeof(float));
        memcpy(&cloud_msg.data[i * cloud_msg.point_step + 16], &target.snr, sizeof(float));
        memcpy(&cloud_msg.data[i * cloud_msg.point_step + 20], &target.power, sizeof(float));
        memcpy(&cloud_msg.data[i * cloud_msg.point_step + 24], &target.rcs, sizeof(float));

        // Update min and max RCS
        if (target.rcs < min_rcs) min_rcs = target.rcs;
        if (target.rcs > max_rcs) max_rcs = target.rcs;

        // Create a marker for this target
        visualization_msgs::Marker marker;
        marker.header = radar_msg->header;
        marker.header.frame_id = output_frame_;
        marker.ns = "radar_targets";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.color.a = 0.8; // Semi-transparent
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        float rcs_clamped = std::max(rcs_range_min, std::min(rcs_range_max, target.rcs));
        float power_clamped = std::max(power_range_min, std::min(power_range_max, target.power));
        // Scale size based on power
        // float marker_size = (power_clamped - power_range_min) / (power_range_max - power_range_min) * 1.5f + 0.01f;
        float marker_size = (rcs_clamped - rcs_range_min) / (rcs_range_max - rcs_range_min) * 1.5 + 0.01; // Scale size based on RCS
        marker.scale.x = marker_size;
        marker.scale.y = marker_size;
        marker.scale.z = marker_size;
        // marker.lifetime = ros::Duration(0.1); // Short lifetime to ensure timely updates
        marker_array.markers.push_back(marker);

        visualization_msgs::Marker velocity_marker;
        velocity_marker.header = radar_msg->header;
        velocity_marker.header.frame_id = output_frame_;
        velocity_marker.ns = "radar_target_velocity";
        velocity_marker.id = i;
        velocity_marker.type = visualization_msgs::Marker::ARROW;
        velocity_marker.action = visualization_msgs::Marker::ADD;
        velocity_marker.pose.orientation.w = 1.0;
        velocity_marker.pose.position.x = 0.0;
        velocity_marker.pose.position.y = 0.0;
        velocity_marker.pose.position.z = 0.0;
        velocity_marker.pose.orientation.x = 0.0;
        velocity_marker.pose.orientation.y = 0.0;
        velocity_marker.pose.orientation.z = 0.0;
        // Set arrow direction based on velocity
        float arrow_length = 3.0f; // Fixed length for visibility
        float dist = sqrt(x*x + y*y + z*z);
        velocity_marker.points.resize(2);
        velocity_marker.points[0].x = x;
        velocity_marker.points[0].y = y;
        velocity_marker.points[0].z = z;
        velocity_marker.points[1].x = x + (target.velocity / velocity_range_max) * arrow_length * (x / dist);
        velocity_marker.points[1].y = y + (target.velocity / velocity_range_max) * arrow_length * (y / dist);
        velocity_marker.points[1].z = z + (target.velocity / velocity_range_max) * arrow_length * (z / dist);
        velocity_marker.scale.x = 0.1f * marker_size; // Shaft diameter
        velocity_marker.scale.y = 0.2f * marker_size; // Head diameter
        velocity_marker.scale.z = 0.2f; // Head length
        velocity_marker.color.a = 0.8f; // Semi-transparent
        velocity_marker.color.r = 1.0f;
        velocity_marker.color.g = 0.0f;
        velocity_marker.color.b = 0.0f;
        // velocity_marker.lifetime = ros::Duration(0.1); // Short lifetime to ensure timely updates
        marker_array.markers.push_back(velocity_marker);
    }

    // Fill in the marker array with null markers to clear previous markers if needed
    // for (size_t i = radar_msg->targets.size(); i < max_points_size; ++i) {
    //     visualization_msgs::Marker null_marker;
    //     null_marker.header = radar_msg->header;
    //     null_marker.header.frame_id = output_frame_;
    //     null_marker.ns = "radar_targets";
    //     null_marker.id = i;
    //     null_marker.action = visualization_msgs::Marker::DELETE;
    //     marker_array.markers.push_back(null_marker);
    // }

    ROS_DEBUG("RCS range: min=%.2f, max=%.2f\t", min_rcs, max_rcs);

    ROS_DEBUG("Created point cloud with %d points", cloud_msg.width);
    
    // For now, just log the number of targets received
    if (radar_msg->targets.size() > 0) {
        ROS_DEBUG("First target: range=%.2f, azimuth=%.2f, elevation=%.2f, velocity=%.2f", 
                  radar_msg->targets[0].range,
                  radar_msg->targets[0].azimuth,
                  radar_msg->targets[0].elevation,
                  radar_msg->targets[0].velocity);
    }
}
