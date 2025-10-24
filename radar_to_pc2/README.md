# radar_to_pc2

This ROS package converts radar data to PointCloud2 format.

## Description

This package is designed to convert radar scan data (using msgs_radar) to standard ROS PointCloud2 messages for visualization and processing in ROS.

## Dependencies

- roscpp
- rospy
- std_msgs
- sensor_msgs
- msgs_radar

## Build

```bash
cd ~/radar_ws
catkin_make
```

## Usage

### Parameters

The node accepts the following ROS parameters:

- `radar_topic` (string, default: "/radar_scan"): Input topic name for RadarScanExtended messages
- `cloud_topic` (string, default: "/radar_pointcloud"): Output topic name for PointCloud2 messages  
- `output_frame` (string, default: "radar"): Frame ID for the output point cloud

### Running the Node

#### Method 1: Using launch file
```bash
roslaunch radar_to_pc2 radar_to_pc2.launch
```

#### Method 2: Using rosrun with parameters
```bash
rosrun radar_to_pc2 radar_to_pc2_node _radar_topic:=/your_radar_topic _cloud_topic:=/your_cloud_topic _output_frame:=your_frame
```

### Message Format

**Input:** `msgs_radar/RadarScanExtended`
- Contains an array of radar targets with range, azimuth, elevation, velocity, SNR, power, RCS, etc.

**Output:** `sensor_msgs/PointCloud2`  
- Point cloud with the following fields:
  - `x, y, z`: Cartesian coordinates (converted from spherical)
  - `velocity`: Target velocity
  - `snr`: Signal-to-noise ratio
  - `power`: Power measurement
  - `rcs`: Radar cross section

### Note

The conversion logic from spherical to Cartesian coordinates is not yet implemented. The framework is ready for implementation.

## Author

TODO: Add author information
