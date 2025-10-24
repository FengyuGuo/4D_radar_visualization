# Tools to visualization 4D radar dataset

dataset:

https://robotics.sjtu.edu.cn/en/xwxshd/1203.html

# Dependencies:

https://github.com/HesaiTechnology/HesaiLidar_Swift_ROS

# Usage:

- compile the source

- modify the path of dataset

```
  <arg name="pcap" default="/media/guo/fs/4D_radar_dataset/Parking_lot/parking_lot_lidar.pcap" />
  <arg name="radar_dataset" default="/media/guo/fs/4D_radar_dataset/Parking_lot/parking_lot.bag" />
```

- launch node

```
roslaunch radar_to_pc2 radar_to_pc2.launch
```

- check the data in rviz

```
rviz -d radar.rviz
```