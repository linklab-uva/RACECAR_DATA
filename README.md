# RACECAR Dataset
Welcome to the RACECAR dataset!
![](https://user-images.githubusercontent.com/25155124/222313249-147a2e71-85da-42de-ac57-becb97a47dbf.png)

The RACECAR dataset is a collection of racing scenarios with full scale and high speed autonomous Indy Lights cars. These vehicles captured a plethora of LiDAR, Camera, Radar, GNSS, and IMU data during the 2021-2022 racing season of the [Indy Autonomous Challenge](https://www.indyautonomouschallenge.com).

This repository contains scripts used to parse the dataset, custom ros messages describing GNSS/IMU/Radar data, and a conversion script that converts ros2 bags to [nuScenes](https://www.nuscenes.org/nuscenes) json files.

Please contact the corresponding author at ark8su@virginia.edu for access to the dataset.


## Data Usage and License
This work is licensed under the Creative Commons Attribution-NonCommercial 4.0 International Public License (CC BY-NC 4.0). To obtain a copy of this license, see LICENSE-CC-BY-NC-4.0.txt in the archive, visit CreativeCommons.org or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

Under the following terms:

Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
NonCommercial — You may not use the material for commercial purposes.
No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.

## Data Organization

Each autonomous run from the AV21 was captured in the form of rosbags,

### Dataset Folder Structure

```
RACECAR
    ├── S1
    │   ├── C_SOLO-SLOW-70
    │   │   ├── metadata.yaml
    │   │   └── SOLO-SLOW-70.db3
    │   ├── M_SOLO-SLOW-70
    │   │   ├── metadata.yaml
    │   │   └── SOLO-SLOW-70.db3
    │   └── P_SOLO-SLOW-70
    │       ├── metadata.yaml
    │       └── SOLO-SLOW-70.db3
    ...
    ├── S6
    │   ├── E_MULTI-FAST-TUM
    │   │   ├── metadata.yaml
    │   │   └── MULTI-FAST-TUM.db3
    │   ├── T_MULTI-FAST-EURO
    │   │   ├── metadata.yaml
    │   │   └── MULTI-FAST-EURO.db3
    │   └── T_MULTI-FAST-POLI
    │       ├── metadata.yaml
    │       └── MULTI-FAST-POLI.db3
```

The ROS2 portion of the dataset is organized by scenario, with each folder within the scenario folder corresponding to a rosbag. Each of the scenario folders or specific rosbags can be replayed with the typical rosbag2 commands and can be downloaded independently or together.


### ROS2 Topics

All topics in the dataset are namespaced by their vehicle number. Each rosbag contains all sensor data available from the ego vehicle, and if a multi-agent label is included, it will be present as an `nav_msgs/msg/Odometry` topic named `local_odometry`.

If additional namespacing or merging is required, a script is included in the racecar_utils folder called rosbag_merger. The inbuilt rosbag2 cli tools available in ROS2 Humble are also useful for performing bag merging and conversion.

### Custom ROS2 Messages

The `delphi_esr_msgs`, `novatel_oem7_msgs`, and `novatel_gps_msgs` are the radar and gps messages obtained from the Autonomous Stuff and Novatel drivers. Install these packages in order to parse the radar and novatel custom messages in the dataset.

|Topic Name|Topic Type|Description|
|----------|----------|-----------|
|`camera/xxx/camera_info`| `sensor_msgs/msg/CameraInfo`|Distortion parameters and Intrinsic Camera Matrix|
|`camera/xxx/image/compressed`| `sensor_msgs/msg/CompressedImage`|Compressed camera image buffer and compression format|
|`luminar_points`| `sensor_msgs/msg/PointCloud2`|Merge LiDAR point cloud from all three sensors|
|`luminar_xxx_points`| `sensor_msgs/msg/PointCloud2`|LiDAR point cloud corresponding to xxx sensor|
|`novatel_xxx/bestgnsspos`| `novatel_oem7_msgs/msg/BESTPOS`|Best available GNSS solution from Novatel PwrPak. Measurement located at antenna phase center transmitted at 20 Hz|
|`novatel_xxx/bestpos`| `novatel_oem7_msgs/msg/BESTPOS`|Best available GNSS solution from Novatel PwrPak. Measurement located at antenna phase center transmitted at 20 Hz|
|`novatel_xxx/bestvel`| `novatel_oem7_msgs/msg/BESTVEL`|Velocity derived from differentiated position. Uses the same solution method from BESTPOS transmitted at 20 Hz|
|`novatel_xxx/heading2`| `novatel_oem7_msgs/msg/HEADING2`|Heading derived from alignment of dual antenna system at variable rate|
|`novatel_xxx/oem7raw`| `novatel_oem7_msgs/msg/Oem7RawMsg`|Binary data received from Novatel receivers before driver processing|
|`novatel_xxx/rawimu`| `novatel_oem7_msgs/msg/RAWIMU`|Accelerometer and Gyroscope data transmitted from receiver at 125 Hz|
|`novatel_xxx/rawimux`| `sensor_msgs/msg/Imu`|Accelerometer and Gyroscope data transmitted from receiver at 125 Hz|
|`novatel_xxx/time`| `novatel_oem7_msgs/msg/TIME`|Satellite time accompanying GNSS packets|
|`radar_front/esr_status1`| `delphi_esr_msgs/msg/EsrStatus1`||
|`radar_front/esr_status2`| `delphi_esr_msgs/msg/EsrStatus2`||
|`radar_front/esr_status3`| `delphi_esr_msgs/msg/EsrStatus3`||
|`radar_front/esr_status4`| `delphi_esr_msgs/msg/EsrStatus4`||
|`radar_front/esr_status5`| `delphi_esr_msgs/msg/EsrStatus5`||
|`radar_front/esr_status6`| `delphi_esr_msgs/msg/EsrStatus6`||
|`radar_front/esr_status7`| `delphi_esr_msgs/msg/EsrStatus7`||
|`radar_front/esr_status8`| `delphi_esr_msgs/msg/EsrStatus8`||
|`radar_front/esr_track`| `delphi_esr_msgs/msg/EsrTrack`|Radar detection|
|`radar_front/esr_valid1`| `delphi_esr_msgs/msg/EsrValid1`||
|`radar_front/esr_valid2`| `delphi_esr_msgs/msg/EsrValid2`||
|`radar_front/esr_vehicle1`| `delphi_esr_msgs/msg/EsrVehicle1`||
|`radar_front/esr_vehicle2`| `delphi_esr_msgs/msg/EsrVehicle2`||
|`radar_front/from_can_bus`| `can_msgs/msg/Frame`|Raw CAN data received from Aptiv ESR Radar|
|`radar_front/to_can_bus`| `can_msgs/msg/Frame`|Raw CAN data sent to Aptiv ESR Radar|
|`radar_front/radar_visz_moving`| `visualization_msgs/msg/Marker`|Visualization of radar detection|
|`radar_front/radar_visz_static`| `visualization_msgs/msg/Marker`|Visualization of radar detection|
|`radar_xxx/marker`| `visualization_msgs/msg/Marker`|Visualization of radar detection|
|`radar_xxx/detection`| `delphi_mrr_msgs/msg/Detection`|Detection from Aptiv MRR Radar|
|`local_odometry`| `nav_msgs/msg/Odometry`|Vehicle odometry in Cartesian coordinates derived from RTK GNSS solution|

## Using the Dataset

### Installation of Custom ROS2 Messages

- novatel_oem7_msgs
- novatel_gps_msgs
- delphi_esr_msgs
- can_msgs

First create a dev workspace that looks like the following. This will be our working directory.

```
├── data
│   └── RACECAR
└── racecar_ws
    ├── conversion_configs
    ├── urdf
    ├── racecar_py
    ├── racecar_utils
    ├── ros2_custom_msgs
    └── rosbag2nuscenes
```

In the working directory source your ROS2 installation, and build the packages in `racecar_utils` and `ros2_custom_msgs`. Source the installation folder in the working directory to then use the installed messages.
```
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
source install/setup.bash
```

The `can_msgs` package should be available via apt.

```
sudo apt install ros-${ROS_DISTRO}-can-msgs
```

