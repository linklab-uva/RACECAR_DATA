# RACECAR Dataset
Welcome to the RACECAR dataset!
![](https://user-images.githubusercontent.com/25155124/222313249-147a2e71-85da-42de-ac57-becb97a47dbf.png)

The RACECAR dataset is a collection of racing scenarios with full scale and high speed autonomous Indy Lights cars. These vehicles captured a plethora of LiDAR, Camera, Radar, GNSS, and IMU data during the 2021-2022 racing season of the [Indy Autonomous Challenge](https://www.indyautonomouschallenge.com).

This repository contains scripts used to parse the dataset, custom ros messages describing GNSS/IMU/Radar data, and a conversion script that converts ros2 bags to [nuScenes](https://www.nuscenes.org/nuscenes) json files.

Please contact the corresponding author at ark8su@virginia.edu for access to the dataset.

## Data Organization

### Scenarios

|Topic Name|Topic Type|Description|
|----------|----------|-----------|
|`/camera/xxx/camera_info`| `sensor_msgs/msg/CameraInfo`|Distortion parameters and Intrinsic Camera Matrix|
|`/camera/xxx/image/compressed`| `sensor_msgs/msg/CompressedImage`|Compressed camera image buffer and compression format|
|`/luminar_points`| `sensor_msgs/msg/PointCloud2`|Merge LiDAR point cloud from all three sensors|
|`/luminar_xxx_points`| `sensor_msgs/msg/PointCloud2`|LiDAR point cloud corresponding to xxx sensor|
|`/novatel_xxx/bestgnsspos`| `novatel_oem7_msgs/msg/BESTPOS`|Best available GNSS solution from Novatel PwrPak. Measurement located at antenna phase center transmitted at 20 Hz|
|`/novatel_xxx/bestpos`| `novatel_oem7_msgs/msg/BESTPOS`|Best available GNSS solution from Novatel PwrPak. Measurement located at antenna phase center transmitted at 20 Hz|
|`/novatel_xxx/bestvel`| `novatel_oem7_msgs/msg/BESTVEL`|Velocity derived from differentiated position. Uses the same solution method from BESTPOS transmitted at 20 Hz|
|`/novatel_xxx/heading2`| `novatel_oem7_msgs/msg/HEADING2`|Heading derived from alignment of dual antenna system at variable rate|
|`/novatel_xxx/oem7raw`| `novatel_oem7_msgs/msg/Oem7RawMsg`|Binary data received from Novatel receivers before driver processing|
|`/novatel_xxx/rawimu`| `novatel_oem7_msgs/msg/RAWIMU`|Accelerometer and Gyroscope data transmitted from receiver at 125 Hz|
|`/novatel_xxx/rawimux`| `sensor_msgs/msg/Imu`|Accelerometer and Gyroscope data transmitted from receiver at 125 Hz|
|`/novatel_xxx/time`| `novatel_oem7_msgs/msg/TIME`|Satellite time accompanying GNSS packets|
|`/radar_front/esr_status1`| `delphi_esr_msgs/msg/EsrStatus1`||
|`/radar_front/esr_status2`| `delphi_esr_msgs/msg/EsrStatus2`||
|`/radar_front/esr_status3`| `delphi_esr_msgs/msg/EsrStatus3`||
|`/radar_front/esr_status4`| `delphi_esr_msgs/msg/EsrStatus4`||
|`/radar_front/esr_status5`| `delphi_esr_msgs/msg/EsrStatus5`||
|`/radar_front/esr_status6`| `delphi_esr_msgs/msg/EsrStatus6`||
|`/radar_front/esr_status7`| `delphi_esr_msgs/msg/EsrStatus7`||
|`/radar_front/esr_status8`| `delphi_esr_msgs/msg/EsrStatus8`||
|`/radar_front/esr_track`| `delphi_esr_msgs/msg/EsrTrack`|Radar detection|
|`/radar_front/esr_valid1`| `delphi_esr_msgs/msg/EsrValid1`||
|`/radar_front/esr_valid2`| `delphi_esr_msgs/msg/EsrValid2`||
|`/radar_front/esr_vehicle1`| `delphi_esr_msgs/msg/EsrVehicle1`||
|`/radar_front/esr_vehicle2`| `delphi_esr_msgs/msg/EsrVehicle2`||
|`/radar_front/from_can_bus`| `can_msgs/msg/Frame`|Raw CAN data received from Aptiv ESR Radar|
|`/radar_front/to_can_bus`| `can_msgs/msg/Frame`|Raw CAN data sent to Aptiv ESR Radar|
|`/radar_front/radar_visz_moving`| `visualization_msgs/msg/Marker`|Visualization of radar detection|
|`/radar_front/radar_visz_static`| `visualization_msgs/msg/Marker`|Visualization of radar detection|
|`/radar_xxx/marker`| `visualization_msgs/msg/Marker`|Visualization of radar detection|
|`/radar_xxx/detection`| `delphi_mrr_msgs/msg/Detection`|Detection from Aptiv MRR Radar|
|`/local_odometry`| `nav_msgs/msg/Odometry`|Vehicle odometry in Cartesian coordinates derived from RTK GNSS solution|
