from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import deserialize_cdr, serialize_cdr
from rosbags.typesys import get_types_from_msg, register_types


from pathlib import Path
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm
from copy import copy, deepcopy

import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import pygeodesy
import math

def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)

def absoluteFilePaths(directory):
    for dirpath,_,filenames in os.walk(directory):
        for f in filenames:
            yield os.path.abspath(os.path.join(dirpath, f))

def importROSMsgs():
    # Import ROS2 Types
    abs_msg_list = absoluteFilePaths('novatel_oem7_msgs/msg')
    add_types = {}

    for pathstr in abs_msg_list:
        msgpath = Path(pathstr)
        msgdef = msgpath.read_text(encoding='utf-8')
        add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

    register_types(add_types)
    from rosbags.typesys.types import novatel_oem7_msgs__msg__BESTPOS as BESTPOS
    from rosbags.typesys.types import novatel_oem7_msgs__msg__BESTVEL as BESTVEL
    from rosbags.typesys.types import novatel_oem7_msgs__msg__Oem7Header as Oem7Header
    from rosbags.typesys.types import nav_msgs__msg__Odometry as Odometry
    from rosbags.typesys.types import geometry_msgs__msg__PoseWithCovariance as PoseWithCovariance
    from rosbags.typesys.types import geometry_msgs__msg__TwistWithCovariance as TwistWithCovariance
    from rosbags.typesys.types import geometry_msgs__msg__Pose as Pose
    from rosbags.typesys.types import geometry_msgs__msg__Point as Point
    from rosbags.typesys.types import geometry_msgs__msg__Quaternion as Quaternion
    from rosbags.typesys.types import geometry_msgs__msg__Twist as Twist
    from rosbags.typesys.types import geometry_msgs__msg__Vector3 as Vector3
    from rosbags.typesys.types import sensor_msgs__msg__NavSatFix as NavSatFix


def read_bag_file(bag_file, topics, topic_types):
    gps_messages = []
    vel_messages = []

    # create reader instance and open for reading
    with Reader(bag_file) as reader:

        # messages() accepts connection filters
        connections = [x for x in reader.connections if x.topic in topics]
        for connection, timestamp, rawdata in tqdm(reader.messages(connections=connections)):
            msg = deserialize_cdr(rawdata, connection.msgtype)
            if connection.msgtype == 'novatel_oem7_msgs/msg/BESTPOS':
                gps_messages.append((timestamp,msg))
            elif connection.msgtype == 'novatel_oem7_msgs/msg/BESTVEL':
                vel_messages.append((timestamp,msg))

    return gps_messages, vel_messages


def visualize_position(local_odom):
    x_pos = []
    y_pos = []
    time  = []
    for odom in local_odom:
        x_pos.append(odom[1].pose.pose.position.x)
        y_pos.append(odom[1].pose.pose.position.y)
        time.append(odom[0]*1e-9)

    plt.rcParams['figure.figsize'] = [50,25]

    fig, ax = plt.subplots(1, 1)
    ax.plot(time, x_pos, marker='o',color='r',linestyle='-', markersize = 1)
    ax.plot(time, y_pos, marker='o',color='g',linestyle='-', markersize = 1)
    plt.gca().set_aspect('equal', adjustable = 'box')
    plt.title('Time vs Position')
    plt.xlabel('time (s)')
    plt.ylabel('pos (m)')
    plt.show()

    
def gen_local_odom(gps_messages, vel_messages, track):
    
    from rosbags.typesys.types import nav_msgs__msg__Odometry as Odometry
    from rosbags.typesys.types import geometry_msgs__msg__PoseWithCovariance as PoseWithCovariance
    from rosbags.typesys.types import geometry_msgs__msg__TwistWithCovariance as TwistWithCovariance
    from rosbags.typesys.types import geometry_msgs__msg__Pose as Pose
    from rosbags.typesys.types import geometry_msgs__msg__Point as Point
    from rosbags.typesys.types import geometry_msgs__msg__Quaternion as Quaternion
    from rosbags.typesys.types import geometry_msgs__msg__Twist as Twist
    from rosbags.typesys.types import geometry_msgs__msg__Vector3 as Vector3
    from rosbags.typesys.types import sensor_msgs__msg__NavSatFix as NavSatFix

    deg2rad = math.pi/180.0

    local_odom_arr = []
    if track == 'ims':
        origin = (39.795398310617, -86.23476471193612, 223)
    elif track == 'lvms':
        origin = (36.272371177449344, -115.01030828834901, 594)
    gpsmap = pygeodesy.LocalCartesian(origin[0], origin[1], origin[2])

    prev_pos = [0.0,0.0,0.0]
    prev_time = gps_messages[0][0]-10000

    for ros_time, bestpos in tqdm(gps_messages):

        
        bestvel = None

        ros_time_vel = 0

        # Parse BESTPOS

        ros_header = bestpos.header
        ros_frame  = ros_header.frame_id
        gps_header = bestpos.nov_header
        gps_time   = gps_header.gps_week_milliseconds
        lat        = bestpos.lat
        lon        = bestpos.lon
        hgt        = bestpos.hgt
        lat_stdev  = bestpos.lat_stdev  
        lon_stdev  = bestpos.lon_stdev  
        hgt_stdev  = bestpos.hgt_stdev

        novel = True
        # Find Corresponding BESTVEL
        for time,msg in vel_messages:
            if msg.nov_header.gps_week_milliseconds == gps_time:
                ros_time_vel = time
                bestvel = msg
                novel = False
                break
            if msg.nov_header.gps_week_milliseconds > gps_time:
                break 
        

        # GPS 2 LOCAL CARTESIAN
        local_tuple = gpsmap.forward(lat, lon, hgt)
        if novel:
            heading = math.atan2(local_tuple[1] - prev_pos[1], local_tuple[0] - prev_pos[0])
            hor_speed = (((local_tuple[1] - prev_pos[1])**2+(local_tuple[0] - prev_pos[0])**2)**(0.5))/(ros_time-prev_time)
            # quat    = R.from_euler('z',heading).as_quat()
        else:
            ros_header_v = bestvel.header
            gps_header_v = bestvel.nov_header
            gps_time_v   = gps_header_v.gps_week_milliseconds
            trk_gnd      = bestvel.trk_gnd
            latency      = bestvel.latency
            hor_speed    = bestvel.hor_speed
            ver_speed    = bestvel.ver_speed
            heading      =-trk_gnd*deg2rad
        
        trk_r  = R.from_euler('z',heading)
        quat = (trk_r).as_quat()
        rot_stdev = (math.pi/2*180)

        prev_pos[0] = local_tuple[0]
        prev_pos[1] = local_tuple[1]
        prev_pos[2] = local_tuple[2]
        
        prev_time   =  ros_time
        
        
        ros_header.frame_id = 'map'
        # Populate Odom Message
        gps_to_ramg : np.ndarray = np.eye(4)
        gps_to_ramg[0,3]=-1.606
        maptogps : np.ndarray             = np.eye(4)
        maptogps[0:3,3]                   = np.asarray([local_tuple[0], local_tuple[1], local_tuple[2]])
        maptogps[0:3,0:3]                 = R.from_quat(np.asarray([quat[0], quat[1], quat[2], quat[3]])).as_matrix()
        ramg_pose : np.ndarray            = np.matmul(maptogps, gps_to_ramg)
        ramg_quat : np.ndarray            = R.from_matrix(ramg_pose[0:3,0:3]).as_quat()
        
        
        odom_point   = Point(x=ramg_pose[0,3],y=ramg_pose[1,3],z=ramg_pose[2,3])
        odom_quat    = Quaternion(x=ramg_quat[0],y=ramg_quat[1],z=ramg_quat[2],w=ramg_quat[3])
        linear_vel   = Vector3(x=hor_speed,y=0.0,z=0.0)
        angular_vel  = Vector3(x=0.0,y=0.0,z=0.0)
        odom_pose    = Pose(position=odom_point,orientation=odom_quat)
        odom_twist   = Twist(linear=linear_vel,angular=angular_vel)
        cov_pose     = np.zeros(36)
        cov_pose[0]  = lon_stdev*lon_stdev
        cov_pose[7]  = lat_stdev*lat_stdev
        cov_pose[14] = hgt_stdev*hgt_stdev
        cov_pose[21] = rot_stdev*rot_stdev
        cov_pose[28] = rot_stdev*rot_stdev
        cov_pose[35] = rot_stdev*rot_stdev
        cov_twist    = np.zeros(36)
        cov_twist[0] = .125*.125
        cov_twist[7] = .0025
        cov_twist[14]= .0025
        odom_posec   = PoseWithCovariance(pose=odom_pose,covariance=cov_pose)
        odom_twistc  = TwistWithCovariance(twist=odom_twist,covariance=cov_twist)
        
        local_odom = Odometry(header = ros_header,
                              child_frame_id = 'rear_axle_middle_ground',
                              pose = odom_posec,
                              twist= odom_twistc)

        local_odom_arr.append([ros_time, local_odom, gps_time])

    return local_odom_arr

def nav2odom(gps_messages, track):

    deg2rad = math.pi/180.0

    local_odom_arr = []
    if track == 'ims':
        origin = (39.795398310617, -86.23476471193612, 223)
    elif track == 'lvms':
        origin = (36.272371177449344, -115.01030828834901, 594)
    gpsmap = pygeodesy.LocalCartesian(origin[0], origin[1], origin[2])

    prev_pos = [0.0,0.0,0.0]
    prev_time = gps_messages[0][0]-10000

    for ros_time, msg in tqdm(gps_messages):

        
        bestvel = None

        ros_time_vel = 0

        # Parse NavSatFix

        ros_header = msg.header
        ros_frame  = ros_header.frame_id
        lat        = msg.latitude
        lon        = msg.longitude
        hgt        = msg.altitude
        lat_stdev  = msg.position_covariance[0]
        lon_stdev  = msg.position_covariance[4] 
        hgt_stdev  = msg.position_covariance[8]

        # GPS 2 LOCAL CARTESIAN
        local_tuple = gpsmap.forward(lat, lon, hgt)
        heading = math.atan2(local_tuple[1] - prev_pos[1], local_tuple[0] - prev_pos[0])
        hor_speed = (((local_tuple[1] - prev_pos[1])**2+(local_tuple[0] - prev_pos[0])**2)**(0.5))/(ros_time-prev_time)
        
        trk_r  = R.from_euler('z',heading)
        quat = (trk_r).as_quat()
        rot_stdev = (math.pi/2*180)
        prev_pos[0] = local_tuple[0]
        prev_pos[1] = local_tuple[1]
        prev_pos[2] = local_tuple[2]
        prev_time   =  ros_time
        
        ros_header.frame_id = 'map'
        # Populate Odom Message
        gps_to_ramg : np.ndarray = np.eye(4)
        gps_to_ramg[0,3]=-1.606
        maptogps : np.ndarray             = np.eye(4)
        maptogps[0:3,3]                   = np.asarray([local_tuple[0], local_tuple[1], local_tuple[2]])
        maptogps[0:3,0:3]                 = R.from_quat(np.asarray([quat[0], quat[1], quat[2], quat[3]])).as_matrix()
        ramg_pose : np.ndarray            = np.matmul(maptogps, gps_to_ramg)
        ramg_quat : np.ndarray            = R.from_matrix(ramg_pose[0:3,0:3]).as_quat()
        
        
        odom_point   = Point(x=ramg_pose[0,3],y=ramg_pose[1,3],z=ramg_pose[2,3])
        odom_quat    = Quaternion(x=ramg_quat[0],y=ramg_quat[1],z=ramg_quat[2],w=ramg_quat[3])
        linear_vel   = Vector3(x=hor_speed,y=0.0,z=0.0)
        angular_vel  = Vector3(x=0.0,y=0.0,z=0.0)
        odom_pose    = Pose(position=odom_point,orientation=odom_quat)
        odom_twist   = Twist(linear=linear_vel,angular=angular_vel)
        cov_pose     = np.zeros(36)
        cov_pose[0]  = lon_stdev*lon_stdev
        cov_pose[7]  = lat_stdev*lat_stdev
        cov_pose[14] = hgt_stdev*hgt_stdev
        cov_pose[21] = rot_stdev*rot_stdev
        cov_pose[28] = rot_stdev*rot_stdev
        cov_pose[35] = rot_stdev*rot_stdev
        cov_twist    = np.zeros(36)
        cov_twist[0] = .125*.125
        cov_twist[7] = .0025
        cov_twist[14]= .0025
        odom_posec   = PoseWithCovariance(pose=odom_pose,covariance=cov_pose)
        odom_twistc  = TwistWithCovariance(twist=odom_twist,covariance=cov_twist)
        
        local_odom = Odometry(header = ros_header,
                              child_frame_id = 'rear_axle_middle_ground',
                              pose = odom_posec,
                              twist= odom_twistc)

        local_odom_arr.append([ros_time, local_odom])

    return local_odom_arr      

def write_merged_bag(ego_bag_file, target_bag, ego_namespace, obj_namespace, ego_odom, obj_odom, start_time, end_time):
    ego_sensors=[]
    conn_map = {}

    annotation_odom = []

    for odom1 in tqdm(ego_odom):
        for odom2 in obj_odom:
            if odom1[2] == odom2[2]:
                annotation_odom.append([odom1[0],odom2[1]])
                continue

    with Reader(ego_bag_file) as reader, Writer(target_bag) as writer:
        for connection in reader.connections:
            if (connection.topic == '/tf'):
                continue
            ego_sensors.append(connection.topic)
            conn_map[connection.id] = writer.add_connection('{}'.format(connection.topic), connection.msgtype, 'cdr', '')

        connections = [x for x in reader.connections if x.topic in ego_sensors]
        for connection, timestamp, rawdata in tqdm(reader.messages(connections=connections)):
            if ((timestamp*1e-9 > start_time) and timestamp*1e-9 < end_time):
                writer.write(conn_map[connection.id], timestamp, rawdata)
        
            
        # add new connection
        topic = '/{}/local_odometry'.format(ego_namespace)
        msgtype = Odometry.__msgtype__
        ego_connection = writer.add_connection(topic, msgtype, 'cdr', '')

        topic = '/{}/local_odometry'.format(obj_namespace)
        obj_connection = writer.add_connection(topic, msgtype, 'cdr', '')
        
        for odom in ego_odom:
            timestamp = odom[0]
            message = odom[1]
            writer.write(ego_connection, timestamp, serialize_cdr(message,msgtype))
        for odom in annotation_odom:
            timestamp = odom[0]
            message = odom[1]
            writer.write(obj_connection, timestamp, serialize_cdr(message,msgtype))

def write_solo_bag(ego_bag_file, target_bag, ego_namespace, ego_odom, start_time, end_time):
    ego_sensors=[]
    conn_map = {}

    with Reader(ego_bag_file) as reader, Writer(target_bag) as writer:
        for connection in reader.connections:
            if (connection.topic == '/tf'):
                continue
            ego_sensors.append(connection.topic)
            conn_map[connection.id] = writer.add_connection('{}'.format(connection.topic), connection.msgtype, 'cdr', '')

        connections = [x for x in reader.connections if x.topic in ego_sensors]
        for connection, timestamp, rawdata in tqdm(reader.messages(connections=connections)):
            if ((timestamp*1e-9 > start_time) and timestamp*1e-9 < end_time):
                writer.write(conn_map[connection.id], timestamp, rawdata)
        
            
        # add new connection
        topic = '/{}/local_odometry'.format(ego_namespace)
        msgtype = Odometry.__msgtype__
        ego_connection = writer.add_connection(topic, msgtype, 'cdr', '')
        
        for odom in ego_odom:
            timestamp = odom[0]
            message = odom[1]
            writer.write(ego_connection, timestamp, serialize_cdr(message,msgtype))