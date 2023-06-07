import pygeodesy

from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import deserialize_cdr, serialize_cdr
from rosbags.typesys import get_types_from_msg, register_types


from pathlib import Path
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm
from copy import copy, deepcopy

import os
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import pygeodesy
import math
import yaml

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

# Import ROS2 Types
abs_msg_list = absoluteFilePaths('ros2_custom_msgs/novatel_gps_msgs/msg')
add_types = {}

for pathstr in abs_msg_list:
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

abs_msg_list = absoluteFilePaths('ros2_custom_msgs/novatel_oem7_msgs/msg')
for pathstr in abs_msg_list:
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

register_types(add_types)
from rosbags.typesys.types import novatel_gps_msgs__msg__NovatelPosition as NovatelPosition
from rosbags.typesys.types import novatel_gps_msgs__msg__NovatelVelocity as NovatelVelocity
from rosbags.typesys.types import novatel_gps_msgs__msg__NovatelMessageHeader as NovatelMessageHeader
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

class OdomConverter(object):

    def __init__(self, cfg):
        with open(cfg, 'r') as f:
            self.cfg = yaml.load(f, Loader=yaml.Loader)
        


    def read_bag_file(self, bag_file, topics, topic_types, start_time, end_time, check_time = True):
        topic_dict = {}
        for topic in topics:
            topic_dict[topic] = []

        # create reader instance and open for reading
        with Reader(bag_file) as reader:

            # messages() accepts connection filters
            connections = [x for x in reader.connections if x.topic in topics]
            for connection, timestamp, rawdata in tqdm(reader.messages(connections=connections)):
                if check_time and (timestamp*1e-9 < start_time or timestamp*1e-9 > end_time):
                    continue
                if connection.msgtype in topic_types:
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    topic_dict[connection.topic].append((timestamp,msg))

        return topic_dict


    def visualize_position(self, local_odom):
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

        
    def gen_local_odom(self, gps_messages, vel_messages):

        deg2rad = math.pi/180.0

        local_odom_arr = []
        origin = self.cfg["track"]["origin"]
        gpsmap = pygeodesy.LocalCartesian(origin[0], origin[1], origin[2])

        prev_pos = [0.0,0.0,0.0]
        prev_time = gps_messages[0][0]

        for ros_time, bestpos in tqdm(gps_messages[1:]):

            
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
                trk_r  = R.from_euler('z',heading)
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
                trk_r = R.from_euler('z',math.pi/2)*trk_r
            
            quat = (trk_r).as_quat()
            rot_stdev = (math.pi/(2*180))

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
    
    def gen_local_odom_euro(self, gps_messages, vel_messages):

        deg2rad = math.pi/180.0

        local_odom_arr = []
        origin = self.cfg["track"]["origin"]
        gpsmap = pygeodesy.LocalCartesian(origin[0], origin[1], origin[2])

        prev_pos = [0.0,0.0,0.0]
        prev_time = gps_messages[0][0]
        prev_index = 0

        for ros_time, bestpos in tqdm(gps_messages[1:]):

            
            bestvel = None

            ros_time_vel = 0

            # Parse BESTPOS

            ros_header = bestpos.header
            gps_header = bestpos.novatel_msg_header
            gps_time   = gps_header.gps_seconds * 1000
            lat        = bestpos.lat
            lon        = bestpos.lon
            hgt        = bestpos.height
            lat_stdev  = bestpos.lat_sigma  
            lon_stdev  = bestpos.lon_sigma  
            hgt_stdev  = bestpos.height_sigma

            novel = True
            # Find Corresponding BESTVEL
            i = 0
            for time,msg in vel_messages[prev_index:]:
                if msg.novatel_msg_header.gps_seconds * 1000 == gps_time:
                    ros_time_vel = time
                    bestvel = msg
                    novel = False
                    prev_index = i + prev_index
                    break
                if msg.novatel_msg_header.gps_seconds * 1000 > gps_time:
                    break 
                i += 1
            

            # GPS 2 LOCAL CARTESIAN
            local_tuple = gpsmap.forward(lat, lon, hgt)
            if novel:
                heading = math.atan2(local_tuple[1] - prev_pos[1], local_tuple[0] - prev_pos[0])
                hor_speed = (((local_tuple[1] - prev_pos[1])**2+(local_tuple[0] - prev_pos[0])**2)**(0.5))/(ros_time-prev_time)
                trk_r  = R.from_euler('z',heading)

            else:
                ros_header_v = bestvel.header
                gps_header_v = bestvel.novatel_msg_header
                gps_time_v   = gps_header_v.gps_seconds
                trk_gnd      = bestvel.track_ground
                latency      = bestvel.latency
                hor_speed    = bestvel.horizontal_speed
                ver_speed    = bestvel.vertical_speed
                heading      =-trk_gnd*deg2rad
                trk_r  = R.from_euler('z',heading)
                trk_r = R.from_euler('z',math.pi/2)*trk_r

            quat = (trk_r).as_quat()
            rot_stdev = (math.pi/(2*180))

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

    def nav2odom(self, gps_messages):

        deg2rad = math.pi/180.0

        local_odom_arr = []
        origin = self.cfg["track"]["origin"]
        gpsmap = pygeodesy.LocalCartesian(origin[0], origin[1], origin[2])

        prev_pos = [0.0,0.0,0.0]
        prev_time = gps_messages[0][0]

        for ros_time, msg in tqdm(gps_messages[1:]):

            
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

    def write_merged_bag(self, ego_bag_file, target_bag, ego_namespace, obj_namespace, ego_odom, obj_odom, start_time, end_time, lidar_topics):
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
                if (connection.topic in '/tf'):
                    continue
                if ('novatel' in connection.topic) or ('camera' in connection.topic) or ('radar' in connection.topic) or (connection.topic in lidar_topics):
                    ego_sensors.append(connection.topic)
                    conn_map[connection.id] = writer.add_connection('/{}{}'.format(ego_namespace,connection.topic), connection.msgtype, 'cdr', '')

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

    def write_solo_bag(self, ego_bag_file, target_bag, ego_namespace, ego_odom, start_time, end_time, lidar_topics):
        ego_sensors=[]
        conn_map = {}

        with Reader(ego_bag_file) as reader, Writer(target_bag) as writer:
            for connection in reader.connections:
                if (connection.topic == '/tf'):
                    continue
                if ('novatel' in connection.topic) or ('camera' in connection.topic) or ('radar' in connection.topic) or (connection.topic in lidar_topics):
                    ego_sensors.append(connection.topic)
                    if connection.topic[0] != '/':
                        conn_map[connection.id] = writer.add_connection('/{}/{}'.format(ego_namespace,connection.topic), connection.msgtype, 'cdr', '')
                    else:
                        conn_map[connection.id] = writer.add_connection('/{}{}'.format(ego_namespace,connection.topic), connection.msgtype, 'cdr', '')

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

    def process_data(self):
        scenario = self.cfg["scenario"]
        multi = self.cfg["multi"]
        track = self.cfg["track"]["name"]
        start_time = self.cfg["track"]["start_time"]
        end_time = self.cfg["track"]["end_time"]
        ego_team = self.cfg["ego"]["team"]
        ego_num = self.cfg["ego"]["number"]
        ego_source = self.cfg["ego"]["source_file"]
        ego_target = self.cfg["ego"]["target_file"]
        ego_path = os.path.join('../../data/RAW_ROSBAG', ego_team, track, ego_source)
        ego_target_path = os.path.join('../../data/RACECAR', scenario, ego_target)
        ego_topics = self.cfg["ego"]["gps_topics"]
        ego_types = self.cfg["ego"]["topic_types"]
        ego_lidar = self.cfg["ego"]["lidar_topics"]
        ego_write = self.cfg["ego"]["write_to_bag"]

        if os.path.exists(ego_target_path):
            print('ros2 bag at {} already exists'.format(ego_target_path))
            return


        ego_data = self.read_bag_file(ego_path, ego_topics, ego_types, start_time, end_time)

        if self.cfg["ego"]["novatel"]:
            ego_odom = self.gen_local_odom(ego_data[ego_topics[0]], ego_data[ego_topics[1]])
        else:
            ego_odom = self.nav2odom(ego_data[ego_topics[0]])

        
        if multi:
            target_team = self.cfg["target"]["team"]
            target_num = self.cfg["target"]["number"]
            target_source = self.cfg["target"]["source_file"]
            target_target = self.cfg["target"]["target_file"]
            target_path = os.path.join('../../data/RAW_ROSBAG', target_team, track, target_source)
            target_target_path = os.path.join('../../data/RACECAR', scenario, target_target)
            target_topics = self.cfg["target"]["gps_topics"]
            target_types = self.cfg["target"]["topic_types"]
            target_lidar = self.cfg["target"]["lidar_topics"]
            target_write = self.cfg["target"]["write_to_bag"]
            target_check_time = self.cfg["target"]["check_time"]

            target_data = self.read_bag_file(target_path, target_topics, target_types, start_time, end_time, target_check_time)

            if self.cfg["target"]["novatel"]:
                target_odom = self.gen_local_odom(target_data[target_topics[0]], target_data[target_topics[1]])
            else:
                target_odom = self.gen_local_odom_euro(target_data[target_topics[0]], target_data[target_topics[1]])

            if ego_write:
                self.write_merged_bag(ego_bag_file=ego_path, 
                        target_bag = ego_target_path,
                        ego_namespace='vehicle_{}'.format(ego_num),
                        obj_namespace='vehicle_{}'.format(target_num),
                        ego_odom=ego_odom,
                        obj_odom=target_odom,
                        start_time=start_time,
                        end_time=end_time,
                        lidar_topics=ego_lidar)
                
            if target_write:
                self.write_merged_bag(ego_bag_file=target_path, 
                        target_bag = target_target_path,
                        ego_namespace='vehicle_{}'.format(target_num),
                        obj_namespace='vehicle_{}'.format(ego_num),
                        ego_odom=target_odom,
                        obj_odom=ego_odom,
                        start_time=start_time,
                        end_time=end_time,
                        lidar_topics=target_lidar)
        else:
            self.write_solo_bag(ego_bag_file=ego_path, 
                        target_bag = ego_target_path,
                        ego_namespace='vehicle_{}'.format(ego_num),
                        ego_odom=ego_odom,
                        start_time=start_time,
                        end_time=end_time,
                        lidar_topics=ego_lidar)

    def viz_data(self):
            multi = self.cfg["multi"]
            track = self.cfg["track"]["name"]
            start_time = self.cfg["track"]["start_time"]
            end_time = self.cfg["track"]["end_time"]
            ego_team = self.cfg["ego"]["team"]
            ego_num = self.cfg["ego"]["number"]
            ego_source = self.cfg["ego"]["source_file"]
            ego_target = self.cfg["ego"]["target_file"]
            ego_path = os.path.join('../../data/RAW_ROSBAG', ego_team, track, ego_source)
            ego_target_path = os.path.join('../../data/LOCAL_ODOM', ego_team, track, ego_target)
            ego_topics = self.cfg["ego"]["gps_topics"]
            ego_types = self.cfg["ego"]["topic_types"]
            ego_lidar = self.cfg["ego"]["lidar_topics"]
            ego_write = self.cfg["ego"]["write_to_bag"]

            ego_data = self.read_bag_file(ego_path, ego_topics, ego_types)
            cx_pos = []
            ctime  = []
            for odom in ego_data['novatel_btm_id0_gps']:
                cx_pos.append(odom[1].latitude)
                ctime.append(odom[0]*1e-9)

            plt.rcParams['figure.figsize'] = [50,25]

            fig, ax = plt.subplots(1, 1)
            ax.plot(ctime, cx_pos, marker='o',color='r',linestyle='-', markersize = 1)
            # ax.plot(ctime, cy_pos, marker='o',color='r',linestyle='-', markersize = 1)
            # plt.gca().set_aspect('equal', adjustable = 'box')
            plt.title('Time vs Position')
            plt.xlabel('time (s)')
            plt.ylabel('lat (deg)')
            plt.savefig('euro_time.pdf')


if __name__ == '__main__':

    config_folder = str(sys.argv[1])
    for config in os.listdir(config_folder):
        # try:
        print('Processing {}'.format(config))
        config_file = os.path.join(config_folder,config)
        converter = OdomConverter(cfg = config_file)
        converter.process_data()
        # except:
        #     print('{} did not succeed'.format(config))
        # else:
        #     print('{} Finished!'.format(config))
    # converter.viz_data()