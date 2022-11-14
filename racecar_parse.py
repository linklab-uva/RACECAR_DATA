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

def read_bag_file(bag_file, topics):
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

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians


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

    
