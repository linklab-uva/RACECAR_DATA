#!/usr/bin/env python
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

def convert_ros_msg_to_cv2(_cv_bridge : CvBridge, ros_data, image_encoding='bgr8'):
    """
    Convert from a ROS Image message to a cv2 image.
    """
    try:
        return _cv_bridge.imgmsg_to_cv2(ros_data, image_encoding)
    except CvBridgeError as e:
        if "[16UC1] is not a color format" in str(e):
            raise CvBridgeError(
                "You may be trying to use a Image method " +
                "(Subscriber, Publisher, conversion) on a depth image" +
                " message. Original exception: " + str(e))
        raise e

def convert_ros_compressed_to_cv2(_cv_bridge : CvBridge, compressed_msg):
    np_arr = np.fromstring(compressed_msg.data, np.uint8)
    return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def convert_ros_compressed_msg_to_ros_msg(_cv_bridge, compressed_msg,
                                            encoding='bgr8'):
    cv2_img = convert_ros_compressed_to_cv2(compressed_msg)
    ros_img = _cv_bridge.cv2_to_imgmsg(cv2_img, encoding=encoding)
    ros_img.header = compressed_msg.header
    return ros_img

def convert_cv2_to_ros_msg(_cv_bridge : CvBridge, cv2_data, image_encoding='bgr8'):
    """
    Convert from a cv2 image to a ROS Image message.
    """
    return _cv_bridge.cv2_to_imgmsg(cv2_data, image_encoding)

def convert_cv2_to_ros_compressed_msg(_cv_bridge, cv2_data,
                                        compressed_format='jpg'):
    """
    Convert from cv2 image to ROS CompressedImage.
    """
    return _cv_bridge.cv2_to_compressed_imgmsg(cv2_data,
                                                    dst_format=compressed_format)

def convert_ros_msg_to_ros_compressed_msg(_cv_bridge : CvBridge, image,
                                            image_encoding='bgr8',
                                            compressed_format="jpg"):
    """
    Convert from ROS Image message to ROS CompressedImage.
    """
    cv2_img = convert_ros_msg_to_cv2(image, image_encoding)
    cimg_msg = _cv_bridge.cv2_to_compressed_imgmsg(cv2_img,
                                                        dst_format=compressed_format)
    cimg_msg.header = image.header
    return cimg_msg
