#include "odom_to_tf.hpp"
#include <rclcpp/qos.hpp>
#include <functional>
#include <exception>
#include <iostream>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

OdomToTFNode::OdomToTFNode() : rclcpp::Node("odom_to_tf_node")
{
    const auto history_size{declare_parameter("qos_history", 1)};
    const auto qos = rclcpp::QoS{static_cast<std::size_t>(history_size)};
    m_parent_frame = declare_parameter<std::string>("parent_frame", "map");
    m_ego_frame = declare_parameter<std::string>("ego_frame", "rear_axle_middle_ground");
    m_opp_frame = declare_parameter<std::string>("opp_frame", "opp_rear_axle_middle_ground");

    m_odom_ego_subscription = create_subscription<nav_msgs::msg::Odometry>(
        "odom_ego", qos, std::bind(&OdomToTFNode::odom_ego_callback, this, std::placeholders::_1));
    m_odom_opp_subscription = create_subscription<nav_msgs::msg::Odometry>(
        "odom_opp", qos, std::bind(&OdomToTFNode::odom_opp_callback, this, std::placeholders::_1));
    m_tf_broadcaster.reset(new tf2_ros::TransformBroadcaster(this));
    m_tfbuffer.reset(new tf2_ros::Buffer(get_clock()));
    m_tflistener.reset(new tf2_ros::TransformListener(*m_tfbuffer));
}

void OdomToTFNode::odom_ego_callback(const nav_msgs::msg::Odometry::SharedPtr odom_in)
{
    Eigen::Isometry3d map_to_ego_eigen;
    tf2::fromMsg(odom_in->pose.pose, map_to_ego_eigen);
    geometry_msgs::msg::TransformStamped tfout = tf2::eigenToTransform(map_to_ego_eigen);
    tfout.header = odom_in->header;
    tfout.child_frame_id = m_ego_frame;
    m_tf_broadcaster->sendTransform(tfout);
}
void OdomToTFNode::odom_opp_callback(const nav_msgs::msg::Odometry::SharedPtr odom_in)
{
    Eigen::Isometry3d map_to_opp_eigen;
    tf2::fromMsg(odom_in->pose.pose, map_to_opp_eigen);
    geometry_msgs::msg::TransformStamped tfout = tf2::eigenToTransform(map_to_opp_eigen);
    tfout.header = odom_in->header;
    tfout.child_frame_id = m_opp_frame;
    m_tf_broadcaster->sendTransform(tfout);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTFNode>());
  rclcpp::shutdown();
  return 0;
}
