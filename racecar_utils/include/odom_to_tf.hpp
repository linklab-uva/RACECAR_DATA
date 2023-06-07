#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>


class OdomToTFNode : public rclcpp::Node
{
    public:
        explicit OdomToTFNode();

    private:
        void odom_ego_callback(const nav_msgs::msg::Odometry::SharedPtr odom_in);
        void odom_opp_callback(const nav_msgs::msg::Odometry::SharedPtr odom_in);

        /// Input message subscription
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_ego_subscription;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_opp_subscription;
        /// Output TF Broadcaster.
        std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
        std::shared_ptr<tf2_ros::Buffer> m_tfbuffer;
        std::shared_ptr<tf2_ros::TransformListener> m_tflistener;

        std::string m_parent_frame;
        std::string m_ego_frame;
        std::string m_opp_frame;
        bool m_tf_out;

        geometry_msgs::msg::TransformStamped m_child_to_base_link_msg;
        Eigen::Isometry3d m_child_to_base_link_eigen;
        Eigen::Vector3d m_child_to_bl_position;

        int m_cache_size;


};