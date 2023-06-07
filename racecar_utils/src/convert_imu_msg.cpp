#include "convert_imu_msg.hpp"
#include <rclcpp/qos.hpp>
#include <functional>
#include <exception>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <cmath>

ConvertImuMsg::ConvertImuMsg() : rclcpp::Node("convert_imu_node")
{
    m_frame_id=declare_parameter<std::string>("frame_id", "odom");
    gps_frame=declare_parameter<std::string>("gps_frame", "gps_top");
    const auto qos = rclcpp::SensorDataQoS();


    m_rawimu_subscription = create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
        "imu_in", qos, std::bind(&ConvertImuMsg::rawimu_callback, this, std::placeholders::_1));


    m_imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu_out", qos);
    
}

void ConvertImuMsg::rawimu_callback(const novatel_oem7_msgs::msg::RAWIMU::SharedPtr rawimu_in)
{
    double angvel_stdev = .05;
    double angvel_var = angvel_stdev*angvel_stdev;

    m_curr_imu.angular_velocity.x = rawimu_in->angular_velocity.y;
    m_curr_imu.angular_velocity.y = -rawimu_in->angular_velocity.x;
    m_curr_imu.angular_velocity.z = rawimu_in->angular_velocity.z;
    m_curr_imu.angular_velocity_covariance[0] = angvel_var;
    m_curr_imu.angular_velocity_covariance[4] = 0.00001;
    m_curr_imu.angular_velocity_covariance[8] = angvel_var;

    m_curr_imu.linear_acceleration.x = rawimu_in->angular_velocity.y;
    m_curr_imu.linear_acceleration.y = -rawimu_in->linear_acceleration.x;
    m_curr_imu.linear_acceleration.z = rawimu_in->linear_acceleration.z;
    m_curr_imu.linear_acceleration_covariance[0] = 2.5*2.5;
    m_curr_imu.linear_acceleration_covariance[4] = 2.5*2.5;
    m_curr_imu.linear_acceleration_covariance[8] = 2.5*2.5;
    m_curr_imu.orientation_covariance[0] = -1.0;
    m_curr_imu.header = rawimu_in->header;
    m_curr_imu.header.frame_id = gps_frame;
    m_imu_publisher->publish(m_curr_imu);   

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConvertImuMsg>());
  rclcpp::shutdown();
  return 0;
}

