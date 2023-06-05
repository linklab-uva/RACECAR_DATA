#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>

class ConvertImuMsg : public rclcpp::Node
{
    public:
        explicit ConvertImuMsg();

    private:
        rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr m_rawimu_subscription{};        
        sensor_msgs::msg::Imu m_curr_imu;


        /// Input message subscriptions
        void rawimu_callback( const novatel_oem7_msgs::msg::RAWIMU::SharedPtr rawimu_in);

        /// Output message publisher.
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_publisher{};
        
        std::string m_frame_id;
        std::string gps_frame;


};