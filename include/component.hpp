#ifndef COMPONENT_HPP_
#define COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>


namespace motiodom
{
    class MotiiPostureEstimater : public rclcpp::Node
    {
        public:
        explicit MotiiPostureEstimater(const rclcpp::NodeOptions & node_options);

        private:
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr magnetic_field_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
    };
}

#endif