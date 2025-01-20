#include "motiodom/motiodom.hpp"

namespace motiodom
{
    MotiOdom::MotiOdom(const rclcpp::NodeOptions& option) : Node("MotiOdom", option)
    {
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            qos_settings,
            std::bind(&MotiOdom::imu_callback, this, _1)
        );

        timer_ = this->create_wall_timer(100ms, std::bind(&MotiOdom::timer_callback, this));

        ydlidar_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/lidar_scan", rclcpp::SystemDefaultsQoS());

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQoS());

        occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::SystemDefaultsQoS());

        imu_ekf_ = std::make_shared<ImuPostureEKF>();
        ydlidar_ = std::make_shared<YDLidarDriver>(230400);
        if(!ydlidar_->startLidar())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize YD Lidar.");
            RCLCPP_ERROR(this->get_logger(), "LIDAR_ERROR : %s", ydlidar_->getError());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Initialize YD Lidar. pitch angle : %lf", ydlidar_->getPitchAngle());
        }
    }

    void MotiOdom::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto current_time = this->now();

        if(prev_imu_callback_time.nanoseconds() > 0)
        {
            rclcpp::Duration delta_time = current_time - prev_imu_callback_time;
            auto dt = delta_time.seconds();

            const auto angular_velocity = Vec3(
                degree2radian(msg->angular_velocity.x),
                degree2radian(msg->angular_velocity.y),
                degree2radian(msg->angular_velocity.z)
            );

            const auto linear_accel =Vec3(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z
            );

            const auto posture = imu_ekf_->estimate(angular_velocity, linear_accel, dt);

            imu_posture_ = euler2quat(posture);
        }

        prev_imu_callback_time = current_time;
    }

    void MotiOdom::timer_callback()
    {
        if(ydlidar_->Scan())
        {
            auto odom = nav_msgs::msg::Odometry();
            odom.header.frame_id = "map";
            odom.child_frame_id = "base_link";
            odom.pose.pose.orientation.w = imu_posture_.w();
            odom.pose.pose.orientation.x = imu_posture_.x();
            odom.pose.pose.orientation.y = imu_posture_.y();
            odom.pose.pose.orientation.z = imu_posture_.z();
            const auto scanPoints = ydlidar_->getScanPoints();

            odom_publisher_->publish(odom);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to Scan");
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motiodom::MotiOdom)