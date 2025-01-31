#ifndef MOTISLAM_HPP_
#define MOTISLAM_HPP_

#include "types.hpp"
#include "posture_ekf.hpp"
#include "ydlidar.hpp"
#include "icp.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace motiodom
{
    class MotiOdom : public rclcpp::Node
    {
        public:
        explicit MotiOdom(const rclcpp::NodeOptions& option=rclcpp::NodeOptions());

        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr ydlidar_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        rclcpp::Time prev_imu_callback_time;
        std::shared_ptr<ImuPostureEKF> imu_ekf_;
        std::shared_ptr<YDLidarDriver> ydlidar_;
        std::shared_ptr<ICP> icp_;
        Quat imu_posture_;
        Vec3 imu_posture_euler_;
        bool set_source_;

        // パラメーター
        bool enable_reverse_;
        int icp_max_iter_;
        float icp_threshold_;

    };
}

#endif