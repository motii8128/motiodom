#ifndef MOTIODOM_HPP_
#define MOTIODOM_HPP_

#include "common.hpp"
#include "iterative_closest_point.hpp"
#include "extended_kalman_filter.hpp"
#include "unscented_kalman_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <atomic>


namespace motiodom
{
    class MotiOdom : public rclcpp::Node
    {
        public:
        explicit MotiOdom();

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        private:
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pointcloud_publisher_;

        rclcpp::Time last_imu_time_;

        std::string frame_id_, child_frame_id_;

        PointCloud2f prev_cloud_;
        std::shared_ptr<ICP2D> icp_;
        bool has_prev_scan_;
        bool imu_received_;
        bool is_degree_imu_;

        float near_lidar_threshold_;

        std::shared_ptr<ImuPostureEKF> imu_posture_estimater_;

        Mat2 rotation_;
        Vec2 translation_;

        std::atomic<float> ekf_estimation_yaw_;

        rclcpp::CallbackGroup::SharedPtr callback_group_;
    };

    PointCloud2f scan_msg2eigen_points(const sensor_msgs::msg::LaserScan &scan, const geometry_msgs::msg::TransformStamped &tf_lidar_to_base, const float near_lidar_threshold);

    sensor_msgs::msg::PointCloud eigen2cloud_msg(const PointCloud2f& eigen_points);
}

#endif