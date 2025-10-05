#ifndef MOTIODOM_HPP_
#define MOTIODOM_HPP_

#include "common.hpp"
#include "icp.hpp"
#include "ekf.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace motiodom
{
    class MotiOdom : public rclcpp::Node
    {
        public:
        explicit MotiOdom();

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        private:
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pointcloud_publisher_;

        std::string frame_id_, child_frame_id_;

        PointCloud2f prev_cloud_;
        std::shared_ptr<ICP2D> icp_;
        bool has_prev_scan_;

        Mat2 rotation_;
        Point2f translation_;
    };

    PointCloud2f scan_msg2eigen_points(const sensor_msgs::msg::LaserScan &scan);

    sensor_msgs::msg::PointCloud eigen2cloud_msg(const PointCloud2f& eigen_points);
}

#endif