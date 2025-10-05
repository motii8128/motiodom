#include "motiodom/motiodom.hpp"

namespace motiodom
{
    MotiOdom::MotiOdom(): Node("motiodom")
    {
        const int max_iter = this->declare_parameter("icp.max_iter", 50);
        const float tolerance_trans = this->declare_parameter("icp.translation_tolerance", 1e-4f);
        const float tolerance_rot = this->declare_parameter("icp.rotation_tolerance", 1e-4f);
        const float max_corr_dist = this->declare_parameter("icp.max_corr_distance", 1.0f);

        frame_id_ = this->declare_parameter("frame_id", "map");
        child_frame_id_ = this->declare_parameter("child_frame_id", "odom");

        RCLCPP_INFO(this->get_logger(), "ICP Parameters\nmax iterations:%d\ntranslation_tolerance:%lf\nrotation_tolerance:%lf\nmax_corr_dist:%lf", 
            max_iter,
            tolerance_trans,
            tolerance_rot,
            max_corr_dist);

        icp_ = std::make_shared<ICP2D>(max_iter, tolerance_trans, tolerance_rot, max_corr_dist);

        has_prev_scan_ = false;
        rotation_ = Mat2::Identity();
        translation_ = Point2f::Zero();

        prev_cloud_ = PointCloud2f();

        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/map_cloud", rclcpp::SystemDefaultsQoS());

        using std::placeholders::_1;
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            0,
            std::bind(&MotiOdom::scan_callback, this, _1)
        );

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "Start MotiOdom Node");
    }

    void MotiOdom::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto current_cloud = scan_msg2eigen_points(*msg);

        if(!has_prev_scan_)
        {
            prev_cloud_ = current_cloud;
            has_prev_scan_ = true;
            return;
        }

        auto result = icp_->align(prev_cloud_, current_cloud, rotation_, translation_);

        auto map_msg = eigen2cloud_msg(prev_cloud_);
        map_msg.header.frame_id = frame_id_;
        map_msg.header.stamp = this->get_clock()->now();
        pointcloud_publisher_->publish(map_msg);

        RCLCPP_INFO(this->get_logger(), "ICP-Result  has_covered:%d, iter:%d", result.has_covered, result.iter);

        geometry_msgs::msg::TransformStamped t;

        t.header.frame_id = frame_id_;
        t.header.stamp = this->get_clock()->now();
        t.child_frame_id = child_frame_id_;

        t.transform.translation.x = translation_.x();
        t.transform.translation.y = translation_.y();
        t.transform.translation.z = 0.0;

        const auto yaw = std::atan2(rotation_(1,0), rotation_(0,0));
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = std::sin(yaw*0.5);
        t.transform.rotation.w = std::cos(yaw*0.5);

        tf_broadcaster_->sendTransform(t);
    }
    
    PointCloud2f scan_msg2eigen_points(const sensor_msgs::msg::LaserScan &scan)
    {
        PointCloud2f points;
        points.reserve(scan.ranges.size());

        float angle = scan.angle_min;

        for(const auto& range: scan.ranges)
        {
            if(std::isfinite(range) && range >= scan.range_min && range <= scan.range_max)
            {
                const auto x = range * cos(angle);
                const auto y = range * sin(angle);
                points.emplace_back(x, y);
            }
            angle += scan.angle_increment;
        }

        return points;
    }

    sensor_msgs::msg::PointCloud eigen2cloud_msg(const PointCloud2f& eigen_points)
    {
        auto ros_msg = sensor_msgs::msg::PointCloud();

        for(const auto& eigen_point: eigen_points)
        {
            auto ros_point = geometry_msgs::msg::Point32();

            ros_point.x = eigen_point.x();
            ros_point.y = eigen_point.y();
            ros_point.z = 0.0;

            ros_msg.points.push_back(ros_point);
        }

        return ros_msg;
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<motiodom::MotiOdom>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}