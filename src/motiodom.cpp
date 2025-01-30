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

        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "/pointcloud",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&MotiOdom::lidar_callback, this, _1)
        );

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQoS());

        occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::SystemDefaultsQoS());


        RCLCPP_INFO(this->get_logger(), "Get Parameters");
        this->declare_parameter("enable_reverse", false);
        this->get_parameter("enable_reverse", enable_reverse_);
        this->declare_parameter("ndt_max_iter", 30);
        this->get_parameter("ndt_max_iter", max_iter_);
        this->declare_parameter("ndt_eps", 0.0001);
        this->get_parameter("ndt_eps", eps_);
        this->declare_parameter("ndt_leaf_size", 0.1);
        this->get_parameter("ndt_leaf_size", voxel_grid_leafsize_);
        this->declare_parameter("ndt_step_size", 0.1);
        this->get_parameter("ndt_step_size", eps_);
        this->declare_parameter("ndt_resolution", 0.1);
        this->get_parameter("ndt_resolution", resolution_);

        imu_posture_ = Quat(1.0, 0.0, 0.0, 0.0);
        initialized_ndt_ = false;


        RCLCPP_INFO(this->get_logger(), "Initialize IMU EKF.");
        imu_ekf_ = std::make_shared<ImuPostureEKF>();

        RCLCPP_INFO(this->get_logger(), "Initialize NDT");
        ndt_ = std::make_shared<NDT>(voxel_grid_leafsize_, eps_, step_size_, resolution_, max_iter_);


        RCLCPP_INFO(this->get_logger(), "Start MotiOdom");
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

    void MotiOdom::lidar_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
    {
        auto odom = nav_msgs::msg::Odometry();
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";
        odom.pose.pose.orientation.w = imu_posture_.w();
        odom.pose.pose.orientation.x = imu_posture_.x();
        odom.pose.pose.orientation.y = imu_posture_.y();
        odom.pose.pose.orientation.z = imu_posture_.z();
           
        if(!initialized_ndt_)
        {
            ndt_->initRegistraion(msg);

            initialized_ndt_ = true;
        }
        else
        {
            ndt_->compute(msg, imu_posture_);
            const auto pose = ndt_->getTranslation();

            odom.pose.pose.position.x = pose.x();
            odom.pose.pose.position.y = pose.y();
            odom.pose.pose.position.z = pose.z();

            odom_publisher_->publish(odom);
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motiodom::MotiOdom)