#include "motiodom_node.hpp"

namespace motiodom
{
    MotiOdom::MotiOdom(const rclcpp::NodeOptions & node_options): rclcpp::Node("motiodom_node", node_options)
    {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            0,
            std::bind(&MotiOdom::imu_callback, this, _1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        this->declare_parameter("enable_magnet", false);
        this->get_parameter("enable_magnet", enable_magnet_);

        this->declare_parameter("enable_position", false);
        this->get_parameter("enable_position", enable_position_);

        this->declare_parameter("frame_id", "odom");
        this->get_parameter("frame_id", frame_id_);

        this->declare_parameter("child_frame_id", "imu");
        this->get_parameter("child_frame_id", child_id_);

        this->declare_parameter("delta_time", 10);
        this->get_parameter("delta_time", delta_time_);

        imu_flag_ = false;
        delta_float_ = (float)(delta_time_)/1000;

        if(enable_magnet_)
        {
            magnetic_field_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
                "/magnet",
                0,
                std::bind(&MotiOdom::magnet_callback, this, _1));
            mag_flag_ = false;

            timer_ = this->create_wall_timer(std::chrono::milliseconds(delta_time_), std::bind(&MotiOdom::axis9_callback, this));
        }
        else
        {
            timer_ = this->create_wall_timer(std::chrono::milliseconds(delta_time_), std::bind(&MotiOdom::axis6_callback, this));
        }

        ekf6_ = std::shared_ptr<motiodom::AccelAngularEKF<float>>(motiodom::AccelAngularEKF<float>::initialize(delta_float_));

        RCLCPP_INFO(this->get_logger(), "Start MotiOdom delta_time:%d", delta_time_);
    }

    void MotiOdom::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_flag_ = true;
        get_imu_ = msg;
    }

    void MotiOdom::magnet_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        mag_flag_ = true;
        get_magnet_ = msg;
    }

    void MotiOdom::axis6_callback()
    {
        if(imu_flag_)
        {
            
            auto linear_accel = Vector3<float>(
                get_imu_->linear_acceleration.x,
                get_imu_->linear_acceleration.y,
                get_imu_->linear_acceleration.z);

            auto angular_velocity = Vector3<float>(
                to_radian<float>(get_imu_->angular_velocity.x),
                to_radian<float>(get_imu_->angular_velocity.y),
                to_radian<float>(get_imu_->angular_velocity.z));

            auto estimation = ekf6_->run(angular_velocity, linear_accel, delta_float_);

            geometry_msgs::msg::TransformStamped t;

            t.header.frame_id = frame_id_;
            t.header.stamp = this->get_clock()->now();
            t.child_frame_id = child_id_;

            tf2::Quaternion q;
            q.setRPY(estimation.x, estimation.y, estimation.z);
            t.transform.rotation.w = q.w();
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();

            tf_broadcaster_->sendTransform(t);
        }
    }

    void MotiOdom::axis9_callback()
    {

    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motiodom::MotiOdom)