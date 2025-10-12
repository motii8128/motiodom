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
        lidar_frame_id_ = this->declare_parameter("lidar_frame_id", "lidar");

        near_lidar_threshold_ = this->declare_parameter("near_lidar_threshold", 0.01);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "ICP Parameters\nmax iterations:%d\ntranslation_tolerance:%lf\nrotation_tolerance:%lf\nmax_corr_dist:%lf", 
            max_iter,
            tolerance_trans,
            tolerance_rot,
            max_corr_dist);

        icp_ = std::make_shared<ICP2D>(max_iter, tolerance_trans, tolerance_rot, max_corr_dist);

        imu_posture_estimater_ = std::make_shared<ImuPostureEKF>();

        has_prev_scan_ = false;
        rotation_ = Mat2::Identity();
        translation_ = Vec2::Zero();

        prev_cloud_ = PointCloud2f();

        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/map_cloud", rclcpp::SystemDefaultsQoS());

        using std::placeholders::_1;
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&MotiOdom::scan_callback, this, _1)
        );

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sensing/realsense/imu",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&MotiOdom::imu_callback, this, _1)
        );

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "Start MotiOdom Node");
        last_imu_time_ = this->get_clock()->now();
    }

    void MotiOdom::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto t = geometry_msgs::msg::TransformStamped();
        try
        {
            t = tf_buffer_->lookupTransform(child_frame_id_, lidar_frame_id_ ,tf2::TimePointZero);

            auto current_cloud = scan_msg2eigen_points(*msg, t, near_lidar_threshold_);

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
        catch(tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s->%s: %s",child_frame_id_.c_str(), lidar_frame_id_.c_str(), ex.what());
        }
        
        
    }

    void MotiOdom::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const auto current_time = this->get_clock()->now();
        const auto duration = current_time - last_imu_time_;

        const auto dt = duration.seconds();

        Vec3 linear_accel(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        );

        Vec3 angular_velocity(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        );

        const auto est = imu_posture_estimater_->estimate(angular_velocity, linear_accel, dt);

        rotation_ = Eigen::Rotation2Df(est.z() / 2.0).toRotationMatrix();

        RCLCPP_INFO(this->get_logger(), "x:%lf,y:%lf,z:%lf", est.x(),est.y(),est.z()/2.0);

        last_imu_time_ = current_time;
    }
    
    PointCloud2f scan_msg2eigen_points(const sensor_msgs::msg::LaserScan &scan, const geometry_msgs::msg::TransformStamped &tf_lidar_to_base, const float near_lidar_threshold)
    {
        Eigen::Affine2f T = Eigen::Affine2f::Identity();

        T.translation() << static_cast<float>(tf_lidar_to_base.transform.translation.x),
                           static_cast<float>(tf_lidar_to_base.transform.translation.y);

        const auto &q = tf_lidar_to_base.transform.rotation;
        const float yaw = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        );
        T.linear() = Eigen::Rotation2Df(yaw).toRotationMatrix();

        PointCloud2f points;
        points.reserve(scan.ranges.size());

        float angle = scan.angle_min;

        for(const auto& range: scan.ranges)
        {
            if(std::isfinite(range) && range >= scan.range_min && range <= scan.range_max && fabs(range) > near_lidar_threshold)
            {
                Vec2 p_lidar(range * cos(angle), range * sin(angle));

                Vec2 p_base = T * p_lidar;
                points.emplace_back(p_base);
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