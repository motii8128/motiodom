#include "motiodom/types.hpp"

namespace motiodom
{
    float degree2radian(const float& deg)
    {
        return deg * (M_PI / 180.0);
    }

    Quat euler2quat(const Vec3& euler)
    {
        tf2::Quaternion tfq;
        tfq.setRPY(euler.x(), euler.y(), euler.z());

        return Quat(tfq.w(), tfq.x(), tfq.y(), tfq.z());
    }

    sensor_msgs::msg::PointCloud toROSMsg(const PointCloud2d input)
    {
        sensor_msgs::msg::PointCloud ros_msg;
        ros_msg.header.frame_id = "map";

        for(const auto& p : input)
        {
            auto ros_p =  geometry_msgs::msg::Point32();
            ros_p.x = p.x();
            ros_p.y = p.y();

            ros_msg.points.push_back(ros_p);
        }

        return ros_msg;
    }
}