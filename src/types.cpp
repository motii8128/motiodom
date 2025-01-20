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
}