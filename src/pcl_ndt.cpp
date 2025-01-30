#include "motiodom/pcl_ndt.hpp"

namespace motiodom
{
    NDT::NDT(float voxel_grid_leaf_size, float eps, float step_size, float resolution, int max_iter)
    :voxel_grid_leafsize_(voxel_grid_leaf_size),eps_(eps),step_size_(step_size),resolution_(resolution),max_iter_(max_iter)
    {
        map_pointcloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        translated_pointcloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        last_pose_ = Vec3(0.0, 0.0, 0.0);
    }

    void NDT::initRegistraion(const sensor_msgs::msg::PointCloud::SharedPtr ros_cloud)
    {
        ros2pcl(ros_cloud, map_pointcloud_);
    }

    void NDT::compute(const sensor_msgs::msg::PointCloud::SharedPtr ros_cloud, const Quat &posture)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
        ros2pcl(ros_cloud, new_pointcloud);

        if(map_pointcloud_->empty() || new_pointcloud->empty()) return;

        dowmSampling(translated_pointcloud_);

        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
        ndt.setTransformationEpsilon(eps_);
        ndt.setStepSize(step_size_);
        ndt.setResolution(resolution_);
        ndt.setMaximumIterations(max_iter_);

        const Eigen::Matrix4f init_guess = (last_pose_ * posture).matrix();

        ndt.setInputSource(new_pointcloud);
        ndt.setInputTarget(map_pointcloud_);

        ndt.align(*translated_pointcloud_, init_guess);

        const Eigen::Matrix4f T = ndt.getFinalTransformation();
        last_pose_(0) = T(0, 3);
        last_pose_(1) = T(1, 3);
        last_pose_(2) = T(2, 3);

        *map_pointcloud_ += *translated_pointcloud_;
    }

    Vec3 NDT::getTranslation()
    {
        return last_pose_;
    }
}