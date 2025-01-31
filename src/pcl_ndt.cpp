#include "motiodom/pcl_ndt.hpp"

namespace motiodom
{
    NDT::NDT(float voxel_grid_leaf_size, float eps, float step_size, float resolution, int max_iter)
    :voxel_grid_leafsize_(voxel_grid_leaf_size),eps_(eps),step_size_(step_size),resolution_(resolution),max_iter_(max_iter)
    {
        map_pointcloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        translated_pointcloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        ndt_ = pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr(new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>);
        ndt_->setTransformationEpsilon(eps_);
        ndt_->setStepSize(step_size_);
        ndt_->setResolution(resolution_);
        ndt_->setMaximumIterations(max_iter_);
        last_pose_ = Vec3(0.0, 0.0, 0.0);
    }

    void NDT::initRegistraion(const sensor_msgs::msg::PointCloud::SharedPtr ros_cloud)
    {
        map_pointcloud_ = ros2pcl(ros_cloud);
    }

    void NDT::compute(const sensor_msgs::msg::PointCloud::SharedPtr ros_cloud, const Quat &posture)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
        new_pointcloud = ros2pcl(ros_cloud);

        if(map_pointcloud_->empty() || new_pointcloud->empty()) return;

        new_pointcloud = dowmSampling(new_pointcloud, voxel_grid_leafsize_);

        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
        const Mat3x3 rotation_mat = posture.toRotationMatrix();
        init_guess.block<3, 3>(0, 0) = rotation_mat;
        init_guess.block<3, 1>(0, 3) = last_pose_;

        ndt_->setInputSource(new_pointcloud);
        ndt_->setInputTarget(map_pointcloud_);

        translated_pointcloud_->clear();

        ndt_->align(*translated_pointcloud_, init_guess);

        const Eigen::Matrix4f T = ndt_->getFinalTransformation();
        last_pose_(0) = T(0, 3);
        last_pose_(1) = T(1, 3);
        last_pose_(2) = T(2, 3);

        *map_pointcloud_ += *translated_pointcloud_;
        map_pointcloud_ = dowmSampling(map_pointcloud_, 0.1);
    }

    Vec3 NDT::getTranslation()
    {
        return last_pose_;
    }

    sensor_msgs::msg::PointCloud2 NDT::getMapPointCloud()
    {
        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(*map_pointcloud_, ros_cloud);
        ros_cloud.header.frame_id = "map";

        return ros_cloud;
    }
}