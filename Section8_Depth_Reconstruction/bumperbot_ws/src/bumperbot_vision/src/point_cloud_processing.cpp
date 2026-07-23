#include "bumperbot_vision/point_cloud_processing.hpp"

#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"

#include "pcl_conversions/pcl_conversions.h"

namespace bumperbot_vision
{
PointCloudProcessing::PointCloudProcessing(const std::string & name)
: rclcpp::Node(name)
{
}

void PointCloudProcessing::initialize()
{
    pct_ = std::make_shared<point_cloud_transport::PointCloudTransport>(shared_from_this());
    cloud_sub_ = pct_->subscribe(
        "camera/points", 10,
        std::bind(&PointCloudProcessing::cloudCallback, this, std::placeholders::_1));
    filtered_cloud_pub_ = pct_->advertise("filtered_cloud", 10);
}

void PointCloudProcessing::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg)
{
    // Convert ROS PointCloud2 message to PCL format
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl::fromROSMsg(*cloud_msg, pcl_cloud);

    // Filter out points that are too far away
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(pcl_cloud));
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_ptr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.1, 5.0);
    pass.filter(pcl_cloud);

    // Downsample the pointcloud
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    float leaf = static_cast<float>(0.05);
    voxel_filter.setLeafSize(leaf, leaf, leaf);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(pcl_cloud));
    pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
    voxel_filter.setInputCloud(filtered_ptr);
    voxel_filter.filter(filtered_cloud);

    // Convert the filtered PCL cloud back to a ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = cloud_msg->header;
    filtered_cloud_pub_.publish(filtered_cloud_msg);
}
}  // namespace bumperbot_vision


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bumperbot_vision::PointCloudProcessing>("point_cloud_processing");
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}