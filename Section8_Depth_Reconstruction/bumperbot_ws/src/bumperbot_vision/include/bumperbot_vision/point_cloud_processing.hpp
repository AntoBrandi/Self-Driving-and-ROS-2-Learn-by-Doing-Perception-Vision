#ifndef POINT_CLOUD_PROCESSING_HPP
#define POINT_CLOUD_PROCESSING_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "point_cloud_transport/point_cloud_transport.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace bumperbot_vision
{
class PointCloudProcessing : public rclcpp::Node
{
public:
    PointCloudProcessing(const std::string & name);

    void initialize();

private:
    std::shared_ptr<point_cloud_transport::PointCloudTransport> pct_;
    point_cloud_transport::Subscriber cloud_sub_;
    point_cloud_transport::Publisher filtered_cloud_pub_;

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);
};
}

#endif  // POINT_CLOUD_PROCESSING_HPP