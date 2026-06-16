#ifndef UNDISTORT_IMAGE_HPP
#define UNDISTORT_IMAGE_HPP

#include <memory>
#include <string>

#include "image_transport/image_transport.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "image_geometry/pinhole_camera_model.hpp"

using namespace std::placeholders;

namespace bumperbot_vision
{
class UndistortImage : public rclcpp::Node
{
public:
  UndistortImage(const std::string & name);

private:
  std::unique_ptr<image_transport::ImageTransport> it_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  image_transport::Publisher undistort_image_pub_;

  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);

  void imageInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

  image_geometry::PinholeCameraModel model_;
  std::string camera_name_;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
};
}  // namespace bumperbot_vision

#endif  // UNDISTORT_IMAGE_HPP