#ifndef BALL_FOLLOWER_HPP
#define BALL_FOLLOWER_HPP

#include <memory>
#include <string>
#include <vector>

#include "image_transport/image_transport.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::placeholders;

namespace bumperbot_navigation
{
class BallFollower : public rclcpp::Node
{
public:
  BallFollower(const std::string & name);

private:
  std::unique_ptr<image_transport::ImageTransport> it_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  image_transport::Publisher detection_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ball_pose_pub_;

  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);

  void imageInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

  std::string camera_name_;
  std::vector<int64_t> lower_hsv_;
  std::vector<int64_t> upper_hsv_;
  double ball_radius_;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
};
}  // namespace bumperbot_navigation

#endif  // BALL_FOLLOWER_HPP