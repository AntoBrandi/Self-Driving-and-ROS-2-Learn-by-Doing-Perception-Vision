#include "bumperbot_vision/undistort_image.hpp"

#include "cv_bridge/cv_bridge.hpp"

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

namespace bumperbot_vision
{
UndistortImage::UndistortImage(const std::string & name) : Node(name)
{
  const auto sensor_data_qos = rclcpp::SensorDataQoS();

  // Declare parameters
  camera_name_ = declare_parameter("camera_name", "camera");
  get_parameter("camera_name", camera_name_);

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    camera_name_ + "/image_raw", sensor_data_qos, std::bind(&UndistortImage::imageCallback, this, _1));
  info_sub_= create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_name_ + "/camera_info", sensor_data_qos, std::bind(&UndistortImage::imageInfoCallback, this, _1));
}

void UndistortImage::imageInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*info_msg);
}

void UndistortImage::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  if(!camera_info_){
    return;
  }
  cv::Mat image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
  cv::Mat image_undistort;

  model_.fromCameraInfo(camera_info_);
  model_.rectifyImage(image, image_undistort, 1);

  auto undistort_msg =
    cv_bridge::CvImage(image_msg->header, image_msg->encoding, image_undistort).toImageMsg();
  undistort_msg->header = image_msg->header;

  if(!it_){
    const auto sensor_data_qos = rclcpp::SensorDataQoS();
    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
    undistort_image_pub_ = it_->advertise(camera_name_ + "/image_rect", sensor_data_qos.get_rmw_qos_profile());
  }
  undistort_image_pub_.publish(*undistort_msg);
}
}  // namespace bumperbot_vision

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bumperbot_vision::UndistortImage>("undistort_image"));
  rclcpp::shutdown();
  return 0;
}