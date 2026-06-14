#include "bumperbot_vision/image_processing.hpp"

#include "cv_bridge/cv_bridge.hpp"

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

namespace bumperbot_vision
{
ImageProcessing::ImageProcessing(const std::string & name) : Node(name)
{
  const auto sensor_data_qos = rclcpp::SensorDataQoS();

  // Declare parameters
  camera_name_ = declare_parameter("camera_name", "camera");
  get_parameter("camera_name", camera_name_);

  declare_parameter("lower_hsv", std::vector<int64_t>{79, 50, 50});
  declare_parameter("upper_hsv", std::vector<int64_t>{99, 255, 255});
  get_parameter("lower_hsv", lower_hsv_);
  get_parameter("upper_hsv", upper_hsv_);

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    camera_name_ + "/image_rect", sensor_data_qos, std::bind(&ImageProcessing::imageCallback, this, _1));
  info_sub_= create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_name_ + "/camera_info", sensor_data_qos, std::bind(&ImageProcessing::imageInfoCallback, this, _1));
}

void ImageProcessing::imageInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*info_msg);
}

void ImageProcessing::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  if(!camera_info_){
    return;
  }
  cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
  cv::Mat processed_image = image.clone();

  // ============================================================
  // LAB — Color Spaces
  // ============================================================
 
  // // Convert to Grayscale — single channel, brightness only
  // cv::Mat gray_image;
  // cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  // processed_image = gray_image;
 
  // // Convert to HSV
  // cv::Mat hsv_image;
  // cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
  // processed_image = hsv_image;
 
  // // Split HSV into individual channels and visualize one
  // cv::Mat hsv_image;
  // cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
  // std::vector<cv::Mat> channels;
  // cv::split(hsv_image, channels);
  // // Pick which channel to display: channels[0]=H, channels[1]=S, channels[2]=V
  // processed_image = channels[2];

  // cv::Mat full_saturation(hsv_image.size(), CV_8UC1, cv::Scalar(255));
  // cv::Mat full_value(hsv_image.size(), CV_8UC1, cv::Scalar(255));

  // cv::Mat hue_colorized;
  // std::vector<cv::Mat> hue_channels = {channels[0], full_saturation, full_value};
  // cv::merge(hue_channels, hue_colorized);
  // cv::cvtColor(hue_colorized, processed_image, cv::COLOR_HSV2BGR);
 
  // ============================================================
  // LAB — Blurring
  // ============================================================
 
  // // Gaussian Blur — smooths the image using a weighted average
  // int kernel_size = 7;
  // cv::GaussianBlur(image, processed_image, cv::Size(kernel_size, kernel_size), 0);
 
  // // Median Blur — replaces each pixel with the median of its neighbors
  // // Excellent at removing salt-and-pepper noise while preserving edges
  // int kernel_size = 7;
  // cv::medianBlur(image, processed_image, kernel_size);
 
  // ============================================================
  // LAB — Drawing on Images
  // ============================================================
 
  // // Draw a line from top-left to center
  // processed_image = image;
  // int height = image.rows;
  // int width = image.cols;
  // cv::line(processed_image, cv::Point(0, 0), cv::Point(width / 2, height / 2),
  //          cv::Scalar(255, 0, 0), 2);
  
  // // Draw a rectangle (simulating a bounding box)
  // cv::Point top_left(width / 4, height / 4);
  // cv::Point bottom_right(3 * width / 4, 3 * height / 4);
  // cv::rectangle(processed_image, top_left, bottom_right,
  //               cv::Scalar(0, 255, 0), 2);
  
  // // Draw a filled circle at the center (simulating a detection center)
  // cv::Point center(width / 2, height / 2);
  // cv::circle(processed_image, center, 5, cv::Scalar(0, 0, 255), -1);
  
  // // Add a text label
  // cv::putText(processed_image, "Detection", cv::Point(top_left.x, top_left.y - 10),
  //             cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
 
  // ============================================================
  // LAB — Color Detection with Masks
  // ============================================================
 
  // // Convert to HSV
  // cv::Mat hsv_image;
  // cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
 
  // // Create a mask using the HSV color range
  // cv::Scalar lower_bound(lower_hsv_[0], lower_hsv_[1], lower_hsv_[2]);
  // cv::Scalar upper_bound(upper_hsv_[0], upper_hsv_[1], upper_hsv_[2]);
  // cv::Mat mask;
  // cv::inRange(hsv_image, lower_bound, upper_bound, mask);
 
  // // Morphological Opening — removes small background noise
  // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  // cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 2);
 
  // // Morphological Closing — fills holes and gaps inside the object
  // cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 5);
 
  // processed_image = mask;
 
  // ============================================================
  // LAB — Contour Detection
  // ============================================================
 
  // // Find contours in the clean mask
  // processed_image = image;
  // std::vector<std::vector<cv::Point>> contours;
  // cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
 
  // if (!contours.empty()) {
  //   // Find the largest contour by area
  //   auto largest_contour = std::max_element(contours.begin(), contours.end(),
  //     [](const std::vector<cv::Point> & a, const std::vector<cv::Point> & b) {
  //       return cv::contourArea(a) < cv::contourArea(b);
  //     });
 
  //   // Get the minimum enclosing circle — center and radius
  //   cv::Point2f center;
  //   float radius;
  //   cv::minEnclosingCircle(*largest_contour, center, radius);
 
  //   // Only process if the detected object is big enough
  //   if (radius > 10.0f) {
  //     // Get the bounding rectangle
  //     cv::Rect bbox = cv::boundingRect(*largest_contour);
 
  //     // Draw the bounding box (green)
  //     cv::rectangle(processed_image, bbox, cv::Scalar(0, 255, 0), 2);
 
  //     // Draw the center point (red, filled)
  //     cv::circle(processed_image, center, 3, cv::Scalar(0, 0, 255), -1);
  //   }
  // }

  auto processed_image_msg =
    cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::BGR8, processed_image).toImageMsg();
 
  // Lazy initialization of image_transport (shared_from_this not available in constructor)
  if (!it_) {
    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
    image_pub_ = it_->advertise(
      camera_name_ + "/processed_image", rmw_qos_profile_sensor_data);
  }
  image_pub_.publish(*processed_image_msg);
}
}  // namespace bumperbot_vision

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bumperbot_vision::ImageProcessing>("image_processing"));
  rclcpp::shutdown();
  return 0;
}