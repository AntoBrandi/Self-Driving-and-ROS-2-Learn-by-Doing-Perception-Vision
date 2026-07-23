cmake_minimum_required(VERSION 3.8)
project(bumperbot_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_transport_plugins REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(point_cloud_transport_plugins REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bumperbot_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(DBoW3 REQUIRED)

find_package(PCL REQUIRED COMPONENTS common filters)

set(dependencies
  rclcpp
  image_transport
  image_transport_plugins
  point_cloud_transport
  point_cloud_transport_plugins
  message_filters
  sensor_msgs
  geometry_msgs
  bumperbot_msgs
  tf2_ros
  std_srvs
  cv_bridge
  image_geometry
  OpenCV
  pcl_ros
  pcl_conversions
)

ament_python_install_package(${PROJECT_NAME})

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${DBoW3_INCLUDE_DIRS}
)

add_executable(undistort_image
  src/undistort_image.cpp
)
ament_target_dependencies(undistort_image ${dependencies})

add_executable(image_processing
  src/image_processing.cpp
)
ament_target_dependencies(image_processing ${dependencies})

add_executable(stereo_vision
  src/stereo_vision.cpp
)
ament_target_dependencies(stereo_vision ${dependencies})

add_executable(feature_detector
  src/feature_detector.cpp
)
ament_target_dependencies(feature_detector ${dependencies})

add_executable(optical_flow
  src/optical_flow.cpp
)
ament_target_dependencies(optical_flow ${dependencies})

add_executable(point_cloud_processing src/point_cloud_processing.cpp)
ament_target_dependencies(point_cloud_processing ${dependencies})
target_include_directories(point_cloud_processing PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(point_cloud_processing ${PCL_LIBRARIES})

add_executable(bag_of_words
  src/bag_of_words.cpp
)
ament_target_dependencies(bag_of_words ${dependencies})
target_link_libraries(bag_of_words
  ${OpenCV_LIBS}
  ${DBoW3_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  undistort_image
  image_processing
  stereo_vision
  feature_detector
  optical_flow
  bag_of_words
  point_cloud_processing
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/undistort_image.py
  ${PROJECT_NAME}/image_processing.py
  ${PROJECT_NAME}/stereo_vision.py
  ${PROJECT_NAME}/feature_detector.py
  ${PROJECT_NAME}/optical_flow.py
  ${PROJECT_NAME}/bag_of_words.py
  ${PROJECT_NAME}/point_cloud_processing.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_package()
