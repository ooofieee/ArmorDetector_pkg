cmake_minimum_required(VERSION 3.10)
project(armor_detector_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/resource)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces_pkg REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(video_capturer_node src/video_capturer_node.cpp)
add_executable(image_processer_node src/image_processer_node.cpp)
add_executable(number_recognizer_node src/number_recognizer_node.cpp)
add_executable(video_player_node src/video_player_node.cpp)
add_executable(aim_predictor_node src/aim_predictor_node.cpp)

ament_target_dependencies(video_capturer_node rclcpp sensor_msgs cv_bridge std_msgs)
ament_target_dependencies(image_processer_node rclcpp sensor_msgs cv_bridge std_msgs interfaces_pkg)
ament_target_dependencies(number_recognizer_node rclcpp sensor_msgs cv_bridge std_msgs interfaces_pkg)
ament_target_dependencies(video_player_node rclcpp sensor_msgs cv_bridge interfaces_pkg)
ament_target_dependencies(aim_predictor_node rclcpp interfaces_pkg)
target_link_libraries(video_capturer_node ${OpenCV_LIBRARIES})
target_link_libraries(image_processer_node ${OpenCV_LIBRARIES})
target_link_libraries(number_recognizer_node ${OpenCV_LIBRARIES})
target_link_libraries(video_player_node ${OpenCV_LIBRARIES})
target_link_libraries(aim_predictor_node ${OpenCV_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS 
  video_capturer_node 
  image_processer_node
  number_recognizer_node
  video_player_node
  aim_predictor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
  ) 

ament_package()
