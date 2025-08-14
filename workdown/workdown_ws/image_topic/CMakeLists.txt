cmake_minimum_required(VERSION 3.8)
project(image_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

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

ament_package()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(Image_Topic_Publisher src/publisher.cpp)
add_executable(Image_Topic_Subscribe src/subscribe.cpp)

ament_target_dependencies(Image_Topic_Publisher rclcpp sensor_msgs)
ament_target_dependencies(Image_Topic_Subscribe rclcpp)

install(TARGETS
  Image_Topic_Publisher
  DESTINATION lib/${PRJECT_NAME}
)

install(TARGETS
  Image_Topic_Subscribe
  DESTINATION lib/${PROJECT_NAME}
)
