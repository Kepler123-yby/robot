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
find_package(aim_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(Image_Topic_Publisher src/publisher.cpp)
add_executable(Image_Topic_Subscribe src/subscribe.cpp)
add_executable(Image_To_Int src/shift.cpp)

target_include_directories(Image_Topic_Subscribe PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(Image_Topic_Subscribe  PUBLIC c_std_99 cxx_std_17)
target_include_directories(Image_To_Int PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(Image_To_Int  PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(Image_Topic_Publisher rclcpp sensor_msgs cv_bridge OpenCV)
ament_target_dependencies(Image_Topic_Subscribe "rclcpp" "aim_interfaces" cv_bridge OpenCV)
ament_target_dependencies(Image_To_Int rclcpp aim_interfaces sensor_msgs cv_bridge OpenCV)



install(TARGETS
  Image_Topic_Publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  Image_Topic_Subscribe
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  Image_To_Int
  DESTINATION lib/${PROJECT_NAME}
)
