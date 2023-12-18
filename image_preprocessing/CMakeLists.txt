cmake_minimum_required(VERSION 3.5)

project(image_preprocessing)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable
add_executable(test_node src/test_node.cpp)

# Include directories for the executable
target_include_directories(test_node
  PRIVATE
  ${sensor_msgs_INCLUDE_DIRS}
)

# Link the executable to the required libraries
ament_target_dependencies(test_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

# Install the executable
install(TARGETS 
  test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
DESTINATION share/${PROJECT_NAME})

# Install launch files, config files, and other directories if necessary
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Install the CMake package file
ament_package()
