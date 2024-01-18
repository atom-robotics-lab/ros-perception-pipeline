cmake_minimum_required(VERSION 3.5)
project(image_preprocessing)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Specify the path to your header file's directory
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/home/rachit/percep_ws/src/ros-perception-pipeline/image_preprocessing/include)

# Include the directory containing your header file
include_directories(${INCLUDE_DIR})

# Add executables
add_executable(image_process_node src/image_process_node.cpp src/pre_processing_functions.cpp)

# Include directories for the executable
target_include_directories(image_process_node
  PRIVATE
  ${sensor_msgs_INCLUDE_DIRS}
)

# Link the executable to the required libraries
ament_target_dependencies(image_process_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

# Install the executable
install(TARGETS 
  image_process_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
DESTINATION share/${PROJECT_NAME})

include_directories(include)

# Install the CMake package file
ament_package()
