cmake_minimum_required(VERSION 3.8)
project(perception_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch models worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.sh.in")

ament_package()
