################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dynamixel_workbench_controllers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)

################################################################################
# Build
################################################################################

add_executable(find_dynamixel src/find_dynamixel.cpp src/dynamixel_driver.cpp src/dynamixel_item.cpp src/dynamixel_tool.cpp src/dynamixel_workbench.cpp)
ament_target_dependencies(find_dynamixel rclcpp dynamixel_sdk)
target_link_libraries(find_dynamixel yaml-cpp)

add_executable(dynamixel_workbench_controllers src/dynamixel_workbench_controllers.cpp src/dynamixel_driver.cpp src/dynamixel_item.cpp src/dynamixel_tool.cpp src/dynamixel_workbench.cpp)
ament_target_dependencies(dynamixel_workbench_controllers rclcpp dynamixel_workbench_msgs sensor_msgs dynamixel_sdk)
target_link_libraries(dynamixel_workbench_controllers yaml-cpp)

#################################################################################
## Install
#################################################################################
install(TARGETS
  find_dynamixel
  dynamixel_workbench_controllers
  DESTINATION lib/${PROJECT_NAME})

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()