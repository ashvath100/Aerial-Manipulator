cmake_minimum_required(VERSION 3.5)
project(eit_playground)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Include cpp "include" directory
include_directories(include)
 
install(
    DIRECTORY models launch worlds mixers config src
    DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
 
# Install Python executables
install(PROGRAMS
  src/offboard_ctrl.py
  src/relay.py
  src/subscriber_member_function.py
  src/publisher_member_function.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
