cmake_minimum_required(VERSION 3.5)
project(dynamixel_control_cpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
## joy_linux is not needed for build, only at runtime

include_directories(include)

# Create the command_publisher executable
add_executable(command_publisher src/command_publisher.cpp)

# Link libraries
ament_target_dependencies(command_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  dynamixel_sdk
)

# Install the executable
install(TARGETS
  command_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()