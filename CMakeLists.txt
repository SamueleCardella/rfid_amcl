cmake_minimum_required(VERSION 3.5)
project(rfid_amcl_node)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Include directories
include_directories(include)

# Add executable for running the node
add_executable(rfid_amcl_node 
  src/rfid_amcl/run_rfid_amcl_node.cpp
  src/rfid_amcl/rfid_amcl_node.cpp
)

ament_target_dependencies(rfid_amcl_node rclcpp)

# Install targets
install(TARGETS
  rfid_amcl_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Export dependencies
ament_package()