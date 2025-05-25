cmake_minimum_required(VERSION 3.5)
project(rfid_amcl_node)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rfid_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable for running the node
add_executable(rfid_amcl_node 
  src/rfid_amcl/run_rfid_amcl_node.cpp
  src/rfid_amcl/rfid_amcl_node.cpp
  src/rfid_amcl/differential_motion_model.cpp
  src/rfid_amcl/particle_filter.cpp
  src/rfid_amcl/particle.cpp
)

ament_target_dependencies(rfid_amcl_node 
                          rclcpp 
                          geometry_msgs
                          std_msgs
                          nav_msgs
                          rfid_msgs
                          tf2
                          tf2_ros
                          tf2_geometry_msgs)

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