cmake_minimum_required(VERSION 3.10)
project(ros_client)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find the VCCSimClient library
find_library(VCCSIM_CLIENT_LIB vccsim_client
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../CppClient/build/
  NO_DEFAULT_PATH
)

# Include directories for the VCCSimClient headers
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../CppClient/include
)

# Add executable
add_executable(vccsim_node src/client_for_nvblox.cpp)
ament_target_dependencies(vccsim_node
  rclcpp
  sensor_msgs
)

# Link against the VCCSimClient library
target_link_libraries(vccsim_node
  ${VCCSIM_CLIENT_LIB}
)

# Install executable
install(TARGETS
  vccsim_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()