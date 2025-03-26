cmake_minimum_required(VERSION 3.10)
project(ros_client)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set vcpkg toolchain file if not already set
set(VCPKG_PATH "$ENV{VCPKG_ROOT}")

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${VCPKG_PATH}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
endif()

# Add this line to set CMAKE_PREFIX_PATH
list(APPEND CMAKE_PREFIX_PATH "${VCPKG_PATH}/installed/x64-linux")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Configure RPATH settings for better library handling
# These settings ensure libraries can be found relative to the executable
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED)
find_package(JPEG REQUIRED)

find_package(gRPC CONFIG REQUIRED)
find_package(protobuf CONFIG REQUIRED)

# Define the path to the CppClient directory
set(VCCSIM_CPP_CLIENT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../CppClient)

# Find the VCCSimClient library
find_library(VCCSIM_CLIENT_LIB vccsim_client
  PATHS ${VCCSIM_CPP_CLIENT_DIR}/build/
  NO_DEFAULT_PATH
)

# Include directories for the VCCSimClient headers
include_directories(
  ${VCCSIM_CPP_CLIENT_DIR}/include
  include
)

# Add executable
add_executable(vccsim_node
  include/client_base.hpp
  src/client_base.cpp
)

ament_target_dependencies(vccsim_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  tf2_ros
)

# Link against the VCCSimClient library
target_link_libraries(vccsim_node
  ${VCCSIM_CLIENT_LIB}
  ${OpenCV_LIBS}
  gRPC::grpc++
  protobuf::libprotobuf
  JPEG::JPEG
)

# Install the VCCSim client library alongside our executable
install(FILES ${VCCSIM_CLIENT_LIB}
  DESTINATION lib
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