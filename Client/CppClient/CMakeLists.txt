cmake_minimum_required(VERSION 3.10)
project(VCCSimCppClient)

# Force C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set vcpkg toolchain file if not already set
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
endif()

# MSVC specific settings - using proper conditionals
if(MSVC)
  # Use proper parallel compilation flag
  add_compile_options(/MP)
  
  # Enable exception handling
  add_compile_options(/EHsc)
endif()

# Find required packages
find_package(gRPC CONFIG REQUIRED)
find_package(protobuf CONFIG REQUIRED)
find_package(OpenSSL CONFIG REQUIRED)
find_package(Threads REQUIRED)

# Add the library source files
add_library(
    vccsim_client
    source/VCCSimClient.cpp
    source/VCCSim.grpc.pb.cc
    source/VCCSim.pb.cc
    include/VCCSimClient.h
    ${VCCSIM_PB_CC}
    ${VCCSIM_PB_H}
    ${VCCSIM_GRPC_PB_CC}
    ${VCCSIM_GRPC_PB_H}
)

target_include_directories(
    vccsim_client
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link with specific components
target_link_libraries(
    vccsim_client
    PUBLIC gRPC::grpc++
    PUBLIC protobuf::libprotobuf
)

# Installation rules
install(
    TARGETS vccsim_client
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(
    FILES include/VCCSimClient.h
    DESTINATION include
)

# Add elegant SIMD intrinsics shim - avoid compiler-specific flags
# This approach ensures we provide the necessary intrinsics without depending on 
# specific processor architecture flags
# Create a proper header file for our shims
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/simd_shims.h 
"#pragma once
#include <algorithm>

// Declarations for SIMD intrinsics shims
extern \"C\" {
    double const* __std_max_element_d(double const* first, double const* last);
    void* __std_min_8i(int64_t const* first, int64_t const* last);
}
")

# Create the implementation file
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/simd_shims.cpp
"#include \"simd_shims.h\"

// Provide shim implementations for the missing SIMD intrinsics
extern \"C\" {
    // Shim for __std_max_element_d using standard algorithms
    double const* __std_max_element_d(double const* first, double const* last) {
        return std::max_element(first, last);
    }
    
    // Shim for __std_min_8i using standard algorithms
    void* __std_min_8i(int64_t const* first, int64_t const* last) {
        return (void*)std::min_element(first, last);
    }
}
")

# Optional: Build the example application
option(BUILD_EXAMPLE "Build example application" ON)
if(BUILD_EXAMPLE)
    add_executable(mesh_manager 
        test_src/mesh_manager.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/simd_shims.cpp
    )
    
    target_include_directories(mesh_manager
        PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
    )
    
    target_link_libraries(mesh_manager 
        PRIVATE vccsim_client
        PRIVATE OpenSSL::SSL
        PRIVATE OpenSSL::Crypto
        PRIVATE Threads::Threads
    )
endif()