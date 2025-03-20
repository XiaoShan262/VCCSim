cmake_minimum_required(VERSION 3.10)
project(VCCSimCppClient)

# Force C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set vcpkg toolchain file if not already set
if(MSVC)
    set(VCPKG_PATH "$ENV{VCPKG_ROOT}")
elseif(UNIX)
    set(VCPKG_PATH "$ENV{VCPKG_ROOT}")
endif()

message(STATUS "VCPKG_PATH: ${VCPKG_PATH}")

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${VCPKG_PATH}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
endif()

message(STATUS "Using toolchain file: ${CMAKE_TOOLCHAIN_FILE}")

# Add this line to set CMAKE_PREFIX_PATH
list(APPEND CMAKE_PREFIX_PATH "${VCPKG_PATH}/installed/x64-linux")

message(STATUS "Using toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

# Compiler-specific settings
if(MSVC)
  # Use proper parallel compilation flag for MSVC
  add_compile_options(/MP)
  
  # Enable exception handling
  add_compile_options(/EHsc)
elseif(UNIX)
  # Ubuntu/Linux specific compiler flags
  add_compile_options(-Wall -Wextra -Wpedantic)
  
  # Add optimizations for Release builds
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
  
  # Enable pthread support
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  
  # Position independent code is required for shared libraries on Linux
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

# Find required packages
find_package(gRPC CONFIG REQUIRED)
find_package(protobuf CONFIG REQUIRED)
find_package(OpenSSL CONFIG REQUIRED)
find_package(Threads REQUIRED)

# Add the library source files
add_library(
    vccsim_client
    SHARED
    source/VCCSimClient.cpp
    source/VCCSim.grpc.pb.cc
    source/VCCSim.pb.cc
    include/VCCSimClient.h
    include/VCCSim.pb.h
    include/VCCSim.grpc.pb.h
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

# For Ubuntu, we need to explicitly link with other dependencies
if(UNIX AND NOT APPLE)
    target_link_libraries(
        vccsim_client
        PUBLIC ${CMAKE_DL_LIBS}  # For dynamic loading support (dlopen, etc.)
        PUBLIC rt                # For POSIX realtime extensions
    )
endif()

# Installation rules with Ubuntu compatibility
# This uses GNUInstallDirs to ensure proper installation paths on Linux
include(GNUInstallDirs)
install(
    TARGETS vccsim_client
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    FILES include/VCCSimClient.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Add elegant SIMD intrinsics shim - avoid compiler-specific flags
# This approach ensures we provide the necessary intrinsics without depending on 
# specific processor architecture flags
# Create a proper header file for our shims
if(MSVC)
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
endif()

# Optional: Build the example application
option(BUILD_EXAMPLE "Build example application" OFF)
if(BUILD_EXAMPLE)
    set(EXAMPLE_SOURCES test_src/mesh_manager.cpp)

    if(MSVC)
        list(APPEND EXAMPLE_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/simd_shims.cpp)
    endif()
    
    add_executable(mesh_manager ${EXAMPLE_SOURCES})
    
    target_include_directories(mesh_manager
        PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
    )
    
    target_link_libraries(mesh_manager 
        PRIVATE vccsim_client
        PRIVATE OpenSSL::SSL
        PRIVATE OpenSSL::Crypto
        PRIVATE Threads::Threads
    )
    
    # Additional Linux-specific libraries for the example
    if(UNIX AND NOT APPLE)
        target_link_libraries(mesh_manager
            PRIVATE ${CMAKE_DL_LIBS}
            PRIVATE rt
        )
    endif()
endif()

# Add an option for building documentation (common in Linux projects)
option(BUILD_DOCUMENTATION "Build documentation" OFF)
if(BUILD_DOCUMENTATION)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        # Set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

        # Request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

        # Add a target to generate API documentation with Doxygen
        add_custom_target(doc
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM
        )
    else()
        message(STATUS "Doxygen not found, documentation will not be built")
    endif()
endif()