# CppClient of VCCSim

## Prerequisites

1. Install grpc by VCPKG

    ``` shell
    vcpkg install grpc
    ```

    Make sure set environment variable `VCPKG_ROOT` to the path of vcpkg.

2. Run the setup script

    ``` powershell
    .\setup.cmd
    ```

## Build

1. The directory structure should be like this:

    ``` file
    CppClient
    ├── CMakeLists.txt
    ├── source
    │   ├── main.cpp
    │   ├── VCCSimClient.cpp
    │   ├── vccsim.grpc.pb.cc
    │   ├── vccsim.pb.cc
    └── include
        ├── VCCSimClient.h
        ├── vccsim.grpc.pb.h
        └── vccsim.pb.h
    ```

2. If you do not need the testing code, you can set BUILD_EXAMPLE to OFF in CMakeLists.txt.

3. Build it as a normal CMake project.
