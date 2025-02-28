@echo off
setlocal

set CURRENT_DIR=%~dp0
set PROTO_PATH=%CURRENT_DIR%..\..\Source\pb\VCCSim.proto
if not exist "%PROTO_PATH%" (
    echo Error: Proto file not found at %PROTO_PATH%
    exit /b 1
)

if not defined VCPKG_ROOT (
    echo Error: VCPKG_ROOT environment variable is not set
    exit /b 1
)

set PROTOC_PATH=%VCPKG_ROOT%\installed\x64-windows\tools\protobuf\protoc.exe
set PLUGIN_PATH=%VCPKG_ROOT%\installed\x64-windows\tools\grpc\grpc_cpp_plugin.exe
set OUTPUT_PATH=.

if not exist "%PROTOC_PATH%" (
    echo Error: protoc not found at %PROTOC_PATH%
    exit /b 1
)

if not exist "%PLUGIN_PATH%" (
    echo Error: grpc_cpp_plugin not found at %PLUGIN_PATH%
    exit /b 1
)

:: Create include and source directories if they don't exist
if not exist ".\include" mkdir ".\include"
if not exist ".\source" mkdir ".\source"

:: Generate gRPC C++ code
"%PROTOC_PATH%" -I="%CURRENT_DIR%..\..\Source\pb" --grpc_out="%OUTPUT_PATH%" --plugin=protoc-gen-grpc="%PLUGIN_PATH%" "%PROTO_PATH%"

:: Generate protobuf C++ code
"%PROTOC_PATH%" -I="%CURRENT_DIR%..\..\Source\pb" --cpp_out="%OUTPUT_PATH%" "%PROTO_PATH%"

:: Move header files to include directory
move /Y *.pb.h .\include\

:: Move source files to source directory
move /Y *.pb.cc .\source\

echo Proto generation completed. Files moved to include and source directories.