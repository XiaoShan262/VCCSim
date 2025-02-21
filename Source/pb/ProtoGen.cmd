@echo off
setlocal

:: Set the path to protoc.exe - replace this with your actual protoc.exe path
set PROTOC_PATH=../grpc/bin/protoc.exe
set PLUGIN_PATH=../grpc/bin/grpc_cpp_plugin.exe
set PROTO_PATH=.
set OUTPUT_PATH=../VCCSim/Private/API

:: Generate gRPC C++ code
"%PROTOC_PATH%" -I="%PROTO_PATH%" --grpc_out="%OUTPUT_PATH%" --plugin=protoc-gen-grpc="%PLUGIN_PATH%" "%PROTO_PATH%\VCCSim.proto"

:: Generate protobuf C++ code
"%PROTOC_PATH%" -I="%PROTO_PATH%" --cpp_out="%OUTPUT_PATH%" "%PROTO_PATH%\VCCSim.proto"

:: Generate Python code
:: python -m grpc_tools.protoc -I "%PROTO_PATH%" --grpc_python_out="%PROTO_PATH%" --python_out="%PROTO_PATH%" "%PROTO_PATH%\VCCSim.proto"

echo Proto generation completed.