#!/bin/bash

# Get the directory where the script is located
CURRENT_DIR=$(dirname "$(readlink -f "$0")")
PROTO_PATH="${CURRENT_DIR}/../../Source/pb/VCCSim.proto"

# Check if proto file exists
if [ ! -f "$PROTO_PATH" ]; then
    echo "Error: Proto file not found at $PROTO_PATH"
    exit 1
fi

# Check if VCPKG_ROOT is defined
if [ -z "$VCPKG_ROOT" ]; then
    echo "Error: VCPKG_ROOT environment variable is not set"
    exit 1
fi

# Define paths
PROTOC_PATH="${VCPKG_ROOT}/installed/x64-linux/tools/protobuf/protoc"
PLUGIN_PATH="${VCPKG_ROOT}/installed/x64-linux/tools/grpc/grpc_cpp_plugin"
OUTPUT_PATH="."

# Check if protoc exists
if [ ! -f "$PROTOC_PATH" ]; then
    echo "Error: protoc not found at $PROTOC_PATH"
    exit 1
fi

# Check if grpc_cpp_plugin exists
if [ ! -f "$PLUGIN_PATH" ]; then
    echo "Error: grpc_cpp_plugin not found at $PLUGIN_PATH"
    exit 1
fi

# Create include and source directories if they don't exist
mkdir -p "./include"
mkdir -p "./source"

# Generate gRPC C++ code
"$PROTOC_PATH" -I="${CURRENT_DIR}/../../Source/pb" --grpc_out="$OUTPUT_PATH" --plugin=protoc-gen-grpc="$PLUGIN_PATH" "$PROTO_PATH"

# Generate protobuf C++ code
"$PROTOC_PATH" -I="${CURRENT_DIR}/../../Source/pb" --cpp_out="$OUTPUT_PATH" "$PROTO_PATH"

# Move header files to include directory
mv -f *.pb.h ./include/

# Move source files to source directory
mv -f *.pb.cc ./source/

echo "Proto generation completed. Files moved to include and source directories."