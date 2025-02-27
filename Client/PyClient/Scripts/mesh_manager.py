#!/usr/bin/env python3
"""
Test script for adding and removing global meshes in VCCSim
"""

import os
import time
import sys
import struct
from typing import List, Dict, Tuple


sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from VCCSim import VCCSimClient


# Magic number expected by the UE mesh processor
MESH_MAGIC_NUMBER = 0x48534D55  # Hex representation of 'UMSH' in little-endian

def parse_obj_file(file_path: str) -> Tuple[List[Tuple[float, float, float]], List[List[int]]]:
    """
    Parse an OBJ file and extract vertices and triangles
    
    Args:
        file_path: Path to the OBJ file
        
    Returns:
        Tuple containing list of vertices and list of triangle indices
        Each triangle is a list of 3 indices
    """
    vertices = []
    triangles = []
    
    try:
        with open(file_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if not parts:
                    continue
                
                if parts[0] == 'v':
                    # Vertex (convert to float)
                    v = (float(parts[1]), float(parts[2]), float(parts[3]))
                    vertices.append(v)
                elif parts[0] == 'f':
                    # Face (convert to 0-based indices)
                    # OBJ format uses 1-based indexing, so subtract 1
                    face_indices = []
                    for p in parts[1:]:
                        # Handle different face formats (v, v/vt, v/vt/vn, v//vn)
                        idx = p.split('/')[0]
                        if idx:
                            face_indices.append(int(idx) - 1)
                    
                    # Only add triangles
                    if len(face_indices) == 3:
                        triangles.append(face_indices)
                    elif len(face_indices) > 3:
                        # Simple triangulation of polygons (fan method)
                        for i in range(1, len(face_indices) - 1):
                            triangles.append([face_indices[0], face_indices[i], face_indices[i+1]])
        
        return vertices, triangles
    except Exception as e:
        print(f"Error parsing OBJ file {file_path}: {e}")
        sys.exit(1)

def create_mesh_binary(vertices: List[Tuple[float, float, float]], 
                      triangles: List[List[int]]) -> bytes:
    """
    Create a binary mesh representation that matches the expected format in UFMeshManager::ProcessMeshData
    
    FMeshHeader structure:
    struct FMeshHeader {
        uint32 Magic;       // Magic number
        uint32 Version;     // Version of the format
        uint32 VertexCount; // Number of vertices
        uint32 IndexCount;  // Number of indices
        uint32 Flags;       // Additional flags
    };
    
    FCompactVertex structure:
    struct FCompactVertex {
        float X, Y, Z;
    };
    
    Args:
        vertices: List of vertex positions (x, y, z)
        triangles: List of triangle indices, where each triangle is a list of 3 indices
        
    Returns:
        Binary data in the format expected by the server
    """
    # Calculate counts
    vertex_count = len(vertices)
    
    # Flatten triangle indices into a single list
    flat_indices = []
    for tri in triangles:
        flat_indices.extend(tri)
    index_count = len(flat_indices)
    
    # Define constants
    version = 1  # Version of the format
    flags = 0    # No special flags
    
    # Create header with all 5 fields (magic, version, vertex_count, index_count, flags)
    header = struct.pack('<IIIII', MESH_MAGIC_NUMBER, version, vertex_count, index_count, flags)
    
    # Create vertices block (FCompactVertex format: 3 floats)
    vertices_data = bytearray()
    for v in vertices:
        vertices_data.extend(struct.pack('<fff', v[0], v[1], v[2]))
    
    # Create indices block (uint32 format)
    indices_data = bytearray()
    for idx in flat_indices:
        indices_data.extend(struct.pack('<I', idx))
    
    # Combine into final binary mesh data
    mesh_data = header + vertices_data + indices_data
    
    # Debug info to verify sizes
    header_size = struct.calcsize('<IIIII')  # 5 uint32s
    vertex_size = struct.calcsize('<fff')    # 3 floats per vertex
    index_size = struct.calcsize('<I')       # 1 uint32 per index
    
    expected_size = header_size + (vertex_count * vertex_size) + (index_count * index_size)
    actual_size = len(mesh_data)
    
    print(f"Mesh binary size breakdown:")
    print(f"  Header: {header_size} bytes")
    print(f"  Vertices ({vertex_count}): {vertex_count * vertex_size} bytes")
    print(f"  Indices ({index_count}): {index_count * index_size} bytes")
    print(f"  Expected total: {expected_size} bytes")
    print(f"  Actual total: {actual_size} bytes")
    
    # Verify the binary size matches our calculation
    assert actual_size == expected_size, f"Size mismatch: expected {expected_size}, got {actual_size}"
    
    return mesh_data

def main():
    # Path to the OBJ files
    obj_dir = r"C:\UEProjects\VCCSimDev\Plugins\VCCSim\test\hull"
    
    # List all OBJ files
    obj_files = [os.path.join(obj_dir, f) for f in os.listdir(obj_dir) if f.endswith('.obj')]
    
    if not obj_files:
        print(f"No OBJ files found in {obj_dir}")
        sys.exit(1)
    
    print(f"Found {len(obj_files)} OBJ files:")
    for i, file_path in enumerate(obj_files):
        print(f"  {i+1}. {os.path.basename(file_path)}")
    
    # Initialize the client
    client = VCCSimClient()
    print("Connected to VCCSim server")
    
    # Dictionary to store mesh IDs
    mesh_ids: Dict[str, int] = {}
    
    try:
        # Add all meshes
        print("\nAdding global meshes...")
        for i, file_path in enumerate(obj_files):
            file_name = os.path.basename(file_path)
            
            # Parse OBJ file
            print(f"Parsing {file_name}...", end="", flush=True)
            vertices, triangles = parse_obj_file(file_path)
            print(f" Done. {len(vertices)} vertices, {len(triangles)} triangles")
            
            # Create binary mesh data
            mesh_data = create_mesh_binary(vertices, triangles)
            file_size = len(mesh_data) / 1024  # Size in KB
            
            # Different transform for each mesh to place them side by side
            x_offset = i * 5.0  # 5 units apart on X axis
            transform_pose = (x_offset, 0.0, 0.0, 0.0, 0.0, 0.0)  # x, y, z, roll, pitch, yaw
            
            # Send the mesh
            print(f"Sending {file_name} ({file_size:.2f} KB)...", end="", flush=True)
            mesh_id = client.send_global_mesh(
                data=mesh_data,
                format=0,          # Format identifier (adjust as needed)
                version=1,         # Format version (adjust as needed)
                simplified=False,  # Using full mesh detail
                transform_pose=transform_pose
            )
            
            mesh_ids[file_name] = mesh_id
            print(f" Done. Mesh ID: {mesh_id}")
            
            # Brief pause between adding meshes
            time.sleep(0.5)
        
        # Display all added meshes
        print("\nAdded global meshes:")
        for file_name, mesh_id in mesh_ids.items():
            print(f"  {file_name}: ID {mesh_id}")
        
        # Wait for user to observe the meshes in the simulator
        input("\nPress Enter to start removing meshes one by one...")
        
        # Remove meshes one by one
        print("\nRemoving global meshes...")
        for file_name, mesh_id in mesh_ids.items():
            print(f"Removing {file_name} (ID: {mesh_id})...", end="", flush=True)
            success = client.remove_global_mesh(mesh_id)
            print(f" {'Success' if success else 'Failed'}")
            
            # Brief pause between removals
            time.sleep(1.0)
        
        print("\nAll meshes removed.")
        
    except Exception as e:
        print(f"\nError during test: {e}")
    finally:
        # Clean up
        print("\nClosing connection...")
        client.close()
        print("Test completed.")

if __name__ == "__main__":
    main()