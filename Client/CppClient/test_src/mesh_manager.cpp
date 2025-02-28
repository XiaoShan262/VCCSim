/**
 * Test application for adding and removing global meshes in VCCSim
 * C++ version of the Python test script - Updated to match newest client API
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <tuple>
#include <chrono>
#include <thread>
#include <cstdint>
#include <sstream>  // Added missing include for istringstream
#include "VCCSimClient.h"

// For older MSVC compilers that don't use C++17 by default
#ifdef _MSC_VER
    #if _MSC_VER < 1914 || !defined(_MSVC_LANG) || _MSVC_LANG < 201703L
        // Define simple path-related functions to replace std::filesystem
        namespace fs_helper {
            bool exists(const std::string& path) {
                std::ifstream file(path);
                return file.good();
            }
            
            std::string filename(const std::string& path) {
                size_t pos = path.find_last_of("/\\");
                if (pos != std::string::npos) {
                    return path.substr(pos + 1);
                }
                return path;
            }
            
            std::string extension(const std::string& path) {
                size_t pos = path.find_last_of(".");
                if (pos != std::string::npos) {
                    return path.substr(pos);
                }
                return "";
            }
            
            std::vector<std::string> list_files(const std::string& dir, const std::string& ext) {
                std::vector<std::string> files;
                // This is a simplified version that works on Windows only
                // For a real project, use proper platform-specific code or third-party libraries
                std::string cmd = "dir /b \"" + dir + "\"";
                FILE* pipe = _popen(cmd.c_str(), "r");
                if (pipe) {
                    char buffer[256];
                    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                        std::string file = buffer;
                        // Remove newline character
                        if (!file.empty() && file[file.size() - 1] == '\n') {
                            file.erase(file.size() - 1);
                        }
                        
                        if (extension(file) == ext) {
                            files.push_back(dir + "\\" + file);
                        }
                    }
                    _pclose(pipe);
                }
                return files;
            }
        }
    #else
        #include <filesystem>
        namespace fs_helper = std::filesystem;
    #endif
#else
    #include <filesystem>
    namespace fs_helper = std::filesystem;
#endif

// Magic number expected by the UE mesh processor - 'UMSH' in little-endian
const uint32_t MESH_MAGIC_NUMBER = 0x48534D55;

// Struct to represent a 3D vertex
struct Vertex {
    float x, y, z;
};

// Typedef for triangle indices
typedef std::vector<uint32_t> Triangle;

// Function to parse an OBJ file and extract vertices and triangles
std::pair<std::vector<Vertex>, std::vector<Triangle>> parseObjFile(const std::string& filePath) {
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
    
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error opening OBJ file: " << filePath << std::endl;
        exit(1);
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        
        if (token == "v") {
            // Parse vertex
            Vertex v;
            if (iss >> v.x >> v.y >> v.z) {
                vertices.push_back(v);
            }
        }
        else if (token == "f") {
            // Parse face
            std::vector<uint32_t> faceIndices;
            std::string indexStr;
            
            while (iss >> indexStr) {
                // Handle different face formats (v, v/vt, v/vt/vn, v//vn)
                size_t slashPos = indexStr.find('/');
                std::string vertexIndexStr;
                
                if (slashPos != std::string::npos) {
                    vertexIndexStr = indexStr.substr(0, slashPos);
                } else {
                    vertexIndexStr = indexStr;
                }
                
                // OBJ format uses 1-based indexing, so subtract 1
                if (!vertexIndexStr.empty()) {
                    uint32_t index = std::stoi(vertexIndexStr) - 1;
                    faceIndices.push_back(index);
                }
            }
            
            // Only add triangles, or triangulate if more than 3 vertices
            if (faceIndices.size() == 3) {
                triangles.push_back({faceIndices[0], faceIndices[1], faceIndices[2]});
            }
            else if (faceIndices.size() > 3) {
                // Simple triangulation of polygons (fan method)
                for (size_t i = 1; i < faceIndices.size() - 1; ++i) {
                    triangles.push_back({faceIndices[0], faceIndices[i], faceIndices[i+1]});
                }
            }
        }
    }
    
    file.close();
    return std::make_pair(vertices, triangles);
}

// Function to create binary mesh data in the format expected by UFMeshManager::ProcessMeshData
std::vector<uint8_t> createMeshBinary(const std::vector<Vertex>& vertices, 
                                     const std::vector<Triangle>& triangles) {
    // Calculate counts
    uint32_t vertexCount = static_cast<uint32_t>(vertices.size());
    
    // Flatten triangle indices into a single vector
    std::vector<uint32_t> flatIndices;
    for (const auto& tri : triangles) {
        flatIndices.insert(flatIndices.end(), tri.begin(), tri.end());
    }
    uint32_t indexCount = static_cast<uint32_t>(flatIndices.size());
    
    // Define constants
    uint32_t version = 1;  // Version of the format
    uint32_t flags = 0;    // No special flags
    
    // Calculate sizes
    const size_t headerSize = 5 * sizeof(uint32_t);  // 5 uint32 fields
    const size_t vertexSize = 3 * sizeof(float);     // 3 floats per vertex
    const size_t indexSize = sizeof(uint32_t);       // 1 uint32 per index
    size_t totalSize = headerSize + (vertexCount * vertexSize) + (indexCount * indexSize);
    
    // Prepare the binary data buffer
    std::vector<uint8_t> meshData(totalSize);
    uint8_t* buffer = meshData.data();
    size_t offset = 0;
    
    // Write header
    memcpy(buffer + offset, &MESH_MAGIC_NUMBER, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(buffer + offset, &version, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(buffer + offset, &vertexCount, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(buffer + offset, &indexCount, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(buffer + offset, &flags, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // Write vertices
    for (const auto& v : vertices) {
        memcpy(buffer + offset, &v.x, sizeof(float));
        offset += sizeof(float);
        memcpy(buffer + offset, &v.y, sizeof(float));
        offset += sizeof(float);
        memcpy(buffer + offset, &v.z, sizeof(float));
        offset += sizeof(float);
    }
    
    // Write indices
    for (const auto& idx : flatIndices) {
        memcpy(buffer + offset, &idx, sizeof(uint32_t));
        offset += sizeof(uint32_t);
    }
    
    // Debug info to verify sizes
    std::cout << "Mesh binary size breakdown:" << std::endl;
    std::cout << "  Header: " << headerSize << " bytes" << std::endl;
    std::cout << "  Vertices (" << vertexCount << "): " << (vertexCount * vertexSize) << " bytes" << std::endl;
    std::cout << "  Indices (" << indexCount << "): " << (indexCount * indexSize) << " bytes" << std::endl;
    std::cout << "  Expected total: " << totalSize << " bytes" << std::endl;
    std::cout << "  Actual total: " << meshData.size() << " bytes" << std::endl;
    
    // Verify the binary size matches our calculation
    if (meshData.size() != totalSize) {
        std::cerr << "Size mismatch: expected " << totalSize << ", got " << meshData.size() << std::endl;
        exit(1);
    }
    
    return meshData;
}

int main() {
    // Path to the OBJ files
    std::string objDir = "C:\\UEProjects\\VCCSimDev\\Plugins\\VCCSim\\test\\hull";
    
    // List all OBJ files
    std::vector<std::string> objFiles;
    
    try {
        #ifdef _MSC_VER
            #if _MSC_VER < 1914 || !defined(_MSVC_LANG) || _MSVC_LANG < 201703L
                // Use the helper for older compilers
                objFiles = fs_helper::list_files(objDir, ".obj");
            #else
                // Use std::filesystem for C++17 support
                for (const auto& entry : fs_helper::directory_iterator(objDir)) {
                    if (fs_helper::is_regular_file(entry) && fs_helper::path(entry).extension() == ".obj") {
                        objFiles.push_back(entry.path().string());
                    }
                }
            #endif
        #else
            // Use std::filesystem for non-MSVC compilers
            for (const auto& entry : fs_helper::directory_iterator(objDir)) {
                if (fs_helper::is_regular_file(entry) && entry.path().extension() == ".obj") {
                    objFiles.push_back(entry.path().string());
                }
            }
        #endif
    } catch (const std::exception& e) {
        std::cerr << "Error listing OBJ files: " << e.what() << std::endl;
        return 1;
    }
    
    if (objFiles.empty()) {
        std::cerr << "No OBJ files found in " << objDir << std::endl;
        return 1;
    }
    
    std::cout << "Found " << objFiles.size() << " OBJ files:" << std::endl;
    for (size_t i = 0; i < objFiles.size(); ++i) {
        #ifdef _MSC_VER
            #if _MSC_VER < 1914 || !defined(_MSVC_LANG) || _MSVC_LANG < 201703L
                std::cout << "  " << (i+1) << ". " << fs_helper::filename(objFiles[i]) << std::endl;
            #else
                std::cout << "  " << (i+1) << ". " << fs_helper::path(objFiles[i]).filename().string() << std::endl;
            #endif
        #else
            std::cout << "  " << (i+1) << ". " << fs_helper::path(objFiles[i]).filename().string() << std::endl;
        #endif
    }
    
    // Initialize the client
    VCCSimClient client;
    std::cout << "Connected to VCCSim server" << std::endl;
    
    // Map to store mesh IDs
    std::map<std::string, int> meshIds;
    
    try {
        // Add all meshes
        std::cout << "\nAdding global meshes..." << std::endl;
        for (size_t i = 0; i < objFiles.size(); ++i) {
            std::string filePath = objFiles[i];
            
            // Get file name for display
            std::string fileName;
            #ifdef _MSC_VER
                #if _MSC_VER < 1914 || !defined(_MSVC_LANG) || _MSVC_LANG < 201703L
                    fileName = fs_helper::filename(filePath);
                #else
                    fileName = fs_helper::path(filePath).filename().string();
                #endif
            #else
                fileName = fs_helper::path(filePath).filename().string();
            #endif
            
            // Parse OBJ file
            std::cout << "Parsing " << fileName << "..." << std::flush;
            auto parsed = parseObjFile(filePath);
            const auto& vertices = parsed.first;
            const auto& triangles = parsed.second;
            std::cout << " Done. " << vertices.size() << " vertices, " << triangles.size() << " triangles" << std::endl;
            
            // Create binary mesh data
            std::vector<uint8_t> meshData = createMeshBinary(vertices, triangles);
            float fileSize = meshData.size() / 1024.0f;  // Size in KB
            
            // Convert binary data to string
            std::string meshDataStr(reinterpret_cast<char*>(meshData.data()), meshData.size());
            
            // Different transform for each mesh to place them side by side
            float xOffset = i * 5.0f;  // 5 units apart on X axis
            
            // Create a tuple for the transform pose (x, y, z, roll, pitch, yaw)
            auto transformPose = std::make_tuple(xOffset, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            
            // Send the mesh using the updated API
            std::cout << "Sending " << fileName << " (" << fileSize << " KB)..." << std::flush;
            int meshId = client.SendGlobalMesh(
                meshDataStr,          // String data instead of raw bytes
                0,                    // Format identifier
                1,                    // Format version
                false,                // Using full mesh detail
                transformPose         // Transform pose as a tuple
            );
            
            meshIds[fileName] = meshId;
            std::cout << " Done. Mesh ID: " << meshId << std::endl;
            
            // Brief pause between adding meshes
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        // Display all added meshes
        std::cout << "\nAdded global meshes:" << std::endl;
        for (const auto& pair : meshIds) {
            const auto& fileName = pair.first;
            const auto& meshId = pair.second;
            std::cout << "  " << fileName << ": ID " << meshId << std::endl;
        }
        
        // Wait for user to observe the meshes in the simulator
        std::cout << "\nPress Enter to start removing meshes one by one..." << std::endl;
        std::cin.get();
        
        // Remove meshes one by one
        std::cout << "\nRemoving global meshes..." << std::endl;
        for (const auto& pair : meshIds) {
            const auto& fileName = pair.first;
            const auto& meshId = pair.second;
            std::cout << "Removing " << fileName << " (ID: " << meshId << ")..." << std::flush;
            bool success = client.RemoveGlobalMesh(meshId);
            std::cout << " " << (success ? "Success" : "Failed") << std::endl;
            
            // Brief pause between removals
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        
        std::cout << "\nAll meshes removed." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "\nError during test: " << e.what() << std::endl;
    }
    
    // Cleanup happens in the VCCSimClient destructor
    std::cout << "\nClosing connection..." << std::endl;
    std::cout << "Test completed." << std::endl;
    
    return 0;
}