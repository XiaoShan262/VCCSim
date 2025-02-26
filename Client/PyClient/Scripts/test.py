import sys
import os
import time
from PIL import Image
import numpy as np
import io

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from VCCSim import VCCSimClient

def load_poses_from_file(file_path):
    poses = []
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) >= 5:  # 确保至少有五列数据
                x, y, z, roll, pitch, yaw = map(float, parts[:6])
                poses.append((x * 100, y * 100, -z * 100, roll, pitch, yaw))
                print(f"Loaded pose: ({x:.2f}, {y:.2f}, {z:.2f}, {roll:.2f}, {pitch:.2f}, {yaw:.2f})")
    return poses[:100]

def save_rgb_image(image_data, filename):
    """Save RGB image data to a file."""
    # Get dimensions and raw data from the image_data
    width = image_data.width
    height = image_data.height
    data = image_data.data
    img_format = image_data.format
    
    if img_format in [2, 3]:  # JPEG or PNG compressed
        # Directly use the compressed data
        with open(filename, 'wb') as f:
            f.write(data)
        print(f"Saved compressed image to {filename}")
    else:  # Raw RGB or BGR format
        # Determine the correct PIL mode based on format
        mode = 'RGB' if img_format == 0 else 'BGR'
        
        # Convert to numpy array and reshape
        img_array = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
        
        # Convert BGR to RGB if needed
        if mode == 'BGR':
            img_array = img_array[:, :, ::-1]  # Reverse the color channels
        
        # Create PIL image
        img = Image.fromarray(img_array, 'RGB')
        img.save(filename)
        print(f"Saved raw image to {filename}")

def main():
    # Create output directory for images
    output_dir = "flash_images"
    os.makedirs(output_dir, exist_ok=True)
    
    # Initialize client
    client = VCCSimClient(host="localhost", port=50996)
    
    # Flash pawn name
    flash_name = "Flash"
    
    # Define test path with several waypoints
    # Format: (x, y, z, roll, pitch, yaw)
    test_path = load_poses_from_file(r"C:\UEProjects\VCCSimDev\Plugins\VCCSim\Client\PyClient\Scripts\ouhuang_3.txt")
    
    try:
        # Send the path to the flash pawn
        print(f"Sending path with {len(test_path)} waypoints to Flash pawn...")
        success = client.send_flash_path(flash_name, test_path)
        
        if not success:
            print("Failed to send path to Flash pawn")
            return
        
        print("Path sent successfully")
        
        # Process each waypoint
        for i, _ in enumerate(test_path):
            # Check if Flash is ready before continuing
            print(f"Waiting for Flash to reach waypoint {i+1}...")
            
            # Wait for Flash to be ready
            ready = False
            max_attempts = 30
            attempts = 0
            
            while not ready and attempts < max_attempts:
                ready = client.check_flash_ready(flash_name)
                if not ready:
                    time.sleep(0.5)
                    attempts += 1
            
            if not ready:
                print(f"Timed out waiting for Flash to reach waypoint {i+1}")
                continue
            
            print(f"Flash has reached waypoint {i+1}")
            
            # Get the current pose for reference
            current_pose = client.get_flash_pose(flash_name)
            print(f"Current position: ({current_pose.x:.2f}, {current_pose.y:.2f}, {current_pose.z:.2f})")
            print(f"Current orientation: ({current_pose.roll:.2f}, {current_pose.pitch:.2f}, {current_pose.yaw:.2f})")
            
            # Capture RGB image (assuming camera index 0)
            try:
                print("Capturing RGB image...")
                image_data = client.get_rgb_indexed_camera_image_data(flash_name, 0)
                
                # Save the image
                image_filename = os.path.join(output_dir, f"flash_waypoint_{i+1}.png")
                save_rgb_image(image_data, image_filename)
                
                # Extract image metadata
                print(f"Image dimensions: {image_data.width}x{image_data.height}")
                print(f"Image format: {image_data.format}")
                
            except Exception as e:
                print(f"Error capturing image: {e}")
            
            # Move to next waypoint if not the last one
            if i < len(test_path) - 1:
                print("Moving to next waypoint...")
                client.move_to_next(flash_name)
            else:
                print("Reached final waypoint")
    
    except Exception as e:
        print(f"An error occurred: {e}")
    
    finally:
        # Close the client connection
        print("Closing client connection...")
        client.close()
        print("Test completed")

if __name__ == "__main__":
    main()