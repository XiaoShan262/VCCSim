import sys
import os
import time
from PIL import Image
import numpy as np
import io
import logging
import traceback

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from VCCSim import VCCSimClient

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("flash_test.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

def load_poses_from_file(file_path):
    """Load poses from a text file."""
    poses = []
    try:
        with open(file_path, 'r') as file:
            for line in file:
                parts = line.strip().split()
                if len(parts) >= 6:  # Ensure at least six columns of data
                    x, y, z, roll, pitch, yaw = map(float, parts[:6])
                    poses.append((x * 100, y * 100, -z * 100, roll, pitch, yaw))
        logger.info(f"Loaded {len(poses)} poses from {file_path}")
        return poses
    except Exception as e:
        logger.error(f"Error loading poses from {file_path}: {e}")
        return []

def save_rgb_image(image_data, filename):
    """Save RGB image data to a file with error handling."""
    try:
        # Get dimensions and raw data from the image_data
        width = image_data.width
        height = image_data.height
        data = image_data.data
        img_format = image_data.format
        
        if img_format in [2, 3]:  # JPEG or PNG compressed
            # Directly use the compressed data
            with open(filename, 'wb') as f:
                f.write(data)
            logger.info(f"Saved compressed image to {filename}")
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
            logger.info(f"Saved raw image to {filename}")
        return True
    except Exception as e:
        logger.error(f"Error saving image to {filename}: {e}")
        logger.error(traceback.format_exc())
        return False

def process_and_save_image(client, flash_name, output_dir, waypoint_idx):
    """Process and save image with retries and graceful error handling."""
    image_filename = os.path.join(output_dir, f"flash_waypoint_{waypoint_idx+1}.png")
    max_retries = 3
    
    for retry in range(max_retries):
        try:
            logger.info(f"Capturing RGB image (attempt {retry+1})...")
            image_data = client.get_rgb_indexed_camera_image_data(flash_name, 0)
            
            success = save_rgb_image(image_data, image_filename)
            if success:
                logger.info(f"Image dimensions: {image_data.width}x{image_data.height}")
                logger.info(f"Image format: {image_data.format}")
                return True
                
        except Exception as e:
            error_msg = str(e)
            if "RESOURCE_EXHAUSTED" in error_msg:
                logger.warning(f"Image too large (attempt {retry+1}): {e}")
                # If the image is too large, we'll need to handle it on post-processing
                # Save a placeholder instead
                try:
                    with open(image_filename.replace(".png", f"_error_{retry+1}.txt"), 'w') as f:
                        f.write(f"Image capture failed: {error_msg}")
                    
                    # We'll need to implement post-processing logic later
                    logger.info("Saved error information for later processing")
                    
                    if retry == max_retries - 1:
                        # On last retry, save a simple placeholder image
                        img = Image.new('RGB', (640, 480), color=(0, 0, 0))
                        img.save(image_filename)
                        logger.info(f"Saved placeholder image to {image_filename}")
                        return False
                except Exception as save_error:
                    logger.error(f"Error saving error information: {save_error}")
            else:
                logger.error(f"Error capturing image: {e}")
                logger.error(traceback.format_exc())
                break  # Exit retry loop for other types of errors
                
        time.sleep(1 + retry)  # Increasing wait between retries
    
    return False

def main():
    # Create output directory for images
    output_dir = "test/flash_images"
    os.makedirs(output_dir, exist_ok=True)
    
    # Initialize client with increased message size limits
    client = VCCSimClient(host="localhost", port=50996, max_message_length=20 * 1024 * 1024)
    
    # Flash pawn name
    flash_name = "Flash"
    
    # Define test path with several waypoints
    test_path_file = r"C:\UEProjects\VCCSimDev\Plugins\VCCSim\Client\PyClient\test_scripts\ouhuang_3.txt"
    test_path = load_poses_from_file(test_path_file)
    
    if not test_path:
        logger.error("Failed to load path or path is empty")
        return
    
    try:
        # Send the path to the flash pawn
        logger.info(f"Sending path with {len(test_path)} waypoints to Flash pawn...")
        success = client.send_flash_path(flash_name, test_path)
        
        if not success:
            logger.error("Failed to send path to Flash pawn")
            return
        
        logger.info("Path sent successfully")
        
        # Process each waypoint with error handling and retries
        for i, _ in enumerate(test_path):
            # Check if Flash is ready before continuing
            logger.info(f"Waiting for Flash to reach waypoint {i+1}...")
            
            # Wait for Flash to be ready with timeout
            ready = False
            max_attempts = 30
            attempts = 0
            
            while not ready and attempts < max_attempts:
                try:
                    ready = client.check_flash_ready(flash_name)
                    if not ready:
                        time.sleep(0.5)
                        attempts += 1
                except Exception as e:
                    logger.warning(f"Error checking if Flash is ready: {e}")
                    time.sleep(1)  # Wait longer on error
                    attempts += 1
            
            if not ready:
                logger.warning(f"Timed out waiting for Flash to reach waypoint {i+1}")
                continue
            
            logger.info(f"Flash has reached waypoint {i+1}")
            
            # Get the current pose for reference
            try:
                current_pose = client.get_flash_pose(flash_name)
                logger.info(f"Current position: ({current_pose.x:.2f}, {current_pose.y:.2f}, {current_pose.z:.2f})")
                logger.info(f"Current orientation: ({current_pose.roll:.2f}, {current_pose.pitch:.2f}, {current_pose.yaw:.2f})")
            except Exception as e:
                logger.error(f"Error getting Flash pose: {e}")
            
            # Process and save image
            image_success = process_and_save_image(client, flash_name, output_dir, i)
            if not image_success:
                logger.warning(f"Note: Image capture had issues at waypoint {i+1}")
            
            # Move to next waypoint if not the last one
            if i < len(test_path) - 1:
                logger.info("Moving to next waypoint...")
                try:
                    client.move_to_next(flash_name)
                except Exception as e:
                    logger.error(f"Error moving to next waypoint: {e}")
            else:
                logger.info("Reached final waypoint")
    
    except Exception as e:
        logger.error(f"An error occurred: {e}")
        logger.error(traceback.format_exc())
    
    finally:
        # Close the client connection
        logger.info("Closing client connection...")
        client.close()
        logger.info("Test completed")

if __name__ == "__main__":
    main()