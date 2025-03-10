import sys
import os
import time
from PIL import Image
import numpy as np


# Add the parent directory of PyClient to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from VCCSim import VCCSimClient


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

def main():
    client = VCCSimClient(host="localhost", port=50996)

    index = 0
    while(True):
        image_data = client.get_rgb_indexed_camera_image_data("Mavic", 0)
        save_rgb_image(image_data, f"test/flash_images/image_{index}.png")
        lidar_data = client.get_lidar_data("Mavic")
        index += 1

if __name__ == "__main__":
    main()