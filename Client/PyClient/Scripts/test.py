import sys
import os
import time
from PIL import Image
import numpy as np
import io

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from VCCSim import VCCSimClient

def main():
    client = VCCSimClient(host="localhost", port=50996)
    
    # Get RGB camera image
    image_data = client.get_rgb_indexed_camera_image_data("Mavic", 0)
    
    # Debug information
    print(f"Received data length: {len(image_data.data)}")
    print(f"Image format: {image_data.format}")
    print(f"Image dimensions: {image_data.width}x{image_data.height}")
    print(f"Is compressed: {image_data.is_compressed}")
    
    try:
        img = Image.open(io.BytesIO(image_data.data))
        img.save('rgb_camera_image.png')
    except Exception as e:
        print(f"Error opening image: {str(e)}")
        
        # Try to peek at the first few bytes
        if len(image_data.data) > 8:
            print(f"First 8 bytes: {[hex(b) for b in image_data.data[:8]]}")
    
    client.close()

if __name__ == "__main__":
    main()