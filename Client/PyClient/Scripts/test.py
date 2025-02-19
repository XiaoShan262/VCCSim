import sys
import os
import time
from PIL import Image
import numpy as np

# Add the parent directory of PyClient to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from VCCSim import VCCSimClient

def main():
    client = VCCSimClient(host="localhost", port=50996)
    
    # test_poses = [(100, 200, 500, 0, 0, 90), (200, -300, 300, 0, 0, 180)]
    # for pose in test_poses:
    #     x, y, z, roll, pitch, yaw = pose
    #     client.send_drone_pose("Mavic", x, y, z, roll, pitch, yaw)
    #     print(f"Set drone pose to ({x}, {y}, {z}, {roll}, {pitch}, {yaw})")
    #     time.sleep(5)
    
    image_data = client.get_rgb_indexed_camera_image_data("Mavic", 0)
    img_array = np.frombuffer(image_data.data, dtype=np.uint8)
    img_array = img_array.reshape((image_data.height, image_data.width, 3))
    img = Image.fromarray(img_array, 'RGB')
    img.save('rgb_camera_image.png')
    
        
    client.close()
    
if __name__ == "__main__":
    main()