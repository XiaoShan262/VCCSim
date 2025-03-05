#!/usr/bin/env python3
import sys
import os
import time
from PIL import Image
import numpy as np
import io
import logging
import traceback

# Add parent directory to path to import VCCSimClient
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from VCCSim import VCCSimClient

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def test_send_drone_path():
    """Test the send_drone_path method of VCCSimClient."""
    
    try:
        # Initialize client
        logger.info("Initializing VCCSimClient...")
        client = VCCSimClient(host="localhost", port=50996)
        
        # Drone name
        drone_name = "Mavic"
        
        # Get current drone pose (to know where we're starting from)
        try:
            current_pose = client.get_drone_pose(drone_name)
            logger.info(f"Current drone pose: x={current_pose.x}, y={current_pose.y}, z={current_pose.z}, "
                       f"roll={current_pose.roll}, pitch={current_pose.pitch}, yaw={current_pose.yaw}")
        except Exception as e:
            logger.warning(f"Could not get current drone pose: {e}")
            logger.info("Using default starting pose")
            # Default starting pose if we can't get the current one
            current_x, current_y, current_z = 0.0, 0.0, 5.0
            current_roll, current_pitch, current_yaw = 0.0, 0.0, 0.0
        else:
            current_x, current_y, current_z = current_pose.x, current_pose.y, current_pose.z
            current_roll, current_pitch, current_yaw = current_pose.roll, current_pose.pitch, current_pose.yaw
        
        # Define a path for the drone to follow (relative to current position)
        # Format: List of (x, y, z, roll, pitch, yaw) tuples
        path = [
            # Start at current position
            (current_x, current_y, current_z, current_roll, current_pitch, current_yaw),
            
            # Move forward 10 units
            (current_x + 1000.0, current_y, current_z, current_roll, current_pitch, current_yaw),
            
            # Move right 5 units and up 2 units
            (current_x + 1000.0, current_y + 500.0, current_z + 200.0, current_roll, current_pitch, current_yaw),
            
            # Move back to starting position but 5 units higher
            (current_x, current_y, current_z + 500.0, current_roll, current_pitch, current_yaw),
            
            # Return to exact starting position
            (current_x, current_y, current_z, current_roll, current_pitch, current_yaw),
        ]
        
        # Send the path to the drone
        logger.info("Sending drone path...")
        success = client.send_drone_path(drone_name, path)
        
        if success:
            logger.info("Successfully sent drone path")
        else:
            logger.error("Failed to send drone path")
            
        # Wait for the drone to complete the path
        logger.info("Waiting for drone to complete path...")
        time.sleep(10)  # Adjust this based on your environment and path length
        
        # Get final position to verify
        try:
            final_pose = client.get_drone_pose(drone_name)
            logger.info(f"Final drone pose: x={final_pose.x}, y={final_pose.y}, z={final_pose.z}, "
                       f"roll={final_pose.roll}, pitch={final_pose.pitch}, yaw={final_pose.yaw}")
        except Exception as e:
            logger.error(f"Could not get final drone pose: {e}")
        
    except Exception as e:
        logger.error(f"Error during test execution: {e}")
        logger.error(traceback.format_exc())
    finally:
        # Close client connection
        try:
            client.close()
            logger.info("Client connection closed")
        except Exception as e:
            logger.error(f"Error closing client connection: {e}")

if __name__ == "__main__":
    test_send_drone_path()