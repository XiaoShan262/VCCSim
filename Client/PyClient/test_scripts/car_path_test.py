#!/usr/bin/env python3
import sys
import os
import time
import logging
import traceback

# Add the parent directory to the path to import VCCSimClient
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from VCCSim import VCCSimClient

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_send_car_path():
    """Test the send_car_path method of VCCSimClient."""

    try:
        # Initialize the client
        logger.info("Initializing VCCSimClient...")
        client = VCCSimClient(host="localhost", port=50996)

        # Car name
        car_name = "Husky"

        # Get the current car pose (to determine the starting point)
        try:
            current_pose = client.get_car_odom(car_name)
            logger.info(f"Current car pose: x={current_pose.pose.x}, y={current_pose.pose.y}, z={current_pose.pose.z}")

            current_x = current_pose.pose.x
            current_y = current_pose.pose.y
            current_z = current_pose.pose.z
            current_yaw = 0.0  # Default yaw or extract from quaternion if available

        except Exception as e:
            logger.warning(f"Could not get current car pose: {e}")
            logger.info("Using default starting pose")
            # Default starting pose if the current one cannot be retrieved
            current_x, current_y, current_z = 0.0, 0.0, 0.0
            current_yaw = 0.0

        # Define a path for the car to follow (S-shaped trajectory example)
        # Each point is defined as: (x, y, z, yaw)
        path = [
            (current_x + 0, current_y + 0, current_z, current_yaw),
            (current_x + 200, current_y + 100, current_z, current_yaw + 10),
            (current_x + 400, current_y + 50, current_z, current_yaw + 20),
            (current_x + 600, current_y - 50, current_z, current_yaw - 10),
            (current_x + 800, current_y - 100, current_z, current_yaw - 20),
            (current_x + 1000, current_y + 0, current_z, current_yaw)
        ]

        # Send the path to the car
        logger.info("Sending car path...")
        success = client.send_car_path(car_name, path)

        if success:
            logger.info("Successfully sent car path")
        else:
            logger.error("Failed to send car path")

        # Wait for the car to complete the path
        logger.info("Waiting for car to complete path...")
        time.sleep(10)  # Adjust this based on your environment and the length of the path

        # Get the final pose to verify
        try:
            final_pose = client.get_car_odom(car_name)
            logger.info(f"Final car pose: x={final_pose.pose.x}, y={final_pose.pose.y}, z={final_pose.pose.z}")
        except Exception as e:
            logger.error(f"Could not get final car pose: {e}")

    except Exception as e:
        logger.error(f"Error during test execution: {e}")
        logger.error(traceback.format_exc())
    finally:
        # Close the client connection
        try:
            client.close()
            logger.info("Client connection closed")
        except Exception as e:
            logger.error(f"Error closing client connection: {e}")


if __name__ == "__main__":
    test_send_car_path()