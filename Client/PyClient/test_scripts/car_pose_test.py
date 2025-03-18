import logging
import time
import traceback
from VCCSim import VCCSimClient

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_send_car_pose():
    """Test the send_car_pose method of VCCSimClient."""

    try:
        # Initialize the client
        logger.info("Initializing VCCSimClient...")
        client = VCCSimClient(host="localhost", port=50996)

        # Car name
        car_name = "Husky"

        # Get the current car pose (to determine the starting point)
        try:
            current_pose = client.get_car_odom(car_name)
            logger.info(f"Raw current_pose: {current_pose}")
            logger.info(f"Raw current_pose: {current_pose.twist}")

            current_x = current_pose.pose.x
            current_y = current_pose.pose.y
            current_z = current_pose.pose.z
            current_yaw = 0.0  # default yaw (or compute if available)

            logger.info(f"Current car pose: x={current_x}, y={current_y}, z={current_z}, yaw={current_yaw}")

        except Exception as e:
            logger.warning(f"Could not get current car pose: {e}")
            logger.info("Using default starting pose")
            current_x, current_y, current_z = 0.0, 0.0, 0.0
            current_yaw = 0.0

        # Define a new pose for the car
        target_x = current_x
        target_y = current_y + 1000.0
        target_z = current_z
        target_yaw = current_yaw+20

        # Send the new pose to the car
        logger.info("Sending car pose...")
        success = client.send_car_pose(car_name, target_x, target_y, target_z, target_yaw)

        if success:
            logger.info("Successfully sent car pose")
        else:
            logger.error("Failed to send car pose")

        # Wait for the car to reach the new pose
        logger.info("Waiting for car to reach new pose...")
        time.sleep(5)

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
        try:
            client.close()
            logger.info("Client connection closed")
        except Exception as e:
            logger.error(f"Error closing client connection: {e}")


if __name__ == "__main__":
    test_send_car_pose()
