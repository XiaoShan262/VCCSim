import random
import sys
import os
import time
import threading
from VCCSim import VCCSimClient

def measure_time(func, *args, **kwargs):
    """Measure the execution time of an API call."""
    start = time.time()
    try:
        result = func(*args, **kwargs)

        return time.time() - start, result
    except Exception as e:
        print(f"Error calling {func.__name__}: {e}")
        return float('inf'), None  # Return a large value if the call fails

def test_platform_api():
    client = VCCSimClient(host="localhost", port=50996)
    api_times = []
    # Device names
    robot_name = "Mavic"  # You can change to "Husky" or "Flash"
    camera_index = 0

    # ====== LidarService ======
    api_times.append(('LidarGetDataCall', measure_time(client.get_lidar_data, robot_name)))
    api_times.append(('LidarGetDataAndOdomCall', measure_time(client.get_lidar_data_and_odom, robot_name)))

    # ====== DepthCameraService ======
    api_times.append(('DepthCameraGetImageDataCall', measure_time(client.get_depth_camera_image_data, robot_name)))
    api_times.append(('DepthCameraGetPointDataCall', measure_time(client.get_depth_camera_point_data, robot_name)))
    api_times.append(('DepthCameraGetImageSizeCall', measure_time(client.get_depth_camera_image_size, robot_name)))

    # ====== RGBCameraService 单个摄像机测试 ======
    api_times.append(('RGBIndexedCameraImageDataCall', measure_time(client.get_rgb_indexed_camera_image_data, robot_name, camera_index)))
    api_times.append(('RGBIndexedCameraImageSizeCall', measure_time(client.get_rgb_indexed_camera_image_size, robot_name, camera_index)))

    print("\nAPI Response Times (in seconds) for non-RGB camera tests:")
    for api, (duration, _) in api_times:
        if duration == float('inf'):
            print(f"{api}: Call Failed")
        else:
            print(f"{api}: {duration:.6f} seconds")

    # ====== RGBCameraService 多摄像机并发测试 ======
    try:
        n = int(input("\n请输入需要测试的摄像机数量: "))
    except Exception as e:
        print("输入无效，默认使用1个摄像机")
        n = 1

    threads = []
    response_times = []
    lock = threading.Lock()

    def rgb_camera_call(cam_index):
        duration, _ = measure_time(client.get_rgb_indexed_camera_image_data, robot_name, cam_index)
        with lock:
            response_times.append(duration)
        print(f"Camera {cam_index}: {duration:.6f} seconds")

    print("\n开始并发调用 RGBIndexedCameraImageDataCall...")
    start_concurrent = time.time()
    for i in range(n):
        t = threading.Thread(target=rgb_camera_call, args=(i-1,))
        threads.append(t)
        t.start()

    for t in threads:
        t.join()
    total_concurrent = time.time() - start_concurrent

    if response_times:
        average_response_time = sum(response_times) / len(response_times)
    else:
        average_response_time = float('inf')

    print("\n==== RGBCamera Concurrent Test Results ====")
    print(f"Total concurrent test time: {total_concurrent:.6f} seconds")
    print(f"Average RGB camera response time: {average_response_time:.6f} seconds")

if __name__ == "__main__":
    test_platform_api()
