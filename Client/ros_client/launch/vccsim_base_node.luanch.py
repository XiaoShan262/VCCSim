from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments for component activation
    rgb_active_arg = DeclareLaunchArgument(
        'rgb_active',
        default_value='true',
        description='Enable RGB camera publishing'
    )
    
    depth_active_arg = DeclareLaunchArgument(
        'depth_active',
        default_value='true',
        description='Enable depth camera publishing'
    )
    
    lidar_active_arg = DeclareLaunchArgument(
        'lidar_active',
        default_value='true',
        description='Enable LiDAR publishing'
    )
    
    drone_pose_active_arg = DeclareLaunchArgument(
        'drone_pose_active',
        default_value='true',
        description='Enable drone pose publishing'
    )
    
    # Declare launch arguments for component frequencies
    rgb_freq_arg = DeclareLaunchArgument(
        'rgb_frequency',
        default_value='5.0',
        description='Frequency (Hz) of RGB camera publishing'
    )
    
    depth_freq_arg = DeclareLaunchArgument(
        'depth_frequency',
        default_value='5.0',
        description='Frequency (Hz) of depth camera publishing'
    )
    
    lidar_freq_arg = DeclareLaunchArgument(
        'lidar_frequency',
        default_value='5.0',
        description='Frequency (Hz) of LiDAR publishing'
    )
    
    drone_pose_freq_arg = DeclareLaunchArgument(
        'drone_pose_frequency',
        default_value='30.0',
        description='Frequency (Hz) of drone pose publishing'
    )
    
    # Declare RViz launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # Configure path to the RViz config file using PathJoinSubstitution
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('ros_client'),
        'config',
        'vccsim_config.rviz'
    ])
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_path,
        description='Path to RViz config file'
    )
    
    # Declare launch arguments for connection parameters
    host_arg = DeclareLaunchArgument(
        'vccsim_host',
        default_value='172.31.178.18',
        description='VCCSim server hostname or IP address'
    )
    
    port_arg = DeclareLaunchArgument(
        'vccsim_port',
        default_value='50996',
        description='VCCSim server port'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='Mavic',
        description='Robot name in VCCSim'
    )
    
    camera_index_arg = DeclareLaunchArgument(
        'rgb_camera_index',
        default_value='0',
        description='RGB camera index'
    )
    
    # Create the node with parameters
    vccsim_node = Node(
        package='ros_client',
        executable='vccsim_node',
        name='vccsim_ros2_node',
        parameters=[{
            # Connection parameters
            'vccsim_host': LaunchConfiguration('vccsim_host'),
            'vccsim_port': LaunchConfiguration('vccsim_port'),
            'robot_name': LaunchConfiguration('robot_name'),
            'rgb_camera_index': LaunchConfiguration('rgb_camera_index'),
            
            # Component activation parameters
            'rgb_active': LaunchConfiguration('rgb_active'),
            'depth_active': LaunchConfiguration('depth_active'),
            'lidar_active': LaunchConfiguration('lidar_active'),
            'drone_pose_active': LaunchConfiguration('drone_pose_active'),
            
            # Frequency parameters
            'rgb_frequency': LaunchConfiguration('rgb_frequency'),
            'depth_frequency': LaunchConfiguration('depth_frequency'),
            'lidar_frequency': LaunchConfiguration('lidar_frequency'),
            'drone_pose_frequency': LaunchConfiguration('drone_pose_frequency'),
        }],
        output='screen'
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        # Component activation arguments
        rgb_active_arg,
        depth_active_arg,
        lidar_active_arg,
        drone_pose_active_arg,
        
        # Frequency arguments
        rgb_freq_arg,
        depth_freq_arg,
        lidar_freq_arg,
        drone_pose_freq_arg,
        
        # RViz arguments
        use_rviz_arg,
        rviz_config_arg,
        
        # Connection arguments
        host_arg,
        port_arg,
        robot_name_arg,
        camera_index_arg,
        
        # Nodes
        vccsim_node,
        rviz_node
    ])