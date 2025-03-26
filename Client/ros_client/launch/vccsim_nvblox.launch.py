from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    # Arguments from Nvblox launch file
    args = lu.ArgumentContainer()
    args.add_arg(
        'log_level', 'warn', choices=['debug', 'info', 'warn'], cli=True)
    args.add_arg(
        'mode',
        NvbloxMode.static,
        choices=NvbloxMode.names(),
        description='The nvblox mode.',
        cli=True)
    args.add_arg(
        'num_cameras',
        '1',  # Changed to string to match choices type
        choices=['0', '1', '3'],
        description='Number of cameras that should be used for 3d reconstruction',
        cli=True)
    args.add_arg(
        'lidar', False, description='Whether to use 3d lidar for 3d reconstruction', cli=True)
    args.add_arg(
        'navigation',
        False,
        description='Whether to enable nav2 for navigation.',
        cli=True)
    
    # Define topic names directly with descriptive variable names
    camera0_color_topic = '/camera_0/color/image'
    camera0_color_info_topic = '/camera_0/color/camera_info'
    camera0_depth_topic = '/camera_0/depth/image'
    camera0_depth_info_topic = '/camera_0/depth/camera_info'
    pointcloud_topic = '/pointcloud'
    pose_topic = '/pose'
    
    # VCCSim parameters - from vccsim launch file
    vccsim_args = [
        # Component activation arguments
        DeclareLaunchArgument(
            'rgb_active',
            default_value='true',
            description='Enable RGB camera publishing'
        ),
        DeclareLaunchArgument(
            'depth_active',
            default_value='true',
            description='Enable depth camera publishing'
        ),
        DeclareLaunchArgument(
            'lidar_active',
            default_value='true',
            description='Enable LiDAR publishing'
        ),
        DeclareLaunchArgument(
            'drone_pose_active',
            default_value='true',
            description='Enable drone pose publishing'
        ),
        
        # Frequency arguments
        DeclareLaunchArgument(
            'rgb_frequency',
            default_value='5.0',
            description='Frequency (Hz) of RGB camera publishing'
        ),
        DeclareLaunchArgument(
            'depth_frequency',
            default_value='5.0',
            description='Frequency (Hz) of depth camera publishing'
        ),
        DeclareLaunchArgument(
            'lidar_frequency',
            default_value='5.0',
            description='Frequency (Hz) of LiDAR publishing'
        ),
        DeclareLaunchArgument(
            'drone_pose_frequency',
            default_value='30.0',
            description='Frequency (Hz) of drone pose publishing'
        ),
        
        # RViz arguments
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('ros_client'),
                'config',
                'vccsim_nvblox.rviz'
            ]),
            description='Path to RViz config file'
        ),
        
        # Connection arguments
        DeclareLaunchArgument(
            'vccsim_host',
            default_value='172.31.178.18',
            description='VCCSim server hostname or IP address'
        ),
        DeclareLaunchArgument(
            'vccsim_port',
            default_value='50996',
            description='VCCSim server port'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='Mavic',
            description='Robot name in VCCSim'
        ),
        DeclareLaunchArgument(
            'rgb_camera_index',
            default_value='0',
            description='RGB camera index'
        ),
    ]
    
    # Get launch actions from nvblox arguments
    actions = args.get_launch_actions()
    
    # Add vccsim arguments to actions
    actions.extend(vccsim_args)

    # Globally set use_sim_time
    actions.append(SetParameter('use_sim_time', True))

    # Navigation
    # NOTE: needs to be called before the component container because it modifies params globally
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/navigation/nvblox_carter_navigation.launch.py',
            launch_arguments={
                'container_name': NVBLOX_CONTAINER_NAME,
                'mode': args.mode,
            },
            condition=IfCondition(lu.is_true(args.navigation))))

    # Create a composite container
    actions.append(
        lu.component_container(
            NVBLOX_CONTAINER_NAME, container_type='isolated', log_level=args.log_level))

    # People segmentation
    actions.append(
        lu.include(
            'semantic_label_conversion',
            'launch/semantic_label_conversion.launch.py',
            condition=IfCondition(lu.has_substring(args.mode, NvbloxMode.people_segmentation))))

    # Load nvblox configuration
    base_config = lu.get_path('nvblox_examples_bringup', 'config/nvblox/nvblox_base.yaml')
    realsense_config = lu.get_path('ros_client', 'config/nvblox/nvblox_sim.yaml')

    # Determine mode config
    if args.mode == NvbloxMode.static.name:
        mode_config = {}
    elif args.mode == NvbloxMode.people_segmentation.name:
        segmentation_config = lu.get_path('nvblox_examples_bringup',
                                         'config/nvblox/specializations/nvblox_segmentation.yaml')
        mode_config = segmentation_config
    elif args.mode == NvbloxMode.people_detection.name:
        detection_config = lu.get_path('nvblox_examples_bringup',
                                      'config/nvblox/specializations/nvblox_detection.yaml')
        mode_config = detection_config
    elif args.mode == NvbloxMode.dynamic.name:
        dynamics_config = lu.get_path('nvblox_examples_bringup',
                                     'config/nvblox/specializations/nvblox_dynamics.yaml')
        mode_config = dynamics_config
    else:
        mode_config = {}
        
    # Log debug information
    actions.append(
        LogInfo(msg=["Starting nvblox with remapped topics from VCCSim"])
    )

    # Parameters for nvblox node
    parameters = []
    parameters.append(base_config)
    parameters.append(mode_config)
    parameters.append(realsense_config)
    
    # FIX: Use LaunchConfiguration for num_cameras rather than int conversion
    parameters.append({'num_cameras': LaunchConfiguration('num_cameras')})
    parameters.append({'use_lidar': lu.is_true(args.lidar)})
    parameters.append({'use_tf_transforms': True})  # Use the pose/transform topics instead of TF
    
    remappings = []
    remappings.append(('camera_0/depth/image', camera0_depth_topic))
    remappings.append(('camera_0/depth/camera_info', camera0_depth_info_topic))
    remappings.append(('camera_0/color/image', camera0_color_topic))
    remappings.append(('camera_0/color/camera_info', camera0_color_info_topic))
    remappings.append(('pose', pose_topic))
    
    # Create and load nvblox node with remappings
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters,
    )
    
    # Load the nvblox node into the container
    actions.append(lu.load_composable_nodes(NVBLOX_CONTAINER_NAME, [nvblox_node]))

    # VCCSim node (replacing rosbag playback)
    # Set topic names to match what nvblox expects
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
            
            # Topic name parameters - match what nvblox expects
            'rgb_topic': camera0_color_topic,
            'rgb_info_topic': camera0_color_info_topic,
            'depth_topic': camera0_depth_topic,
            'depth_info_topic': camera0_depth_info_topic,
            'lidar_topic': pointcloud_topic,
            'pose_topic': pose_topic,
            
            # Set use_sim_time to true for compatibility with nvblox
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # Add vccsim node to actions
    actions.append(vccsim_node)

    map_to_odom_tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        # condition=IfCondition(lu.is_true(args.navigation))
    )
    actions.append(map_to_odom_tf_broadcaster)
    
    base_link_to_odom_tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_odom_tf_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': True}],
    )
    actions.append(base_link_to_odom_tf_broadcaster)
    
    # Launch RViz directly with the provided configuration
    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen'
        )
    )

    return LaunchDescription(actions)