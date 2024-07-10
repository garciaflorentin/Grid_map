import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

octomap_to_gridmap = False

def generate_launch_description():
    # Find the grid_map_demos package share directory
    grid_map_demos_dir = get_package_share_directory('grid_map_demos')

    # Declare launch configuration variables that can access the launch arguments values
    filters_config_file = LaunchConfiguration('filters_config')
    visualization_config_file = LaunchConfiguration('visualization_config')
    rviz_config_file = LaunchConfiguration('rviz_config')


    # Declare launch arguments
    declare_filters_config_file_cmd = DeclareLaunchArgument(
        'filters_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'filters_demo_filter_chain.yaml'),
        description='Full path to the filter chain config file to use')

    declare_visualization_config_file_cmd = DeclareLaunchArgument(
        'visualization_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'filters_demo.yaml'),
        description='Full path to the Gridmap visualization config file to use')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'rviz', 'filters_demo.rviz'),
        description='Full path to the RVIZ config file to use')

    # Declare node actions
    grid_map_filter_demo_node = Node(
        package='grid_map_demos',
        executable='filters_demo',
        name='grid_map_filters',
        output='screen',
        parameters=[filters_config_file]
    )

    grid_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[visualization_config_file]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    ignition_ros2_bridge_tf = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        arguments=['/model/robot/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   '/lidar/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                   '/model/robot/pose@geometry_msgs/msg/TransformStamped@ignition.msgs.Pose',
                   '/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
                ],
        remappings=[
                ('/camera_info', '/image_raw/camera_info'),
                ('/image_raw', '/image_raw/image_raw'),
            ]
    )

    tf_node = Node(
        package='grid_map_demos',
        executable='transform_publisher_node',
        name='transform_publisher_node',
        output='screen',
    )

    tf_static_node = Node(
        package='grid_map_demos',
        executable='tf_static_publisher',
        name='tf_static_publisher',
        output='screen',
    )

    # costmap_fusion = Node(
    #     package='grid_map_demos',
    #     executable='occupancy_grid_fusion',
    #     name='occupancy_grid_fusion',
    #     output='screen',
    # )

    costmap_concat = Node(
        package='grid_map_demos',
        executable='occupancy_grid_concat_node',
        name='occupancy_grid_concat_node',
        output='screen',
    )

    map_value  = Node(
        package='grid_map_demos',
        executable='occupancy_grid_value_node',
        name='occupancy_grid_value_node',
        output='screen',
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    if octomap_to_gridmap : 

        param_file = LaunchConfiguration('param_file')

        declare_param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'octomap_to_gridmap_demo.yaml'),
        description='Full path to the config file to use')

        octomap_to_gridmap_demo_node = Node(
        package='grid_map_demos',
        executable='octomap_to_gridmap_demo',
        name='octomap_to_gridmap_demo',
        output='screen',
        parameters=[param_file] )

        ld.add_action(declare_param_file_cmd)
        ld.add_action(octomap_to_gridmap_demo_node)

    
    # Add launch arguments to the launch description
    ld.add_action(declare_filters_config_file_cmd)
    ld.add_action(declare_visualization_config_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add node actions to the launch description
    ld.add_action(grid_map_filter_demo_node)
    ld.add_action(grid_map_visualization_node)
    ld.add_action(rviz2_node)
    ld.add_action(ignition_ros2_bridge_tf)
    ld.add_action(tf_node)
    ld.add_action(tf_static_node)
    ld.add_action(map_value)
    ld.add_action(costmap_concat)




    return ld
