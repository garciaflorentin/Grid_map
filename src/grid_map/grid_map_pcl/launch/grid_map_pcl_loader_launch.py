from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    node_params = [
        {'folder_path': '/home/fgarcia/ground_pc2_pcds'},
        {'pcd_filename': '1581791878-0.pcd'},
        {'map_rosbag_topic': 'grid_map'},
        {'output_grid_map': 'map.bag'},
        {'map_frame': 'odom'},
        {'map_layer_name': 'elevation'},
        {'prefix': ''},
        {'set_verbosity_to_debug': False}
    ]

    pcl_loader_node = Node(
        package='grid_map_pcl',
        executable='grid_map_pcl_loader_node',
        name='grid_map_pcl_loader_node',
        output='screen',
        parameters=node_params
    )

    ld = LaunchDescription()

    ld.add_action(pcl_loader_node)

    return ld
