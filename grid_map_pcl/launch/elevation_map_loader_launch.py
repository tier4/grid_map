from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    node_params = [
        {'folder_path': '/home/kosuke55/autoware.proj.eva/src/vendor/grid_map/grid_map_pcl/test/test_data'},
        # {'pcd_filename': 'plane_noisy.pcd'},
        {'pcd_filename': 'inside_centerized.pcd'},
        {'map_rosbag_topic': 'grid_map'},
        {'output_grid_map': 'elevation_map.bag'},
        {'map_frame': 'map'},
        {'map_layer_name': 'elevation'},
        {'prefix': ''},
        {'set_verbosity_to_debug': False}
    ]

    pcl_loader_node = Node(
        package='grid_map_pcl',
        executable='elevation_map_loader',
        name='elevation_map_loader_node',
        output='screen',
        parameters=node_params
    )

    ld = LaunchDescription()

    ld.add_action(pcl_loader_node)

    return ld
