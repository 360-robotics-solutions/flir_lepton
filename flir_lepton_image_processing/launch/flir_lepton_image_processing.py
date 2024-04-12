from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetCondition
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define launch arguments
    namespace = DeclareLaunchArgument(
        'namespace',
        default_value='flir_lepton_image_processing',
        description='Namespace for the node'
    )
    bag_mode = DeclareLaunchArgument(
        'bag_mode',
        default_value='false',
        description='Set to true to enable bag mode'
    )
    bag_path = DeclareLaunchArgument(
        'path',
        default_value='/home/user/bag4.bag',
        description='Path to the bag file'
    )
    output = DeclareLaunchArgument(
        'output',
        default_value='screen',
        description='Output type'
    )

    # Conditional rosbag player node
    rosbag_player_node = Node(
        package='rosbag2',
        executable='play',
        name='player',
        output=LaunchConfiguration('output'),
        arguments=['--rate', '1.0', '--clock', LaunchConfiguration('path')],
        condition=IfCondition(LaunchConfiguration('bag_mode'))
    )

    # Thermal camera processing node
    thermal_camera_node = Node(
        package='flir_lepton_image_processing',
        executable='thermal_camera_node',
        namespace=LaunchConfiguration('namespace'),
        name='thermal_camera_node',
        output=LaunchConfiguration('output'),
        parameters=[{
            'param_file': Command([
                'ros2', 'param', 'load', 
                '--namespace', LaunchConfiguration('namespace'),
                'flir_lepton_image_processing',
                FindPackageShare('flir_lepton_image_processing') + '/cfg/processing_node_topics.yaml'
            ])
        }]
    )

    # Launch description to execute the nodes
    return LaunchDescription([
        namespace,
        bag_mode,
        bag_path,
        output,
        rosbag_player_node,
        thermal_camera_node
    ])
