from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='flir_lepton_sensor',
        description='Namespace for the flir_lepton_sensor node.')

    gray_image_arg = DeclareLaunchArgument(
        'gray_image',
        default_value='true',
        description='Enable gray image processing.')

    rgb_image_arg = DeclareLaunchArgument(
        'rgb_image',
        default_value='true',
        description='Enable RGB image processing.')

    # Node configuration
    flir_lepton_sensor_node = Node(
        package='flir_lepton_sensor',
        executable='flir_lepton_sensor_node',
        namespace=LaunchConfiguration('namespace'),
        name='flir_lepton_sensor',
        output='screen',
        parameters=[{
            'gray_image': LaunchConfiguration('gray_image'),
            'rgb_image': LaunchConfiguration('rgb_image'),
            os.path.join(
                get_package_share_directory('flir_lepton_sensor'), 
                'config', 'params.yaml')
        }]
    )

    # Launch description
    return LaunchDescription([
        namespace_arg,
        gray_image_arg,
        rgb_image_arg,
        flir_lepton_sensor_node
    ])
