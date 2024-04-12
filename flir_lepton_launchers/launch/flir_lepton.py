from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ConditionalInclude
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Define launch arguments
    image_processing_arg = DeclareLaunchArgument(
        'image_processing', default_value='true',
        description='Enable or disable image processing'
    )
    gray_image_arg = DeclareLaunchArgument(
        'gray_image', default_value='true',
        description='Enable or disable gray image processing'
    )
    rgb_image_arg = DeclareLaunchArgument(
        'rgb_image', default_value='true',
        description='Enable or disable RGB image processing'
    )

    # Include flir_lepton_sensor launch file
    flir_lepton_sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('flir_lepton_sensor'),
                'launch',
                'flir_lepton_sensor.launch.py'
            ])
        ]),
        launch_arguments={
            'gray_image': LaunchConfiguration('gray_image'),
            'rgb_image': LaunchConfiguration('rgb_image')
        }.items()
    )

    # Conditionally include flir_lepton_image_processing launch file
    flir_lepton_image_processing_launch = ConditionalInclude(
        condition=LaunchConfiguration('image_processing'),
        launch_description_source=PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('flir_lepton_image_processing'),
                'launch',
                'flir_lepton_image_processing.launch.py'
            ])
        ])
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(image_processing_arg)
    ld.add_action(gray_image_arg)
    ld.add_action(rgb_image_arg)

    # Include the other launch files
    ld.add_action(flir_lepton_sensor_launch)
    ld.add_action(flir_lepton_image_processing_launch)

    return ld

