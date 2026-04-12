import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_pkg = get_package_share_directory('camera_vision_pkg')
    recognition_pkg = get_package_share_directory('recognition_pkg')
    control_pkg = get_package_share_directory('control_pkg')

    default_recognition_params = os.path.join(
        recognition_pkg,
        'config',
        'recognition_params.face.yaml',
    )
    default_detector_params = os.path.join(
        recognition_pkg,
        'config',
        'target_detector.face.yaml',
    )
    default_control_params = os.path.join(
        control_pkg,
        'config',
        'control_params.yaml',
    )

    camera_launch = os.path.join(camera_pkg, 'launch', 'camera_source.launch.py')
    image_preprocessor_launch = os.path.join(
        recognition_pkg,
        'launch',
        'image_preprocessor.launch.py',
    )
    target_detector_launch = os.path.join(
        recognition_pkg,
        'launch',
        'target_detector.launch.py',
    )
    control_launch = os.path.join(control_pkg, 'launch', 'control_follow.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_camera',
            default_value='true',
            description='Whether to start the camera source pipeline.',
        ),
        DeclareLaunchArgument(
            'enable_recognition',
            default_value='true',
            description='Whether to start the recognition pipeline.',
        ),
        DeclareLaunchArgument(
            'enable_control',
            default_value='true',
            description='Whether to start the control pipeline.',
        ),
        DeclareLaunchArgument(
            'camera_mode',
            default_value='vga',
            description='Camera mode: vga | wide | hd',
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/ros2vision_camera',
            description='Persistent video device path.',
        ),
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='camera',
            description='Camera namespace.',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='front_camera',
            description='Logical camera name.',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='camera_optical_frame',
            description='Frame id for published image.',
        ),
        DeclareLaunchArgument(
            'recognition_params_file',
            default_value=default_recognition_params,
            description='Parameter YAML for image_preprocessor_node.',
        ),
        DeclareLaunchArgument(
            'detector_params_file',
            default_value=default_detector_params,
            description='Parameter YAML for target_detector_node.',
        ),
        DeclareLaunchArgument(
            'control_params_file',
            default_value=default_control_params,
            description='Parameter YAML for control nodes.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            condition=IfCondition(LaunchConfiguration('enable_camera')),
            launch_arguments={
                'mode': LaunchConfiguration('camera_mode'),
                'video_device': LaunchConfiguration('video_device'),
                'namespace': LaunchConfiguration('camera_namespace'),
                'camera_name': LaunchConfiguration('camera_name'),
                'frame_id': LaunchConfiguration('frame_id'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(image_preprocessor_launch),
            condition=IfCondition(LaunchConfiguration('enable_recognition')),
            launch_arguments={
                'params_file': LaunchConfiguration('recognition_params_file'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(target_detector_launch),
            condition=IfCondition(LaunchConfiguration('enable_recognition')),
            launch_arguments={
                'params_file': LaunchConfiguration('detector_params_file'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            condition=IfCondition(LaunchConfiguration('enable_control')),
            launch_arguments={
                'params_file': LaunchConfiguration('control_params_file'),
            }.items(),
        ),
    ])