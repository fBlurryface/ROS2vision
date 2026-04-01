import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODE_TO_CONFIG = {
    'vga': 'camera_vga.yaml',
    'wide': 'camera_wide.yaml',
    'hd': 'camera_hd.yaml',
}


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)

    if mode not in MODE_TO_CONFIG:
        raise RuntimeError(
            f'Unsupported mode: {mode}. Supported modes: {list(MODE_TO_CONFIG.keys())}'
        )

    pkg_share = get_package_share_directory('camera_vision_pkg')
    params_file = os.path.join(pkg_share, 'config', MODE_TO_CONFIG[mode])

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace=LaunchConfiguration('namespace'),
        name='camera_source',
        output='screen',
        parameters=[
            params_file,
            {
                'video_device': LaunchConfiguration('video_device'),
                'camera_name': LaunchConfiguration('camera_name'),
                'frame_id': LaunchConfiguration('frame_id'),
            },
        ],
    )

    return [camera_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='hd',
            description='Camera mode: vga | wide | hd',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='camera',
            description='Camera namespace',
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Video device path',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='front_camera',
            description='Logical camera name',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='camera_optical_frame',
            description='Frame id for published image',
        ),
        OpaqueFunction(function=launch_setup),
    ])
