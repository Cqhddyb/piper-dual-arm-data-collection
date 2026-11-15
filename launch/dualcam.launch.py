import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ================= 1. 左臂摄像头 (Left Camera) =================
    left_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace='dabai_left',          # 对应命令行中的 -r __ns:=/dabai_left
        name='usb_cam',                  # 节点名称
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 1280,
            'image_height': 720,
            'framerate': 30.0,
            'brightness': 0,
            'saturation': 64,
            'pixel_format': 'mjpeg2rgb',  # 强制转为彩色 RGB
            'camera_frame_id': 'camera_left_link',
            'camera_name': 'dabai_left'
        }]
    )

    # ================= 2. 右臂摄像头 (Right Camera) =================
    right_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace='dabai_right',         # 对应命令行中的 -r __ns:=/dabai_right
        name='usb_cam',                  
        output='screen',
        parameters=[{
            'video_device': '/dev/video2',
            'image_width': 1280,
            'image_height': 720,
            'framerate': 30.0,
            'brightness': 0,
            'saturation': 64,
            'pixel_format': 'mjpeg2rgb',  # 强制转为彩色 RGB
            'camera_frame_id': 'camera_right_link',
            'camera_name': 'dabai_right'
        }]
    )

    # ================= 3. RealSense 摄像头 (High/Main Camera) =================
    # 相当于在命令行执行: ros2 launch realsense2_camera rs_launch.py
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        # 如果你想给 realsense 传参数（比如分辨率），可以像下面这样加：
        launch_arguments={
            'depth_module.color_profile': '1280x720x30',
            'enable_compressed': 'true',
            'compressed_format': 'jpeg',
            # 关闭深度流
            'enable_depth': 'false',
            # （可选）关闭红外流，避免不必要的数据传输
            'enable_infra': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            }.items(),
    )

    

    # 将所有节点/Launch文件添加到 Launch 描述中并返回
    return LaunchDescription([
        left_cam_node,
        right_cam_node,
        realsense_node
    ])