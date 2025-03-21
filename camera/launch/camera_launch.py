from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from camera.common_launch import agr_board_type,agr_image_topic

def generate_launch_description():
    node_camera = Node(package='usb_cam',executable='usb_cam_node_exe',name='usb_cam',output='screen',parameters=[{
                'video_device': '/dev/video0',  # 指定摄像头设备
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'yuyv'  # 确保格式支持
            }])
    
    node_calibration = Node(package='camera',executable='camera_calibration_node',name='calibrate_node',parameters=[{
        'image_topic':LaunchConfiguration('image_topic'),
        'Board_type':LaunchConfiguration('Board_type'),
    }])

    node_Extrinsics_publish = Node(package='camera',executable='Extrinsics_publish_node',name='Extrinsics_publish')

    return LaunchDescription([
        agr_board_type,
        agr_image_topic,
        node_camera,
        node_calibration,
        node_Extrinsics_publish]
    )
