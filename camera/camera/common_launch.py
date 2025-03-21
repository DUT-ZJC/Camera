from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals

agr_board_type = DeclareLaunchArgument('Board_type',choices=["Chessboard","Arcuo"],description='type of Board')
agr_image_topic = DeclareLaunchArgument('image_topic',default_value='/iamge_raw')

is_chess = LaunchConfigurationEquals('Board_type', 'Chessboard')