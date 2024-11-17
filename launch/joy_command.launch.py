from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_command_node',
            executable='joy_command_node',
            name='joy_command_node',
            output='screen',
            parameters=[
                # 必要に応じてパラメータを設定
                # {"param_name": "param_value"}
            ]
        )
    ])
