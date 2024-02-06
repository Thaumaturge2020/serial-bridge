from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rmcv_bridge",
            executable="bridge",
            name="rmcv_bridge_1",
            output="screen",
            emulate_tty=True
        ),
        Node(
            package="serial_detector",
            executable="talker",
            name="serial_detector_1",
            output="screen",
            emulate_tty=True
        )
    ])