from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Node that starts immediately
        Node(
            package='sensors',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen'
        ),

        # Node that starts after 3 seconds to allow imu to calibrate
        TimerAction(
            period=3.0,  # delay in seconds
            actions=[
                Node(
                    package='foxglove_bridge',
                    executable='foxglove_bridge',
                    name='foxglove_bridge',
                    output='screen'
                ),
                Node(
                    package='sensors',
                    executable='sonar_publisher',
                    name='sonar_publisher',
                    output='screen'
                ),
                Node(
                    package='slam',
                    executable='slam_publisher',
                    name='slam_publisher',
                    output='screen'
                )
            ]
        ),
    ])