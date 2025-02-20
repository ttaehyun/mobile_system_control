from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package path
    package_path = get_package_share_directory('pid_control')
    track_file_path = os.path.join(package_path,'update_odometry25_1_18.csv')

    # Declare the launch description
    return LaunchDescription([
        # Define parameters for the node
        Node(
            package='pid_control',
            executable='PID_control_ex_node',
            name='pid_control',
            output='screen',
            parameters=[
                {'path': track_file_path},  # Pass the track file path
                {'Kp': 2.0},               # Example PID gain parameters
                {'Ki': 0.5},
                {'Kd': 0.1},
                {'accel': 0.4}
            ]
        )
    ])
