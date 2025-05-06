import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package path
    package_path = get_package_share_directory('pid_control')
    track_file_path = os.path.join(package_path,'250504_pathin_modified.txt')

    # Declare the launch description
    return launch.LaunchDescription([
        # Define parameters for the node
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle',
            description='Name of the vehicle'
        ),
        launch_ros.actions.Node(
            package='pid_control',
            executable='PID_control_ex_node',
            name='pid_control',
            output='screen',
            parameters=[
                {'path': track_file_path},  # Pass the track file path
                {'Kp': 1.0},               # Example PID gain parameters
                {'Ki': 0.15},
                {'Kd': 0.0},
                #{'accel': 0.4},
                {'role_name': launch.substitutions.LaunchConfiguration('role_name')}
            ]
        )
    ])
