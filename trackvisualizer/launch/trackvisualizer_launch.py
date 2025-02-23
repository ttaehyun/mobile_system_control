import os
import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    trackvisualizer_dir = get_package_share_directory('trackvisualizer')
    
    csv_file_path = os.path.join(trackvisualizer_dir, 'update_odometry25_1_18.csv')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle',
            description='Name of the vehicle'
        ),
        launch_ros.actions.Node(   
            package='trackvisualizer',
            executable='trackvisualizer',
            name='trackvisualizer',
            output='screen',
            parameters=[{'csv_dir':csv_file_path},
                        {'role_name': launch.substitutions.LaunchConfiguration('role_name')}
            ]
        ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            # arguments=['-d',os.path.join(trackvisualizer_dir,'track.rviz')],
        ),
    ])