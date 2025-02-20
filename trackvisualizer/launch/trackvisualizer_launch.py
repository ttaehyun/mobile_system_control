import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    trackvisualizer_dir = get_package_share_directory('trackvisualizer')
    
    csv_file_path = os.path.join(trackvisualizer_dir, 'update_odometry25_1_18.csv')

    return LaunchDescription([

        Node(
            package='trackvisualizer',
            executable='trackvisualizer',
            name='trackvisualizer',
            output='screen',
            remappings=[
                ('pose', '/mobile_system_control/ego_vehicle'),
                ('track','/trackvisualizer/track'),
                ('vehicle_arrow','/trackvisualizer/vehicle_arrow'),
                ('vehicle_dot','/trackvisualizer/vehicle_dot'),
            ],
            parameters=[{'csv_dir':csv_file_path}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d',os.path.join(trackvisualizer_dir,'track.rviz')],
        ),
    ])