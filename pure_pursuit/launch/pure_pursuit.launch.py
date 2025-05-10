from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'role_name',
            default_value='hero',
            description='Role name of the ego vehicle'
        ),
        DeclareLaunchArgument(
            'path_file',
            default_value='/home/th_ws/src/pure_pursuit/track/250504_pathin_modified.txt',
            description='Relative path file under share/pure_pursuit/path/'
        ),
        # Pure Pursuit Node
        Node(
            package='pure_pursuit',  # 🔄 네가 만든 패키지명으로 변경해
            executable='pure_pursuit',   # 🔄 빌드된 실행 파일 이름
            name='pure_pursuit',
            output='screen',
            parameters=[{
                'role_name': LaunchConfiguration('role_name'),
                'L': 1.6,
                'VL': 4.5,
                'max_lfd': 10.0,
                'min_lfd': 3.0,
                'lfd': 10.0,
                'lfd_param': 100.0,
                'path_file': LaunchConfiguration('path_file')
            }],
            arguments=[
                # 이걸 C++ 노드에서 추가 구현할 경우에만 필요
                # '--ros-args', '--params-file', '/path/to/params.yaml'
            ]
        )
    ])
