import launch 
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle',
            description='Name of the vehicle'
        ),
        launch_ros.actions.Node(
            package='mobile_system_control',
            executable='mobile_system_control',
            name='mobile_system_node',
            output='screen',
            parameters=[
                {'role_name': launch.substitutions.LaunchConfiguration('role_name')}
            ]
        )
    ])
