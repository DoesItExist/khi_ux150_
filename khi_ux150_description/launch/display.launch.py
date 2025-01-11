import launch
import launch_ros
from launch.substitutions import Command, LaunchConfiguration
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='khi_ux150_description').find('khi_ux150_description')   
    default_model_path = os.path.join(pkg_share, 'urdf/khi_ux150.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config/display.rviz')
    world_path = os.path.join(pkg_share, 'worlds/room.sdf')
    
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    spawn_entity = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',  # Use create or appropriate executable for spawning
        arguments=['-entity', 'khi_ux150', '-topic', 'robot_description'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                              description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                              description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                              description='Absolute path to rviz config file'),
        spawn_entity,
        rviz_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
    ])