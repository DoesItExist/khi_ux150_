import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package='khi_ux150_description').find('khi_ux150_description')   
    default_model_path = os.path.join(pkg_share, 'urdf/khi_ux150.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config/display.rviz')
    world_path = os.path.join(pkg_share, 'worlds/empty.sdf')
    # sdf_path = os.path.join(pkg_share, 'models/urdf/khi_ux150/model.sdf')    
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')

    robot_description_content = ParameterValue(Command(['xacro ', default_model_path]), value_type=str)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint_state_publisher_node_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',  # Use create or appropriate executable for spawning
        output='screen',
        arguments=[ '-topic', 'robot_description'
                    '-name', 'khi_ux150',
                    '-allow_renaming', 'true'
                   ]
    )

    ld.add_action(DeclareLaunchArgument(name='jsp_gui', default_value='true', choices=['true', 'false'],
                                        description='Flag to enable joint_state_publisher_gui'))
    ld.add_action(DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                        description='Flag to enable use_sim_time'))
    ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Absolute path to robot urdf file'))
    ld.add_action(DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file'))
    ld.add_action(ExecuteProcess(condition=launch.conditions.IfCondition(use_sim_time), cmd=['gz','sim', 
                                            '--verbose', '-s','libgazebo_ros_init.so', '-s', 
                                            'libgazebo_ros_factory.so', world_path],output='screen'))
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_node_gui)
    ld.add_action(rviz_node)
    ld.add_action(spawn_entity)


    return ld