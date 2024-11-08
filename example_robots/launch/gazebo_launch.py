import os
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    urdf_file_name = 'urdf/planar_r1.urdf'
    rviz_file_name = 'config/default.rviz'
    package_path = get_package_share_directory('example_robots')
    urdf_default = os.path.join(package_path, urdf_file_name)
    rviz_default = os.path.join(package_path, rviz_file_name)

    # Launch Configuration variables
    arg_urdf_model          = LaunchConfiguration('urdf_model')
    arg_rviz_config_file    = LaunchConfiguration('rviz_config_file')
    arg_use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    arg_use_rviz            = LaunchConfiguration('use_rviz')
    arg_use_sim_time        = LaunchConfiguration('use_sim_time', default='True')
    arg_gz_world            = LaunchConfiguration('world')

    # Launch arguments
    declare_urdf_model_path = DeclareLaunchArgument(
        name='urdf_model',
        default_value=urdf_default,
        description='Absolute path to robot urdf file'
    )

    declare_rviz_config_file = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=rviz_default,
        description='Full path to the rviz config'
    )

    declare_use_robot_state_pub = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publsiher'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Enable RVIZ for display or not'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulaiton (Gazebo) clock if true'
    )

    declare_gz_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value='',
        description='Gazeb sim wrold file'
    )

    # gazebo include launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[('gz_args', [arg_gz_world, ' -v 4', ' -r'])]
    )

    robot_description = Command(['xacro ', arg_urdf_model])
    # robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sime_time': arg_use_sim_time, 
                     'robot_description': robot_description}],
        arguments=[urdf_default]
    )

    # spawn robot
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '1.0',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'planar_r1',
                   '-allow_renaming', 'false'],
    )

    # load controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controller'],
        output='screen'
    )

    # rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', arg_rviz_config_file]
    )

    # launch description
    ld = LaunchDescription()

    # add launch declarations
    ld.add_action(declare_urdf_model_path)
    ld.add_action(declare_rviz_config_file)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_gz_world_cmd)

    # add event handlers
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[load_joint_state_controller]
        )
    ))
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_joint_position_controller]
        )
    ))
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gz_spawn_entity)
    ld.add_action(rviz_node)

    return ld

