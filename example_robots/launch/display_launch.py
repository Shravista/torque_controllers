import os
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'urdf/r2d2urdf.xml'
    rviz_file_name = 'config/default.rviz'
    package_path = get_package_share_directory('urdf_tutorials')
    urdf_default = os.path.join(package_path, urdf_file_name)
    rviz_default = os.path.join(package_path, rviz_file_name)

    # Launch configuration variables
    gui = LaunchConfiguration('gui')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

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
    
    declare_use_joint_state_publisher = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Flag to enable joint state publisher gui'
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

    # Launch actions

    # joint state publisher
    joint_state_publisher_node = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    # gui for joint state publisher
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # robot state publisher
    robot_state_publisher_node = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sime_time': use_sim_time, 
                     'robot_description': Command(['xacro ', urdf_model])}],
        arguments=[urdf_default]
    )

    # rviz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch options
    ld.add_action(declare_urdf_model_path)
    ld.add_action(declare_rviz_config_file)
    ld.add_action(declare_use_joint_state_publisher)
    ld.add_action(declare_use_robot_state_pub)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # declare launch actions
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld





 