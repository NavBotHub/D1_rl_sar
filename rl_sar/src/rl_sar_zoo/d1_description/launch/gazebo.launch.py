import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    package_name = 'd1_description' 
    pkg_path = get_package_share_directory(package_name)

    # ========================================================
    # 1. Launch arguments
    # ========================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')
    
    gui = LaunchConfiguration('gui')
    declare_gui_cmd = DeclareLaunchArgument(
        'gui', default_value='true', description='Set to "false" to run Gazebo headless')

    # ========================================================
    # 2. XACRO/URDF processing
    # ========================================================
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    # Use Command to process the XACRO file
    robot_description_config = Command(['xacro ', xacro_file])
    
    # ========================================================
    # 3. Core ROS 2 nodes
    # ========================================================
    
    # 3.1 Robot state publisher node
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[
    #         {'robot_description': robot_description_config},
    #         {'use_sim_time': use_sim_time}
    #     ]
    # )

    # 3.2 Gazebo startup
    gazebo_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'world': 'empty.world', 'gui': gui, 'verbose': 'false'}.items() 
    )
    
    # 3.3 Model spawn node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
                   '-entity', 'legged_dm',
                   '-z', '0.53'],
        output='screen'
    )

    # 3.4 Controller spawner (uses the default spawner)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output='screen'
    )
    
    robot_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_joint_controller", "-c", "/controller_manager"],
        output='screen'
    )
    
    # ========================================================
    # 4. Launch description (parallel startup)
    # ========================================================
    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time_cmd,
        declare_gui_cmd,

        # Core nodes
        # robot_state_publisher_node,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        robot_joint_controller_spawner,
    ])
