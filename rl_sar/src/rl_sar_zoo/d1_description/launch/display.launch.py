from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Get the directory containing the current launch file
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    # Get the package root directory (parent of the launch directory)
    package_dir = os.path.dirname(launch_dir)
    
    urdf_path = os.path.join(package_dir, 'urdf', 'd1_description.urdf')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'rviz.rviz')

    # Read URDF file contents
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }],
        output='screen'
    )
    
    # Joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
