"""Launch file for the real robot -> MuJoCo mapping validation tool.

Usage example:
    source install/setup.bash
    ros2 launch rl_sar rl_real2mujoco.launch.py

Optional arguments:
    port:=/dev/ttyACM0   # IMU serial device
    baud:=921600         # IMU baud rate
    robot:=d1          # Robot name (rl_sar_zoo/<robot>_description)
    scene:=scene         # mjcf/<scene>.xml

Behavior:
    - Start dm_imu_node to publish /imu/data
    - Start rl_real2mujoco to open can1/can2 and subscribe to /imu/data,
      synchronizing base orientation and 12 joint positions in a separate MuJoCo window in real time.
    - Motors stay passive with kp=kd=tau=0 MIT commands and can be rotated by hand.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Serial port for IMU'),
        DeclareLaunchArgument('baud', default_value='921600',
                              description='Baud rate for IMU'),
        DeclareLaunchArgument('robot', default_value='d1',
                              description='Robot name under rl_sar_zoo'),
        DeclareLaunchArgument('scene', default_value='scene',
                              description='mjcf scene file (without .xml)'),

        Node(
            package='dm_imu',
            executable='dm_imu_node',
            name='dm_imu_node',
            output='screen',
            # dm_imu declares baud as an int in declare_parameter("baud", 921600) .
            # LaunchConfiguration expands to a string, so ParameterValue must specify the type
            # conversion; otherwise the node fails to start due to"parameter type mismatch"and /imu/data is not published.
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int),
            }],
        ),

        Node(
            package='rl_sar',
            executable='rl_real2mujoco',
            name='rl_real2mujoco',
            output='screen',
            emulate_tty=True,
            arguments=[
                LaunchConfiguration('robot'),
                LaunchConfiguration('scene'),
            ],
        ),
    ])
