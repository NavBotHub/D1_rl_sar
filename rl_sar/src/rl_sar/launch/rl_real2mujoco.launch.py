"""真机 -> MuJoCo 映射校验工具的 launch 文件。

用法示例：
    source install/setup.bash
    ros2 launch rl_sar rl_real2mujoco.launch.py

可选参数：
    port:=/dev/ttyACM0   # IMU 串口设备
    baud:=921600         # IMU 波特率
    robot:=d1          # 机器人名 (rl_sar_zoo/<robot>_description)
    scene:=scene         # mjcf/<scene>.xml

功能：
    - 启动 dm_imu_node 发布 /imu/data
    - 启动 rl_real2mujoco 打开 can1/can2 + 订阅 /imu/data，
      在独立的 MuJoCo 窗口里实时同步机身姿态和 12 个关节位置。
    - 电机以 kp=kd=tau=0 的 MIT 指令保持被动，可用手转动。
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
            # dm_imu 在 declare_parameter("baud", 921600) 中把 baud 声明成 int。
            # LaunchConfiguration 展开是字符串，必须通过 ParameterValue 明确类型
            # 转换，否则节点会因"参数类型不匹配"启动失败，/imu/data 就不会发。
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
