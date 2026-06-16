from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明 launch 参数
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port for IMU'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='921600',
            description='Baud rate for IMU'
        ),
        DeclareLaunchArgument(
            'dog_usb_enable',
            default_value='false',
            description='Enable dog ESP32-S3 USB DOG_CTRL receiver'
        ),
        DeclareLaunchArgument(
            'dog_usb_device',
            default_value='',
            description='Dog ESP32-S3 USB serial device path'
        ),
        DeclareLaunchArgument(
            'dog_usb_baud',
            default_value='115200',
            description='Dog ESP32-S3 USB serial baud rate'
        ),
        DeclareLaunchArgument(
            'dog_usb_timeout_ms',
            default_value='300',
            description='Dog USB valid frame timeout in milliseconds'
        ),
        DeclareLaunchArgument(
            'dog_remote_timeout_ms',
            default_value='1500',
            description='LoRa remote packet age timeout in milliseconds'
        ),
        DeclareLaunchArgument(
            'dog_usb_allow_fallback',
            default_value='false',
            description='Allow joystick/keyboard fallback after dog USB timeout'
        ),
        DeclareLaunchArgument(
            'dog_usb_l1_off_exit',
            default_value='false',
            description='Exit rl_real_d1 when DOG_CTRL L1 changes from ON to OFF'
        ),
        DeclareLaunchArgument(
            'dog_usb_l1_button_bit',
            default_value='8',
            description='DOG_CTRL L1 bit in buttons field'
        ),
        DeclareLaunchArgument(
            'keyboard_enable',
            default_value='true',
            description='Enable stdin keyboard control loop for rl_real_d1'
        ),
        DeclareLaunchArgument(
            'sys_joystick_device',
            default_value='/dev/input/js0',
            description='Linux joystick device path for rl_real_d1'
        ),
        
        # dm_imu 节点
        Node(
            package='dm_imu',
            executable='dm_imu_node',
            name='dm_imu_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
            }],
            # required=True 在 ROS 2 中不是直接参数
            # 可以通过 on_exit 行为实现类似功能
        ),
        
        # rl_real_d1 节点
        # 必须用 xterm 包一层。原因：
        #   1. ros2 launch 启动子进程时 stdin 是 pipe，不是 TTY；
        #   2. KeyboardInterface 里的 tcgetattr / read(STDIN) 需要 TTY；
        #   3. 不包 xterm 会导致键盘按键完全没反应（手柄不受影响）。
        # 加 -hold 让 xterm 窗口在程序退出后保留，方便看错误信息。
        Node(
            package='rl_sar',
            executable='rl_real_d1',
            name='rl_real_d1',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'dog_usb_enable': ParameterValue(LaunchConfiguration('dog_usb_enable'), value_type=bool),
                'dog_usb_device': LaunchConfiguration('dog_usb_device'),
                'dog_usb_baud': ParameterValue(LaunchConfiguration('dog_usb_baud'), value_type=int),
                'dog_usb_timeout_ms': ParameterValue(LaunchConfiguration('dog_usb_timeout_ms'), value_type=int),
                'dog_remote_timeout_ms': ParameterValue(LaunchConfiguration('dog_remote_timeout_ms'), value_type=int),
                'dog_usb_allow_fallback': ParameterValue(LaunchConfiguration('dog_usb_allow_fallback'), value_type=bool),
                'dog_usb_l1_off_exit': ParameterValue(LaunchConfiguration('dog_usb_l1_off_exit'), value_type=bool),
                'dog_usb_l1_button_bit': ParameterValue(LaunchConfiguration('dog_usb_l1_button_bit'), value_type=int),
                'keyboard_enable': ParameterValue(LaunchConfiguration('keyboard_enable'), value_type=bool),
                'sys_joystick_device': LaunchConfiguration('sys_joystick_device'),
            }],
            prefix=['xterm -hold -e'],
            shell=True,
        ),
    ])
