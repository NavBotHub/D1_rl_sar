from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    dm_imu_node = Node(
        package='dm_imu',
        executable='dm_imu_node',
        name='dm_imu_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
        }],
    )

    rl_real_node = Node(
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
            'dog_usb_l1_exit_timeout_ms': ParameterValue(LaunchConfiguration('dog_usb_l1_exit_timeout_ms'), value_type=int),
            'keyboard_enable': ParameterValue(LaunchConfiguration('keyboard_enable'), value_type=bool),
            'sys_joystick_device': LaunchConfiguration('sys_joystick_device'),
        }],
    )

    return LaunchDescription([
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
            default_value='true',
            description='Get down then exit rl_real_d1 when DOG_CTRL L1 changes from ON to OFF'
        ),
        DeclareLaunchArgument(
            'dog_usb_l1_button_bit',
            default_value='8',
            description='DOG_CTRL L1 bit in buttons field'
        ),
        DeclareLaunchArgument(
            'dog_usb_l1_exit_timeout_ms',
            default_value='8000',
            description='Timeout for DOG_CTRL L1 OFF getdown then exit flow in milliseconds'
        ),
        DeclareLaunchArgument(
            'keyboard_enable',
            default_value='false',
            description='Enable stdin keyboard control loop for rl_real_d1'
        ),
        DeclareLaunchArgument(
            'sys_joystick_device',
            default_value='/dev/input/js0',
            description='Linux joystick device path for rl_real_d1'
        ),

        dm_imu_node,
        rl_real_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=rl_real_node,
                on_exit=[Shutdown(reason='rl_real_d1 exited')],
            )
        ),
    ])
