import os

from ament_index_python.packages import get_package_share_directory # Method for querying the package path

from launch import LaunchDescription    # Launch file description class
from launch_ros.actions import Node     # Node launch description class
from launch.actions import TimerAction

def generate_launch_description():      # Function that generates the launch file
   rviz_config = os.path.join(          # Find the full config file path
      get_package_share_directory('dm_imu'),
      'rviz',
      'imu.rviz'
      )
   # rviz_config = '/home/dm/imu_ws/src/dm_imu/rviz/imu.rviz'
   rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      # arguments=['-d', rviz_config],
      output='screen'
   )
   
   delayed_rviz = TimerAction(
      period=1.5,         # Delay in seconds
      actions=[rviz_node] # Node to start after the delay
   )
   return LaunchDescription([           # Return the launch file description
      Node(
         package='dm_imu',
         executable='dm_imu_node',
         name='dm_imu_node',
         output='screen',
         parameters=[{'port': '/dev/ttyACM0', 'baud': 921600}]
      ),
        
     delayed_rviz
   ])

