import os

from ament_index_python.packages import get_package_share_directory # Method for querying the package path

from launch import LaunchDescription    # Launch file description class
from launch_ros.actions import Node     # Node launch description class


def generate_launch_description():      # Function that generates the launch file


   return LaunchDescription([           # Return the launch file description
	Node(
	    package='dm_imu',
	    executable='dm_imu_node',
	    name='dm_imu_node',
	    output='screen',
	    parameters=[{'port': '/dev/ttyACM0', 'baud': 921600}]
	),
        

   ])

