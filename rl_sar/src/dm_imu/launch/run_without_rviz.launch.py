import os

from ament_index_python.packages import get_package_share_directory # 查询功能包路径的方法

from launch import LaunchDescription    # launch文件的描述类
from launch_ros.actions import Node     # 节点启动的描述类


def generate_launch_description():      # 自动生成launch文件的函数


   return LaunchDescription([           # 返回launch文件的描述信息
	Node(
	    package='dm_imu',
	    executable='dm_imu_node',
	    name='dm_imu_node',
	    output='screen',
	    parameters=[{'port': '/dev/ttyACM0', 'baud': 921600}]
	),
        

   ])

