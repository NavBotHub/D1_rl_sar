import os

from ament_index_python.packages import get_package_share_directory # 查询功能包路径的方法

from launch import LaunchDescription    # launch文件的描述类
from launch_ros.actions import Node     # 节点启动的描述类
from launch.actions import TimerAction

def generate_launch_description():      # 自动生成launch文件的函数
   rviz_config = os.path.join(          # 找到配置文件的完整路径
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
      period=1.5,         # 延时秒数
      actions=[rviz_node] # 到时间后启动的节点
   )
   return LaunchDescription([           # 返回launch文件的描述信息
      Node(
         package='dm_imu',
         executable='dm_imu_node',
         name='dm_imu_node',
         output='screen',
         parameters=[{'port': '/dev/ttyACM0', 'baud': 921600}]
      ),
        
     delayed_rviz
   ])

