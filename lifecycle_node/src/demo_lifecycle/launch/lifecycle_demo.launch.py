from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node


def generate_launch_description():
  return LaunchDescription([
      LifecycleNode(package='demo_lifecycle', executable='lifecycle_talker',
                    name='lc_talker', namespace='', output='screen'),
      Node(package='demo_lifecycle', executable='listener', output='screen'),
      Node(package='demo_lifecycle', executable='service_client', output='screen')
  ])
