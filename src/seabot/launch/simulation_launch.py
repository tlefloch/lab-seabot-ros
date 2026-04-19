import launch
import launch_ros.actions
from launch_ros.actions import Node

def generate_launch_description():

    simulation = Node(
        package='simulation',
        executable='simulation',
        name='simulation'
    )

    animation = Node(
        package='animation',
        executable='animation',
        name='animation'
    )


    return launch.LaunchDescription([
        simulation,
        animation
  ])