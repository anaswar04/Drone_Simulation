from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_path = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'my_robot.urdf'
    )
    urdf_path = os.path.abspath(urdf_path)

    return LaunchDescription([
        Node(
            package='drone',
            executable='pose_to_tf_broadcaster',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}]
        )
    ])
