# launch/dwa_pso_planner.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    param_file = os.path.join(
        get_package_share_directory("ros2_dwa_pso"),
        "config/params.yaml"
    )

    node = Node(
        package="ros2_dwa_pso",
        executable="dwa_pso_planner",
        name="dwa_pso_planner",
        output="screen",
        parameters=[param_file]
    )

    return LaunchDescription([node])