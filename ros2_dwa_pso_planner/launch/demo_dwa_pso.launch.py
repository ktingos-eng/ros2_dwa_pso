# launch/dwa_pso_planner.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        ################### SET GOAL HERE ###################
        DeclareLaunchArgument("goal_x", default_value="5.0"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        #####################################################

        DeclareLaunchArgument("controller_step_ms", default_value="100.0"),
        DeclareLaunchArgument("eps_goal",           default_value="1e-2"),

        DeclareLaunchArgument("linear_vel_max",     default_value="10.0"),
        DeclareLaunchArgument("angular_vel_max",    default_value="10.0"),
        DeclareLaunchArgument("linear_acc_max",     default_value="2.0"),
        DeclareLaunchArgument("angular_acc_max",    default_value="1.0"),

        DeclareLaunchArgument("heading_weight",     default_value="1.0"),
        DeclareLaunchArgument("velocity_weight",    default_value="2.0"),
        DeclareLaunchArgument("progress_weight",    default_value="2.5"),
        DeclareLaunchArgument("clearence_weight",    default_value="1.5"),

        DeclareLaunchArgument("reject_oob_trajectories", default_value="False"),

        DeclareLaunchArgument("pso_max_iter",       default_value="60"),
        DeclareLaunchArgument("particles",          default_value="30"),

        DeclareLaunchArgument("eps_head",           default_value="1e-2"),
        DeclareLaunchArgument("eps_cost",           default_value="1e-4"),
        DeclareLaunchArgument("patience",           default_value="5"),
        DeclareLaunchArgument("random_init",        default_value="False"),

        DeclareLaunchArgument("cognitive_coeff",    default_value="2.0"),
        DeclareLaunchArgument("social_coeff",       default_value="2.0"),
        DeclareLaunchArgument("init_inertia_coeff", default_value="0.9"),
        DeclareLaunchArgument("final_inertia_coeff",default_value="0.4"),
    ]

    node = Node(
        package="ros2_dwa_pso",
        executable="dwa_pso_planner",
        name="dwa_pso_planner",
        output="screen",
        parameters=[{
            "goal_x": LaunchConfiguration("goal_x"),
            "goal_y": LaunchConfiguration("goal_y"),

            "dt_ms":      LaunchConfiguration("controller_step_ms"),
            "eps_goal":   LaunchConfiguration("eps_goal"),

            "limits.max_vel.linear":  LaunchConfiguration("linear_vel_max"),
            "limits.max_vel.angular": LaunchConfiguration("angular_vel_max"),
            "limits.max_acc.linear":  LaunchConfiguration("linear_acc_max"),
            "limits.max_acc.angular": LaunchConfiguration("angular_acc_max"),

            "heading_weight": LaunchConfiguration("heading_weight"),
            "velocity_weight": LaunchConfiguration("velocity_weight"),
            "progress_weight": LaunchConfiguration("progress_weight"),
            "clearence_weight": LaunchConfiguration("clearence_weight"),

            "reject_oob_trajectories": LaunchConfiguration("reject_oob_trajectories"),

            "imax":  LaunchConfiguration("pso_max_iter"),
            "n_par": LaunchConfiguration("particles"),

            "eps_head": LaunchConfiguration("eps_head"),
            "eps_cost": LaunchConfiguration("eps_cost"),
            "patience": LaunchConfiguration("patience"),
            "random_int": LaunchConfiguration("random_init"),

            "acc_cog":    LaunchConfiguration("cognitive_coeff"),
            "acc_soc":    LaunchConfiguration("social_coeff"),
            "iner_start": LaunchConfiguration("init_inertia_coeff"),
            "iner_end":   LaunchConfiguration("final_inertia_coeff"),
        }]
    )

    return LaunchDescription(args + [node])