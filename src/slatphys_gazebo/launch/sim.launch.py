"""UR3e + gripper + wrist F/T in Gazebo Harmonic, with the probing scene's objects.

ur_simulation_gz's ur_sim_control.launch.py does the arm (robot_state_publisher, gz, spawn,
joint_state_broadcaster, scaled_joint_trajectory_controller) with this package's description
and controllers; on top of it this starts the gripper's effort controller and bridges the wrist
F/T sensor to the topic the real ur_robot_driver publishes.

    ros2 launch slatphys_gazebo sim.launch.py world:=/abs/path/world.sdf            # headless
    ros2 launch slatphys_gazebo sim.launch.py world:=... gui:=true                   # needs DISPLAY
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

WRENCH = "/force_torque_sensor_broadcaster/wrench"


def generate_launch_description():
    share = FindPackageShare("slatphys_gazebo")
    ur = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "launch",
                                                            "ur_sim_control.launch.py"])),
        launch_arguments={
            "ur_type": "ur3e",
            "description_file": PathJoinSubstitution([share, "urdf", "ur3e_hande_gz.urdf.xacro"]),
            "controllers_file": PathJoinSubstitution([share, "config", "controllers.yaml"]),
            "world_file": LaunchConfiguration("world"),
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "gazebo_gui": LaunchConfiguration("gui"),
            "launch_rviz": "false",
        }.items(),
    )
    # The controller manager only exists once the robot is spawned in gz.
    gripper = TimerAction(period=5.0, actions=[Node(
        package="controller_manager", executable="spawner",
        arguments=["gripper_effort_controller", "-c", "/controller_manager", "--controller-manager-timeout", "60"],
        output="screen")])
    ft_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                     arguments=[f"{WRENCH}@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench"], output="screen")
    return LaunchDescription([
        DeclareLaunchArgument("world", description="absolute path of the world .sdf (scripts/make_world.py)"),
        DeclareLaunchArgument("gui", default_value="false"),
        ur, gripper, ft_bridge,
    ])
