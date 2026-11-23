"""The lab's stack (github.com/alexhjkim/ur3e_experiments README, terminals 1-2) plus a camera:
ur_robot_driver for the UR3e and the ENME480 bridge (ur3e_mrc), which slatphys_real drives
through ur3e_plan_runner's PlanSender.

    ros2 launch slatphys_real bringup.launch.py robot_ip:=$UR_IP
    ros2 launch slatphys_real bringup.launch.py robot_ip:=0.0.0.0 fake_hardware:=true camera:=false   # testing

Then press Play on the pendant's External Control program, and keep the speed slider at <= 30 %
(ur3e_plan_runner refuses to move above that). The bridge waits up to 20 s for the trajectory
controller, so it starts 8 s after the driver.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fake = LaunchConfiguration("fake_hardware")
    ur = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])),
        launch_arguments={"ur_type": "ur3e", "robot_ip": LaunchConfiguration("robot_ip"),
                          "initial_joint_controller": "scaled_joint_trajectory_controller",
                          "use_fake_hardware": fake,   # Humble (the lab's Docker image)
                          "use_mock_hardware": fake,   # Jazzy
                          "launch_rviz": "false"}.items(),
    )
    bridge = TimerAction(period=8.0, actions=[
        Node(package="ur3e_mrc", executable="ur3e_mrc_enme480_ctrl", output="screen"),
        Node(package="ur3e_mrc", executable="ur3e_mrc_enme480_topics", output="screen"),
    ])
    rs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])),
        launch_arguments={"rgb_camera.color_profile": "1280x720x15", "enable_depth": "false"}.items(),
        condition=IfCondition(LaunchConfiguration("camera")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.77.22"),
        DeclareLaunchArgument("fake_hardware", default_value="false"),
        DeclareLaunchArgument("camera", default_value="true", description="start realsense2_camera"),
        ur, bridge, rs,
    ])
