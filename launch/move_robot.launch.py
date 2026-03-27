import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from pathlib import Path
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    DeclareLaunchArgument
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

robot_ns = "ranger_robot"
world_file = "empty.sdf"


def generate_launch_description():
    pkg_name = "robot_urdf"

    this_pkg_path = os.path.join(get_package_share_directory(pkg_name))

    urdf_file_path = os.path.join(
        get_package_share_directory(pkg_name), "urdf", "2wv3.urdf"
    )

    with open(urdf_file_path, "r") as infp:
        robot_desc = infp.read()
    
    simu_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    # FIXED: Replaced 'vitrox_project' with 'ranger_1s_urdf'
    pkg_share_dir = get_package_share_directory(pkg_name)
    robot_desc = robot_desc.replace(
        "package://robot_urdf", "file://" + pkg_share_dir
    )

    ign_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            os.path.join(this_pkg_path, "worlds"),
            ":" + str(Path(this_pkg_path).parent.resolve()),
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        # Pass a list of tuples, and format the value as a single string
        launch_arguments=[("gz_args", f"-r -v 4 {this_pkg_path}/worlds/{world_file}")],
    )

    open_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(this_pkg_path + "/rviz/ns_robot.rviz")],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        # FIXED: Added use_sim_time
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_desc,
            "-name",
            robot_ns,
            "-allow_renaming",
            "true",
            "-z",
            "0.5",
        ],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_wheel_holder_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "wheel_holder_controller",
        ],
        output="screen",
    )

    load_dd_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "diff_drive_base_controller",
        ],
        output="screen",
    )

    # FIXED: Wait for the robot to spawn before loading the controller
    delay_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )

    # delay_wh_controller = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[load_wheel_holder_controller],
    #     )
    # )

    delay_dd_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_dd_controller],
        )
    )

    rqt_robot_steering_node = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
        remappings=[("/cmd_vel", "/diff_drive_base_controller/cmd_vel_unstamped")],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/"
            + robot_ns
            + "/depth_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
            "/"
            + robot_ns
            + "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            "/"
            + robot_ns
            + "/depth_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
            f"/world/empty/model/{robot_ns}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        parameters=[
            {"qos_overrides./model/" + robot_ns + ".subscriber.reliability": "reliable"}
        ],
        output="screen",
        remappings=[
            (
                "/world/empty/model/" + robot_ns + "/joint_state",
                "/joint_states",
            ),
            ],
    )

    return LaunchDescription(
        [
            simu_time,
            clock_bridge,
            ign_resource_path,
            gazebo,
            robot_state_publisher,
            spawn_entity,
            delay_broadcaster,  # <-- Replaced the raw ExecuteProcess with the delayed version
            # delay_wh_controller,
            delay_dd_controller,
            rqt_robot_steering_node,
            # bridge,
            open_rviz,
        ]
    )
