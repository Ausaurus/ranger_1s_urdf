import os, xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

robot_model = "2wv3(camera)"
robot_ns = "r1"  # Robot namespace (robot name)
pose = ["-10.57", "-0.02", "0.0", "0.53"]  # Initial robot pose: x,y,z,th
robot_base_color = (
    "0.0 0.0 1.0 0.95"  # Ign and Rviz color of the robot's main body (rgba)
)


def generate_launch_description():
    pkg_name = "robot_urdf"
    pkg_share = get_package_share_directory(pkg_name)

    # CHANGED: Dynamically resolve the models path using the package's share directory
    farm_models_path = os.path.join(pkg_share, "models")

    this_pkg_path = os.path.join(get_package_share_directory("robot_urdf"))

    parent_share_dir = os.path.dirname(pkg_share)

    simu_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    # CHANGED: Removed the duplicate models path entry since farm_models_path now points to it
    set_ign_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            farm_models_path,
            ":",
            os.path.join(pkg_share, "worlds"),
            ":",
            parent_share_dir,
        ],
    )

    set_sdf_path = SetEnvironmentVariable(name="SDF_PATH", value=farm_models_path)

    world_file = os.path.join(pkg_share, "worlds", "black curve curve line", "black curve curve line.sdf")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    open_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(this_pkg_path + "/rviz/ns_robot.rviz")],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -v 4 \"{world_file}\""
        }.items(),  # The -r flag tells it to run (unpaused)
    )

    urdf_file_path = os.path.join(
            get_package_share_directory(pkg_name), "urdf", robot_model + ".urdf"
    )

    with open(urdf_file_path, "r") as infp:
        robot_desc = infp.read()

    pkg_share_dir = get_package_share_directory(pkg_name)
    robot_desc = robot_desc.replace(
        "package://robot_urdf", "file://" + pkg_share_dir
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_desc,
            "-x",
            pose[0],
            "-y",
            pose[1],
            "-z",
            pose[2],
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            pose[3],
            "-name",
            robot_ns,
            "-allow_renaming",
            "false",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{
            "robot_description": robot_desc,
            "use_sim_time": True}],
    )

    # Bridge
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[  # ign topic -t <topic_name> --info
            # "/depth_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
            # "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            # "/depth_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/lidar/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",

            "/lidar_l@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/lidar_l/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
            "/lidar_r@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/lidar_r/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
            
            "/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
        ],
        parameters=[
            {"qos_overrides./model/" + robot_ns + ".subscriber.reliability": "reliable"}
        ],
        output="screen",
        remappings=[  # ign topic -l
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
    
    delay_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )
    
    delay_dd_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[load_dd_controller],
        )
    )
    
    rqt_robot_steering_node = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
        remappings=[("/cmd_vel", "/diff_drive_base_controller/cmd_vel_unstamped")],
    )
    
    rqt_image_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        # remappings=[("/cmd_vel", "/diff_drive_base_controller/cmd_vel_unstamped")],
    )
    
    pid_node = Node (
        package="robot_urdf",
        executable="pid",
        parameters=[{'velocity_topic': '/diff_drive_base_controller/cmd_vel_unstamped'}]
    )
    
    lookahead_node = Node (
        package="rgb_path",
        executable="rgb_path",
    )
    
    angle_error_node = Node (
        package="rgb_path",
        executable="angle_error",
    )
    
    return LaunchDescription(
        [
            simu_time,
            set_ign_path, # both set necessary for world loading 
            set_sdf_path,
            open_rviz,
            gz_sim,
            clock_bridge,
            gz_spawn_entity,
            robot_state_publisher,
            delay_broadcaster,
            delay_dd_controller,
            lookahead_node,
            angle_error_node,
            pid_node,
            rqt_image_view,
            bridge,
        ]
    )
