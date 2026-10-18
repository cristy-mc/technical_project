import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch Gazebo GUI",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
    )

    pkg_share = FindPackageShare("iiwa_description")

    world_path = PathJoinSubstitution(
        [pkg_share, "gazebo", "worlds", "empty.world"]
    )

    xacro_iiwa = PathJoinSubstitution(
        [pkg_share, "config", "iiwa.config.xacro"]
    )
    
    # ------------------------------------------------------------------
    # fra2mo description
    # ------------------------------------------------------------------

    fra2mo_xacro = PathJoinSubstitution(
        [
            FindPackageShare("ros2_fra2mo"),
            "urdf",
            "fra2mo.urdf.xacro",
        ]
    )


    # ------------------------------------------------------------------
    # Robot descriptions
    # ------------------------------------------------------------------

    robot_description_1 = Command([
        "xacro ",
        xacro_iiwa,
        " prefix:=iiwa1_",
        " namespace:=/iiwa1",
        " robot_name:=iiwa1",
        " controllers_file:=iiwa2_controllers.yaml",
        " runtime_config_package:=iiwa_description",
        " initial_positions_file:=initial_positions.yaml",
        " use_sim:=true",
    ])

    robot_description_2 = Command([
        "xacro ",
        xacro_iiwa,
        " prefix:=iiwa2_",
        " namespace:=/iiwa2",
        " robot_name:=iiwa2",
        " controllers_file:=iiwa2_controllers.yaml",
        " runtime_config_package:=iiwa_description",
        " initial_positions_file:=initial_positions2.yaml",
        " use_sim:=true",
    ])
    
    robot_description_fra2mo = Command([
        "xacro ",
        fra2mo_xacro,
    ])
    
    # ------------------------------------------------------------------
    # Gazebo
    # ------------------------------------------------------------------

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "gz_args": [
                TextSubstitution(text="-r "),
                world_path,
            ],
        }.items(),
    )

    # ------------------------------------------------------------------
    # Robot State Publishers
    # ------------------------------------------------------------------

    rsp1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="iiwa1",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_1},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    rsp2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="iiwa2",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_2},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    rsp_fra2mo = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="fra2mo",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    robot_description_fra2mo,
                    value_type=str
                ),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )
    
    # ------------------------------------------------------------------
    # Spawn Robot 1
    # ------------------------------------------------------------------

    spawn1 = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_iiwa1",
        output="screen",
        arguments=[
            "-string",
            robot_description_1,
            "-entity",
            "iiwa1",
            "-x",
            "0.0",
            "-y",
            "-2",
            "-z",
            "0.4",
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            "1.5708",
            "-filetype",
            "urdf",
            "-unpause",
        ],
    )

    # ------------------------------------------------------------------
    # Spawn Robot 2
    # ------------------------------------------------------------------

    spawn2 = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_iiwa2",
        output="screen",
        arguments=[
            "-string",
            robot_description_2,
            "-entity",
            "iiwa2",
            "-x",
            "-4.8",
            "-y",
            "4",
            "-z",
            "0.4",
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            "0",
            "-filetype",
            "urdf",
            "-unpause",
        ],
    )
    
    # ------------------------------------------------------------------
    # Spawn fra2mo
    # ------------------------------------------------------------------

    spawn_fra2mo = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_fra2mo",
        output="screen",
        arguments=[
            "-string",
            robot_description_fra2mo,
            "-entity",
            "fra2mo",
            "-x",
            "-2.5",
            "-y",
            "-2",
            "-z",
            "0",
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            "1.5708",
            "-filetype",
            "urdf",
            "-unpause",
        ],
    )
    
    odom_tf_fra2mo = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        namespace='fra2mo',
        parameters=[{"use_sim_time": True}],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('/model/fra2mo/odometry', '/model/fra2mo/odometry'),  
        ]
    )
        
    spawn_controller_1 = Node(
        package="controller_manager",
        executable="spawner",
        namespace="iiwa1",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/iiwa1/controller_manager"
        ],
        output="screen",
    )

    spawn_velocity_1 = Node(
        package="controller_manager",
        executable="spawner",
        namespace="iiwa1",
        arguments=[
            "iiwa_arm_controller",
            "-c",
            "/iiwa1/controller_manager",
            "--controller-manager-timeout",
            "30"
        ],
        output="screen",
    )

    spawn_controller_2 = Node(
        package="controller_manager",
        executable="spawner",
        namespace="iiwa2",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/iiwa2/controller_manager"
        ],
        output="screen",
    )

    spawn_velocity_2 = Node(
        package="controller_manager",
        executable="spawner",
        namespace="iiwa2",
        arguments=[
            "iiwa_arm_controller",
            "-c",
            "/iiwa2/controller_manager",
            "--controller-manager-timeout",
            "30"
        ],
        output="screen",
    )
        
    load_iiwa1_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn1,
            on_exit=[
                spawn_controller_1
            ],
        )
    )

    load_iiwa1_velocity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_controller_1,
            on_exit=[
                spawn_velocity_1
            ],
        )
    )

    load_iiwa2_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn2,
            on_exit=[
                spawn_controller_2
            ],
        )
    )

    load_iiwa2_velocity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_controller_2,
            on_exit=[
                spawn_velocity_2
            ],
        )
    )

    bridge_camera_iiwa2 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/iiwa2_camera_ee@sensor_msgs/msg/Image@gz.msgs.Image',
            '/iiwa2_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args',
            '-r',
            '/iiwa2_camera_ee:=/iiwa2/camera_ee',
            '-r',
            '/iiwa2_camera_info:=/iiwa2/camera_info',
        ],
        output='screen'
    )

    # Bridge per i bracci iiwa (senza clock per evitare conflitti)
    bridge_iiwa = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # =========================
            # IIWA1
            # =========================
            '/iiwa1/picker/scissors/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/iiwa1/picker/scalpel/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/iiwa1/picker/forceps/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/iiwa1/picker/scissors/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/iiwa1/picker/scalpel/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/iiwa1/picker/forceps/detach@std_msgs/msg/Empty]ignition.msgs.Empty',

            # =========================
            # IIWA2
            # =========================
            '/iiwa2/picker/scissors/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/iiwa2/picker/scalpel/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/iiwa2/picker/forceps/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/iiwa2/picker/scissors/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/iiwa2/picker/scalpel/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/iiwa2/picker/forceps/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
        ],
        output='screen'
    )        
    

    bridge_fra2mo = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/rover/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rover/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',  
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gui_arg,
        use_sim_time_arg,

        gz_sim_launch,

        rsp1,
        rsp2,
        rsp_fra2mo,

        spawn1,
        spawn2,
        spawn_fra2mo,
        odom_tf_fra2mo,

        load_iiwa1_jsb,
        load_iiwa1_velocity,

        load_iiwa2_jsb,
        load_iiwa2_velocity,

        bridge_camera_iiwa2,
        bridge_iiwa,
        bridge_fra2mo,
    ])
