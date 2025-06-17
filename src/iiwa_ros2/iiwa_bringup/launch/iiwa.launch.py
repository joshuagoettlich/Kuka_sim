# Copyright 2022 ICube Laboratory, University of Strasbourg
# Copyright 2024 (modification) Google
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="iiwa_robotique_moveit",
            description='Package with the controller\'s configuration in "config" folder.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="iiwa_robotique_moveit",
            description="Description package with robot URDF/xacro files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="iiwa_with_robotiq.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="iiwa_robotique_moveit",
            description="MoveIt config package for the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="/",
            description="Namespace of launched nodes.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Start robot in Gazebo simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_planning",
            default_value="false",
            description="Start robot with Moveit2 `move_group` planning config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.170.10.2",
            description="Robot IP of FRI interface (only for real hardware).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_port",
            default_value="30200",
            description="Robot port of FRI interface (only for real hardware).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="initial_positions.yaml",
            description="Configuration file of robot initial positions for simulation.",
        )
    )

    # Use OpaqueFunction to resolve launch arguments at runtime
    def launch_setup(context):
        # Initialize Arguments
        runtime_config_package = LaunchConfiguration("runtime_config_package")
        controllers_file = LaunchConfiguration("controllers_file")
        description_package = LaunchConfiguration("description_package")
        description_file = LaunchConfiguration("description_file")
        moveit_config_package = LaunchConfiguration("moveit_config_package")
        prefix = LaunchConfiguration("prefix")
        namespace = LaunchConfiguration("namespace")
        use_sim = LaunchConfiguration("use_sim")
        use_fake_hardware = LaunchConfiguration("use_fake_hardware")
        use_planning = LaunchConfiguration("use_planning")
        start_rviz = LaunchConfiguration("start_rviz")
        robot_ip = LaunchConfiguration("robot_ip")
        robot_port = LaunchConfiguration("robot_port")
        initial_positions_file = LaunchConfiguration("initial_positions_file")

        # Construct full paths to configuration files
        controllers_config_path = PathJoinSubstitution(
            [FindPackageShare(runtime_config_package), "config", controllers_file]
        )
        initial_positions_file_path = PathJoinSubstitution(
            [FindPackageShare(runtime_config_package), "config", initial_positions_file]
        )

        # Get URDF via xacro
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(description_package), "config", description_file]
                ),
                " ", "prefix:=", prefix,
                " ", "use_sim:=", use_sim,
                " ", "use_fake_hardware:=", use_fake_hardware,
                " ", "robot_ip:=", robot_ip,
                " ", "robot_port:=", robot_port,
                " ", "runtime_config_package:=", runtime_config_package,
                " ", "initial_positions_file:=", initial_positions_file_path,
                " ", "controllers_file:=", controllers_config_path,
                " ", "namespace:=", namespace,
            ]
        )
        robot_description = {"robot_description": robot_description_content}

        # Get SRDF via xacro
        robot_description_semantic_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(moveit_config_package), "config", "iiwa_with_robotiq.srdf"]
                ),
                " ", "prefix:=", prefix,
            ]
        )
        robot_description_semantic = {
            "robot_description_semantic": ParameterValue(
                robot_description_semantic_content, value_type=str
            )
        }

        # Get MoveIt parameters as file paths
        kinematics_path = PathJoinSubstitution(
            [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
        )
        ompl_planning_path = PathJoinSubstitution(
            [FindPackageShare(moveit_config_package), "config", "ompl_planning.yaml"]
        )
        
        # Load moveit_controllers.yaml manually
        moveit_controllers_yaml_path = os.path.join(
            get_package_share_directory(LaunchConfiguration('moveit_config_package').perform(context)),
            "config",
            "moveit_controllers.yaml"
        )
        with open(moveit_controllers_yaml_path, 'r') as f:
            moveit_controllers = yaml.safe_load(f)

        # OctoMap parameters
        octomap_config = {
            "octomap_frame": "world",
            "octomap_resolution": 0.02,
            "max_range": 5.0,
        }
        octomap_updater_config = {
            "moveit_simple_controller_manager": {
                "octomap_updater": {
                    "point_cloud_topic": "/world/default/model/camera1/link/link/sensor/depth_camera1/depth_image/points",
                    "filtered_cloud_topic": "filtered_cloud",
                }
            }
        }
        
        # Generic MoveIt parameters
        trajectory_execution = {
            "moveit_manage_controllers": True,
            "trajectory_execution.allowed_execution_duration_scaling": 1.2,
            "trajectory_execution.allowed_goal_duration_margin": 0.5,
            "trajectory_execution.allowed_start_tolerance": 0.01,
        }

        # Parameters for the Planning Scene Monitor
        planning_scene_monitor_parameters = {
            # Basic scene monitor parameters
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            
            # OctoMap parameters
            "octomap_frame": "world",
            "octomap_resolution": 0.02,
            
            # Definition of the 3D sensor to use for the OctoMap
            "sensors": [
                {
                    "sensor_plugin": "occupancy_map_monitor/PointCloudOctomapUpdater",
                    "point_cloud_topic": "/world/default/model/camera1/link/link/sensor/depth_camera1/depth_image/points",
                    "max_range": 5.0,
                    "point_subsample": 1,
                    "padding_offset": 0.1,
                    "padding_scale": 1.0,
                    "filtered_cloud_topic": "filtered_cloud",
                }
            ],
        }

        # Gazebo simulation
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ),
            launch_arguments={
                "gz_args": [
                    "-r ",
                    PathJoinSubstitution(
                        [FindPackageShare(description_package), "worlds", "empty.world"]
                    ),
                    " -v 4",
                ]
            }.items(),
            condition=IfCondition(use_sim),
        )

        # ROS2 control node for real hardware
        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controllers_config_path],
            output="both",
            namespace=namespace,
            condition=UnlessCondition(use_sim),
        )

        # Robot state publisher
        robot_state_pub_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            output="both",
            parameters=[
                robot_description,
                {"use_sim_time": use_sim},  # <-- Add this line
            ],
        )

        # MoveGroup node
        # Path to the new sensor configuration file
        
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            namespace=namespace,
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_path,
                ompl_planning_path,
                moveit_controllers,
                trajectory_execution,
                {"use_sim_time": use_sim},
            ],
            condition=IfCondition(use_planning),
        )

        # Load MoveIt kinematics configuration src/iiwa_robotique_moveit/config/moveit.rviz
        # RViz
        rviz_config_file = PathJoinSubstitution(
            [FindPackageShare(moveit_config_package), "config", "moveit.rviz"]
        )
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_path,
                {"use_sim_time": use_sim},  # <-- Add this line
            ],
            condition=IfCondition(start_rviz),
        )

        # Spawn robot
        spawn_entity = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic", "robot_description",
                "-name", "iiwa14",
                "-allow_renaming", "true",
                "-x", "1",
                "-y", "0",
                "-z", "1.01",
                "-Y", "3.14",
            ],
            output="screen",
            namespace=namespace,
            condition=IfCondition(use_sim),
        )
        
        # Spawn controllers
        def spawn_controllers(controllers):
            return [
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[controller],
                    namespace=namespace,
                    output="screen",
                )
                for controller in controllers
            ]

        controller_spawners = spawn_controllers(
            ["joint_state_broadcaster", "iiwa_controller", "gripper_controller"]
        )
        
        # Camera launch
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare("intel_435"), "launch", "camera.launch.py"])]
            ),
            launch_arguments={"use_sim": use_sim}.items(),
            condition=IfCondition(use_sim),
        )

        # Event handler to launch Gazebo and dependent nodes
        start_sim_handler = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=robot_state_pub_node,
                on_start=[
                    gazebo,
                    camera_launch,
                    spawn_entity,
                ],
            ),
            condition=IfCondition(use_sim),
        )

        # Event handler to launch controllers after the robot is spawned
        start_controllers_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=controller_spawners,
            ),
            condition=IfCondition(use_sim),
        )

        # Event handler to launch MoveIt and RViz after the controllers are running
        start_moveit_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_spawners[-1], # Wait for the last controller to spawn
                on_exit=[
                    move_group_node,
                    rviz_node,
                ],
            ),
            condition=IfCondition(use_sim),
        )

        # Bridge to publish world poses from Gazebo to ROS
        # Bridge to publish world object and camera poses from Gazebo to ROS
        gz_pose_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_pose_bridge',
            arguments=[
                # This topic name MUST match the output from 'ign topic -l'
                '/world/default/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            output='screen'
        )
        
        nodes_to_start = [
            gz_pose_bridge,
            control_node,
            robot_state_pub_node,
            start_sim_handler,
            start_controllers_handler,
            start_moveit_handler,
        ]

        return nodes_to_start

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
