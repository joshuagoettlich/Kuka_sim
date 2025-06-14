# Copyright 2022 ICube Laboratory, University of Strasbourg
# MODIFIED 2025 for gripper integration
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    # MODIFIED: Default package for the robot description (URDF)
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='iiwa_robotique_moveit',
            description='Description package with robot URDF/xacro files.',
        )
    )
    # MODIFIED: Default URDF/XACRO file
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='iiwa_with_robotiq.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    # ADDED: Argument for the MoveIt configuration package
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_package',
            default_value='iiwa_robotique_moveit',
            description='MoveIt config package for the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint names, useful for multi-robot setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='/',
            description='Namespace of launched nodes.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use simulation (Gazebo) clock if true.',
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    prefix = LaunchConfiguration('prefix')
    namespace = LaunchConfiguration('namespace')
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            # MODIFIED: The new URDF is in the 'urdf' folder
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'config', description_file]
            ),
            ' ',
            'prefix:=',
            prefix,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Get SRDF
    # MODIFIED: The new SRDF is in the MoveIt config package
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "config", "iiwa_with_robotiq.srdf"]
            ),
        ]
    )
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    # Get planning parameters from the new MoveIt config package
    # MODIFIED: All YAML files are now loaded from the 'iiwa_robotique_moveit' package
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    #ompl_planning_config = PathJoinSubstitution(
    #    [FindPackageShare(moveit_config_package), "config", "ompl_planning.yaml"]
    #)

    moveit_controllers = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "moveit_controllers.yaml"]
    )

    # These parameters are generic and can usually stay the same
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # The move_group node uses the updated parameters
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim},
        ],
    )

    # The rviz node also uses the updated parameters
    rviz_config_file = PathJoinSubstitution(
        # MODIFIED: The RViz config is now in the MoveIt package
        [FindPackageShare(moveit_config_package), 'config', 'moveit.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        condition=IfCondition(start_rviz),
    )

    nodes = [
        move_group_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)