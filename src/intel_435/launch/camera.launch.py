# Copyright 2024 Google
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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation.',
        )
    )

    # Initialize Arguments
    use_sim = LaunchConfiguration('use_sim')

    camera_nodes = []

    # Define the camera configurations.
    camera_configs = [
        {
            'instance_name': 'camera1',
            'x': '0', 'y': '0', 'z': '1.5', 'R': '0', 'P': '0.785398', 'Y': '0'
        },
        {
            'instance_name': 'camera2',
            'x': '0', 'y': '-0.5', 'z': '1.7', 'R': '0', 'P': '0.785398', 'Y': '0.7'
        },
        {
            'instance_name': 'camera3',
            'x': '0', 'y': '0.5', 'z': '1.7', 'R': '0', 'P': '0.785398', 'Y': '-0.7'
        }
    ]

    # The path to your single SDF file
    sdf_file_path = PathJoinSubstitution([
        get_package_share_directory('intel_435'), 'urdf', 'intel_d435.sdf'
    ])

    for config in camera_configs:
        instance_name = config['instance_name']

        # Spawner for each camera
        camera_spawner = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', sdf_file_path,
                '-name', instance_name,  # Unique name for this instance
                '-allow_renaming', 'true',
                '-x', config['x'], '-y', config['y'], '-z', config['z'],
                '-R', config['R'], '-P', config['P'], '-Y', config['Y'],
            ],
            output='screen',
        )
        camera_nodes.append(camera_spawner)

        # Bridge for each camera's topics
        camera_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Color Camera Topics
                f'/world/default/model/{instance_name}/link/link/sensor/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/world/default/model/{instance_name}/link/link/sensor/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

                # Depth Camera Topics
                f'/world/default/model/{instance_name}/link/link/sensor/depth_camera1/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/world/default/model/{instance_name}/link/link/sensor/depth_camera1/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                f'/world/default/model/{instance_name}/link/link/sensor/depth_camera1/depth_image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
            ],
            remappings=[
                (f'/world/default/model/{instance_name}/link/link/sensor/rgbd_camera/image', f'/{instance_name}/color/image_raw'),
                (f'/world/default/model/{instance_name}/link/link/sensor/rgbd_camera/camera_info', f'/{instance_name}/color/camera_info'),
                (f'/world/default/model/{instance_name}/link/link/sensor/depth_camera1/depth_image', f'/{instance_name}/depth/image_raw'),
                (f'/world/default/model/{instance_name}/link/link/sensor/depth_camera1/camera_info', f'/{instance_name}/depth/camera_info'),
                (f'/world/default/model/{instance_name}/link/link/sensor/depth_camera1/depth_image/points', f'/{instance_name}/depth/color/points'),
            ],
            output='screen',
        )
        camera_nodes.append(camera_bridge)

    return LaunchDescription(declared_arguments + camera_nodes)