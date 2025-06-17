#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import time

class TfRelayNode(Node):
    """
    This node subscribes to Gazebo's pose topic, filters for specific models,
    adds a new 'map' frame, corrects headers, and republishes the data as
    valid transforms on the standard /tf topic.
    """
    def __init__(self):
        super().__init__('tf_relay_node')

        # This publisher will publish to the standard /tf topic
        self.tf_publisher = self.create_publisher(
            TFMessage,
            '/tf',
            10
        )

        # A list of the specific child frames we want to publish.
        # This will be used to filter the incoming messages from Gazebo.
        self.target_frames = ['camera1', 'camera2', 'camera3', 'bowl','Threshold_Porcelain_Teapot_White','ACE_Coffee_Mug_Kristen_16_oz_cup']

        # This subscriber listens to the pose topic from your Gazebo simulation
        self.pose_subscription = self.create_subscription(
            TFMessage,
            '/world/default/pose/info',  # The topic you confirmed is working
            self.listener_callback,
            10
        )
        
        self.get_logger().info(
            "TF Relay Node started. Filtering for specific frames and "
            "forwarding to '/tf' with a new 'map' frame."
        )
        self.last_log_time = time.time()

    def listener_callback(self, msg: TFMessage):
        """
        This function is called every time a message is received.
        It adds a static transform for map->world, filters for target frames,
        corrects the header of each transform, and then republishes the
        combined message.
        """
        # Get the current ROS time. We will apply this to all transforms.
        now = self.get_clock().now().to_msg()

        # 1. Create the static transform from 'map' to 'world'
        # This places the world's origin at (-1, 0, -1) in the map's coordinate system.
        map_to_world = TransformStamped()
        map_to_world.header.stamp = now
        map_to_world.header.frame_id = 'map'
        map_to_world.child_frame_id = 'world'
        map_to_world.transform.translation.x = 1.0
        map_to_world.transform.translation.y = 0.0
        map_to_world.transform.translation.z = 1.0
        # No rotation, so identity quaternion
        map_to_world.transform.rotation.x = 0.0
        map_to_world.transform.rotation.y = 0.0
        map_to_world.transform.rotation.z = 1.0
        map_to_world.transform.rotation.w = 0.0

        # Create a new list of transforms to publish, starting with our new map->world transform
        output_transforms = [map_to_world]

        # 2. Iterate through each transform from Gazebo, filter, fix, and add to our list
        for transform in msg.transforms:
            # Check if the transform's child_frame_id contains any of our target names.
            # This is a robust way to match 'bowl' in 'bowl::link', for example.
            if any(target in transform.child_frame_id for target in self.target_frames):
                # Set the timestamp to the current time.
                transform.header.stamp = now
                # Set the parent frame_id to 'world', which is the standard base frame.
                transform.header.frame_id = "map"
                output_transforms.append(transform)

        # 3. Publish the corrected and filtered message as a single TFMessage
        if len(output_transforms) > 1: # Only publish if we found some of our targets
            output_msg = TFMessage(transforms=output_transforms)
            self.tf_publisher.publish(output_msg)

            # Log a message every 5 seconds to show that it's working
            current_time = time.time()
            if (current_time - self.last_log_time > 5.0):
                self.get_logger().info("Relaying filtered TF messages with map->world transform...", throttle_duration_sec=5)
                self.last_log_time = current_time

def main(args=None):
    rclpy.init(args=args)
    tf_relay_node = TfRelayNode()
    try:
        rclpy.spin(tf_relay_node)
    except KeyboardInterrupt:
        pass
    finally:
        tf_relay_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
