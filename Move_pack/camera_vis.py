#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters

class MultiCameraVisualizer(Node):
    """
    A ROS 2 node to visualize color and depth images from multiple cameras.
    """
    def __init__(self):
        """
        Initializes the MultiCameraVisualizer node.
        """
        super().__init__('multi_camera_visualizer')
        self.bridge = CvBridge()

        # Define camera names
        self.camera_names = ['camera1', 'camera2', 'camera3']
        self.synchronizers = []

        self.get_logger().info("Initializing subscribers for color and depth images...")

        for camera_name in self.camera_names:
            # Subscribers for color image and depth image
            image_sub = message_filters.Subscriber(self, Image, f'/{camera_name}/color/image_raw')
            depth_sub = message_filters.Subscriber(self, Image, f'/{camera_name}/depth/image_raw')

            # Synchronize the color and depth image topics
            ats = message_filters.ApproximateTimeSynchronizer(
                [image_sub, depth_sub],
                queue_size=10,
                slop=0.1  # a 100ms tolerance
            )
            
            # Register a callback that is specific to the camera name
            ats.registerCallback(lambda img, depth, cam_name=camera_name: self.camera_callback(img, depth, cam_name))
            
            # Keep a reference to the synchronizer object
            self.synchronizers.append(ats)

        self.get_logger().info(f"Visualizer started for cameras: {self.camera_names}")

    def camera_callback(self, image_msg, depth_msg, camera_name):
        """
        Callback to process and visualize data from a specific camera.
        """
        try:
            # --- Process Color Image ---
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # --- Process Depth Image ---
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            
            depth_min = 0.0
            depth_max = 5000.0 # Example: 5 meters in mm
            
            # Handle cases where depth is in meters (float) vs. mm (uint16)
            if cv_depth.dtype == np.uint16:
                depth_scaled = cv_depth.astype(np.float32)
            else:
                depth_scaled = cv_depth * 1000 # convert meters to mm
            
            # Replace infinite and NaN values to prevent conversion errors
            depth_scaled[np.isinf(depth_scaled)] = depth_max
            depth_scaled[np.isnan(depth_scaled)] = 0

            # Normalize depth image for visualization and apply a color map
            depth_normalized = np.clip((depth_scaled - depth_min) * 255.0 / (depth_max - depth_min), 0, 255).astype(np.uint8)
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

            # --- Display Images ---
            cv2.imshow(f"Color - {camera_name}", cv_image)
            cv2.imshow(f"Depth - {camera_name}", depth_colormap)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error for {camera_name}: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in callback for {camera_name}: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    visualizer_node = MultiCameraVisualizer()
    
    try:
        rclpy.spin(visualizer_node)
    except KeyboardInterrupt:
        visualizer_node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        # Destroy the node explicitly
        visualizer_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()