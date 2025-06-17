import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Point  # Used for XYZ position
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
import threading

# --- Constants ---
CONTROLLER_JOINT_NAMES = [
    'joint_a1', 'joint_a2', 'joint_a3', 'joint_a4',
    'joint_a5', 'joint_a6', 'joint_a7'
]
EE_FRAME_NAME = "tool0"

class RobotStatePublisher(Node):
    """
    Publishes the current XYZ position of the robot's end-effector.
    """
    def __init__(self, robot_model_loader):
        super().__init__('robot_state_publisher')
        self.robot = None
        self.model_loaded = robot_model_loader.model_loaded
        self.robot_loader = robot_model_loader

        # --- Publisher for the current robot position ---
        self.xyz_publisher = self.create_publisher(Point, '/robot/current_xyz', 10)

        # --- Subscriber for joint states ---
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        self.get_logger().info('Robot state publisher started.')
        # Wait for the model to be loaded
        while not self.robot_loader.model_loaded.is_set():
            self.get_logger().info('State publisher waiting for robot model...')
            rclpy.spin_once(self.robot_loader, timeout_sec=1.0)
        self.robot = self.robot_loader.robot
        self.get_logger().info('Robot model loaded in state publisher.')

    def joint_state_callback(self, msg):
        """
        Callback for joint state updates. Performs forward kinematics
        and publishes the end-effector's current XYZ position.
        """
        if not self.robot:
            return

        q = self._get_current_configuration(msg)
        
        # Perform forward kinematics
        pin.forwardKinematics(self.robot.model, self.robot.data, q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)

        # Get the end-effector frame ID
        try:
            frame_id = self.robot.model.getFrameId(EE_FRAME_NAME)
            current_pose = self.robot.data.oMf[frame_id]
            
            # Publish the position
            position_msg = Point()
            position_msg.x = current_pose.translation[0]
            position_msg.y = current_pose.translation[1]
            position_msg.z = current_pose.translation[2]
            self.xyz_publisher.publish(position_msg)

        except IndexError:
            self.get_logger().error(f"End-effector frame '{EE_FRAME_NAME}' not found.")

    def _get_current_configuration(self, joint_state_msg):
        """Helper to get the full configuration vector 'q' from joint_states."""
        q = pin.neutral(self.robot.model)
        joint_positions_map = dict(zip(joint_state_msg.name, joint_state_msg.position))
        for j_name, j_model in zip(self.robot.model.names[1:], self.robot.model.joints[1:]):
            if j_name in joint_positions_map:
                q_slice = slice(j_model.idx_q, j_model.idx_q + j_model.nq)
                q[q_slice] = joint_positions_map[j_name]
        return q


# Add this import at the top of your file
from geometry_msgs.msg import Pose

class MotionPlanner(Node):
    """
    Subscribes to a desired Pose (position + orientation), calculates the IK, 
    and moves the robot.
    """
    def __init__(self, robot_model_loader):
        super().__init__('motion_planner')
        self.robot = None
        self.current_joint_state = None
        self.robot_loader = robot_model_loader
        self.ik_running = threading.Lock()

        # --- MODIFIED SUBSCRIBER ---
        # We now subscribe to a 'Pose' message to get position AND orientation.
        self.desired_pose_subscriber = self.create_subscription(
            Pose,
            '/robot/desired_pose', # New topic name
            self.desired_pose_callback,
            10)

        self.joint_state_subscriber = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, '/iiwa_controller/joint_trajectory', 10)

        self.get_logger().info('Motion planner node started. Waiting for desired Pose...')
        while not self.robot_loader.model_loaded.is_set():
            self.get_logger().info('Motion planner waiting for robot model...')
            rclpy.spin_once(self.robot_loader, timeout_sec=1.0)
        self.robot = self.robot_loader.robot
        self.get_logger().info('Robot model loaded in motion planner.')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def desired_pose_callback(self, msg):
        """
        Callback for receiving a desired Pose. This triggers
        the inverse kinematics calculation and robot motion.
        """
        if self.ik_running.locked():
            self.get_logger().warn("Motion is already in progress. Ignoring new request.")
            return
            
        if not self.robot or self.current_joint_state is None:
            self.get_logger().warn("Robot model or current joint state not available yet.")
            return

        with self.ik_running:
            self.get_logger().info(f"Received desired pose request.")

            # --- Extract Position and Orientation from the Pose message ---
            position = np.array([msg.position.x, msg.position.y, msg.position.z])
            
            # Create a Pinocchio Quaternion. 
            # IMPORTANT: ROS quaternion is (x,y,z,w) but Pinocchio is (w,x,y,z).
            quat_pin = pin.Quaternion(msg.orientation.w, 
                                      msg.orientation.x, 
                                      msg.orientation.y, 
                                      msg.orientation.z)
            
            # Convert quaternion to a rotation matrix
            rotation_matrix = quat_pin.toRotationMatrix()

            # Create the final SE3 target pose for Pinocchio
            target_pose = pin.SE3(rotation_matrix, position)
            
            q_solution = self._calculate_ik(target_pose)

            if q_solution is not None:
                self.publish_trajectory(q_solution)

    # --- The rest of the class (_calculate_ik, _get_current_configuration, publish_trajectory) ---
    # --- remains unchanged. You can copy it from your previous working version.    ---
    def _calculate_ik(self, target_pose):
        """Performs the inverse kinematics calculation."""
        q0 = self._get_current_configuration(self.current_joint_state)
        
        try:
            frame_id = self.robot.model.getFrameId(EE_FRAME_NAME)
        except IndexError:
             self.get_logger().error(f"End-effector frame '{EE_FRAME_NAME}' not found.")
             return None

        q = q0
        eps = 1e-4
        IT_MAX = 1000
        damp = 1e-6
        dt = 1e-1
        
        self.get_logger().info('Starting inverse kinematics calculation...')
        for i in range(IT_MAX):
            pin.forwardKinematics(self.robot.model, self.robot.data, q)
            pin.updateFramePlacements(self.robot.model, self.robot.data)
            
            err = pin.log(self.robot.data.oMf[frame_id].inverse() * target_pose).vector
            if np.linalg.norm(err) < eps:
                self.get_logger().info(f"Inverse kinematics converged in {i+1} iterations.")
                return q

            J = pin.computeFrameJacobian(self.robot.model, self.robot.data, q, frame_id)
            v = J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(6), err))
            q = pin.integrate(self.robot.model, q, v * dt)
            q = np.clip(q, self.robot.model.lowerPositionLimit, self.robot.model.upperPositionLimit)

        self.get_logger().warn("Inverse kinematics did not converge for the requested pose.")
        return None
    
    def _get_current_configuration(self, joint_state_msg):
        """Helper to get the full configuration vector 'q' from joint_states."""
        q = pin.neutral(self.robot.model)
        joint_positions_map = dict(zip(joint_state_msg.name, joint_state_msg.position))
        for j_name, j_model in zip(self.robot.model.names[1:], self.robot.model.joints[1:]):
            if j_name in joint_positions_map:
                q_slice = slice(j_model.idx_q, j_model.idx_q + j_model.nq)
                q[q_slice] = joint_positions_map[j_name]
        return q

    def publish_trajectory(self, q_final):
        """Publishes the calculated joint positions as a trajectory."""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = CONTROLLER_JOINT_NAMES
        
        point = JointTrajectoryPoint()
        final_positions = []
        for name in CONTROLLER_JOINT_NAMES:
            joint_id = self.robot.model.getJointId(name)
            q_idx = self.robot.model.joints[joint_id].idx_q
            final_positions.append(q_final[q_idx])

        point.positions = final_positions
        point.velocities = [0.0] * len(CONTROLLER_JOINT_NAMES)
        point.time_from_start.sec = 4
        
        trajectory_msg.points.append(point)
        self.joint_trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info(f"Publishing trajectory for joints: {trajectory_msg.joint_names}")

        
class RobotModelLoader(Node):
    """
    A simple node to load the robot description and make the model
    available to other nodes.
    """
    def __init__(self):
        super().__init__('robot_model_loader')
        self.robot = None
        self.model_loaded = threading.Event()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = self.create_subscription(
            String, '/robot_description', self.robot_description_callback, qos)

    def robot_description_callback(self, msg):
        """Load the robot model from the robot_description string."""
        try:
            model = pin.buildModelFromXML(msg.data)
            self.robot = RobotWrapper(model)
            self.get_logger().info('Robot model loaded successfully.')
            self.model_loaded.set()
            # Unsubscribe after successfully loading the model
            self.destroy_subscription(self.sub)
        except Exception as e:
            self.get_logger().error(f"Failed to load robot model: {e}")


def main(args=None):
    rclpy.init(args=args)

    # Node to load the robot model, shared between other nodes
    robot_model_loader = RobotModelLoader()
    
    # The motion planner and state publisher nodes
    motion_planner = MotionPlanner(robot_model_loader)
    robot_state_publisher = RobotStatePublisher(robot_model_loader)
    
    # Use a multi-threaded executor to spin multiple nodes in the same process
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot_model_loader)
    executor.add_node(motion_planner)
    executor.add_node(robot_state_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        robot_state_publisher.destroy_node()
        motion_planner.destroy_node()
        robot_model_loader.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()