import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np

class PinocchioController(Node):
    def __init__(self):
        super().__init__('pinocchio_controller')

        # --- IMPORTANT: Controller-Specific Configuration ---
        # This list MUST match the joints that your 'iiwa_controller' is configured to control.
        # Check your controller's configuration YAML file to be certain.
        self.CONTROLLER_JOINT_NAMES = [
            'joint_a1', 'joint_a2', 'joint_a3', 'joint_a4',
            'joint_a5', 'joint_a6', 'joint_a7'
        ]

        # --- State Variables ---
        self.robot = None
        self.model_loaded = False
        self.current_joint_state = None
        self.target_reached = False
        self.ik_running = False

        # --- QoS Profile for Robot Description ---
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # --- ROS 2 Subscribers ---
        self.robot_description_subscriber = self.create_subscription(
            String,
            '/robot_description',
            self.robot_description_callback,
            qos_profile)

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # --- ROS 2 Publishers ---
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/iiwa_controller/joint_trajectory',
            10)

        # --- Define Target Pose (Position and Orientation) ---
        # IMPORTANT: Change these values to your desired target
        self.target_pose = pin.SE3(pin.rpy.rpyToMatrix(np.pi, 0.0, np.pi/2), np.array([0.7, 0, 0.2]))

        # --- Control Loop Timer ---
        self.timer = self.create_timer(1.0, self.control_loop)
        self.get_logger().info('Pinocchio controller node started. Waiting for robot description...')

    def robot_description_callback(self, msg):
        if self.model_loaded:
            return

        robot_description_string = msg.data
        self.get_logger().info('Received robot description.')

        try:
            # --- CORRECTED MODEL LOADING ---
            # First, build the model from the XML string.
            model = pin.buildModelFromXML(robot_description_string)
            # Then, create the RobotWrapper from the model.
            self.robot = RobotWrapper(model)
            self.model_loaded = True
            
            # --- IMPORTANT: Diagnostic Logging ---
            self.get_logger().info('Pinocchio model loaded successfully.')
            self.get_logger().info(f'Model Configuration Dimension (nq): {self.robot.model.nq}')
            self.get_logger().info(f'Model Velocity Dimension (nv): {self.robot.model.nv}')
            self.get_logger().info(f'Model Joint Names: {list(self.robot.model.names)}')
            
            self.destroy_subscription(self.robot_description_subscriber)
        except Exception as e:
            self.get_logger().error(f"Failed to load robot model from description: {e}")

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def control_loop(self):
        if not self.model_loaded or self.current_joint_state is None or self.ik_running:
            return

        self.ik_running = True
        
        # --- Build the full configuration vector 'q' ---
        q0 = pin.neutral(self.robot.model)
        joint_positions_map = dict(zip(self.current_joint_state.name, self.current_joint_state.position))

        for j_name, j_model in zip(self.robot.model.names[1:], self.robot.model.joints[1:]):
            if j_name in joint_positions_map:
                q_slice = slice(j_model.idx_q, j_model.idx_q + j_model.nq)
                q0[q_slice] = joint_positions_map[j_name]
        
        # --- Inverse Kinematics Calculation ---
        EE_FRAME_NAME = "tool0"
        try:
            frame_id = self.robot.model.getFrameId(EE_FRAME_NAME)
        except IndexError:
             self.get_logger().error(f"End-effector frame '{EE_FRAME_NAME}' not found. Stopping.")
             self.timer.cancel()
             self.ik_running = False
             return
        
        q = q0
        eps = 1e-4
        IT_MAX = 1000
        damp = 1e-6
        dt = 1e-1
        
        self.get_logger().info('Starting inverse kinematics calculation...')
        for i in range(IT_MAX):
            pin.forwardKinematics(self.robot.model, self.robot.data, q)
            pin.updateFramePlacements(self.robot.model, self.robot.data)
            
            err = pin.log(self.robot.data.oMf[frame_id].inverse() * self.target_pose).vector
            if np.linalg.norm(err) < eps:
                self.get_logger().info(f"Inverse kinematics converged in {i+1} iterations.")
                self.target_reached = True
                break

            J = pin.computeFrameJacobian(self.robot.model, self.robot.data, q, frame_id)
            v = J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(6), err))
            q = pin.integrate(self.robot.model, q, v * dt)
            
            # Enforce joint limits at each step of the IK
            q = np.clip(q, self.robot.model.lowerPositionLimit, self.robot.model.upperPositionLimit)

        if not self.target_reached:
            self.get_logger().warn("Inverse kinematics did not converge.")
            self.ik_running = False
            return

        # --- Publish Trajectory ---
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.CONTROLLER_JOINT_NAMES
        
        point = JointTrajectoryPoint()
        final_positions = []
        # Extract the positions for only the joints the controller manages
        for name in self.CONTROLLER_JOINT_NAMES:
            joint_id = self.robot.model.getJointId(name)
            q_idx = self.robot.model.joints[joint_id].idx_q
            final_positions.append(q[q_idx])

        point.positions = final_positions
        point.time_from_start.sec = 4
        print(f"Final joint positions: {point.positions}")


        trajectory_msg.points.append(point)
        self.joint_trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info(f"Publishing trajectory to controller for joints: {trajectory_msg.joint_names}")
        self.timer.cancel()
        self.ik_running = False

def main(args=None):
    rclpy.init(args=args)
    pinocchio_controller = PinocchioController()
    try:
        rclpy.spin(pinocchio_controller)
    except KeyboardInterrupt:
        pass
    finally:
        pinocchio_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

