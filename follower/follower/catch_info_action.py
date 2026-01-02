import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from math import sqrt
from time import sleep

from follower_interfaces.action import CatchInfo
from turtlesim.msg import Pose

from .shared_poses import SharedPoses

class CatchInfoAction(Node):
    def __init__(self, shared_poses: SharedPoses):
        super().__init__("catch_info_action")

        self.shared_poses = shared_poses

        self.group = ReentrantCallbackGroup()

        self.catch_info_action_server = rclpy.action.ActionServer(
            self,
            CatchInfo,
            "catch_info",
            self.catch_callback,
            callback_group=self.group
        )

    def catch_callback(self, goal_handle):
        feedback_msg = CatchInfo.Feedback()
        
        # Esperar a que las poses se inicialicen
        while not self.shared_poses.are_poses_initialized() and rclpy.ok():
            sleep(0.05)
        
        turtle_pose, explorer_pose = self.shared_poses.get_poses()
        
        self.get_logger().info(
            f"Starting catch action - Distance: {sqrt((explorer_pose.x - turtle_pose.x)**2 + (explorer_pose.y - turtle_pose.y)**2):.2f}"
        )

        while rclpy.ok() and not goal_handle.is_cancel_requested and not self._acceptable_caught_distance(turtle_pose, explorer_pose):
            turtle_pose, explorer_pose = self.shared_poses.get_poses()

            feedback_msg.turtle_x = turtle_pose.x
            feedback_msg.turtle_y = turtle_pose.y
            feedback_msg.turtle_theta = turtle_pose.theta

            feedback_msg.explorer_x = explorer_pose.x
            feedback_msg.explorer_y = explorer_pose.y
            feedback_msg.explorer_theta = explorer_pose.theta

            feedback_msg.distance = sqrt(
                (explorer_pose.x - turtle_pose.x) ** 2 +
                (explorer_pose.y - turtle_pose.y) ** 2
            )

            goal_handle.publish_feedback(feedback_msg)

            sleep(0.1)
        
        goal_handle.succeed()
        result = CatchInfo.Result()
        result.caught = self._acceptable_caught_distance(turtle_pose, explorer_pose)

        return result
    
    def _acceptable_caught_distance(self, pose1:Pose, pose2:Pose) -> bool:
        distance = sqrt(
            (pose1.x - pose2.x) ** 2 +
            (pose1.y - pose2.y) ** 2
        )
        return distance < 0.1

def main(args=None):
    from rclpy.executors import MultiThreadedExecutor

    from .create_explorer import spawn_explorer
    from .pose_savers import PoseSaver
    from .explorer_velocity import ExplorerVelocity

    rclpy.init(args=args)

    spawn_explorer()

    shared_poses = SharedPoses()

    pose_saver = PoseSaver(shared_poses)
    explorer_velocity = ExplorerVelocity(shared_poses)
    catch_info_action = CatchInfoAction(shared_poses)

    executor = MultiThreadedExecutor()

    executor.add_node(pose_saver)
    executor.add_node(explorer_velocity)
    executor.add_node(catch_info_action)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    pose_saver.destroy_node()
    explorer_velocity.destroy_node()
    catch_info_action.destroy_node()
    rclpy.shutdown()

