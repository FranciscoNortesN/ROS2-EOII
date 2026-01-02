import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from math import sqrt

from follower_interfaces.srv import TurtleInfo

from .shared_poses import SharedPoses

class TurtleInfoService(Node):

    def __init__(self, shared_poses: SharedPoses):
        super().__init__("turtle_info_service")

        self.shared_poses = shared_poses

        self.callback_group = ReentrantCallbackGroup()


        self.turtle_info_service = self.create_service(
            TurtleInfo,
            "turtle_info",
            self.handle_turtle_info,
            callback_group=self.callback_group
        )

        self.get_logger().info("Turtle Info Service is ready.")
    
    def handle_turtle_info(self, request, response):
        turtle_pose, explorer_pose = self.shared_poses.get_poses()

        response.turtle_x = turtle_pose.x
        response.turtle_y = turtle_pose.y
        response.turtle_theta = turtle_pose.theta

        response.explorer_x = explorer_pose.x
        response.explorer_y = explorer_pose.y
        response.explorer_theta = explorer_pose.theta

        response.turtle_linear_velocity = turtle_pose.linear_velocity
        response.turtle_angular_velocity = turtle_pose.angular_velocity

        response.explorer_linear_velocity = explorer_pose.linear_velocity
        response.explorer_angular_velocity = explorer_pose.angular_velocity

        response.distance = sqrt(
            (explorer_pose.x - turtle_pose.x) ** 2 +
            (explorer_pose.y - turtle_pose.y) ** 2
        )

        return response
    
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
    turtle_info_service = TurtleInfoService(shared_poses)

    executor = MultiThreadedExecutor()

    executor.add_node(pose_saver)
    executor.add_node(explorer_velocity)
    executor.add_node(turtle_info_service)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    pose_saver.destroy_node()
    explorer_velocity.destroy_node()
    turtle_info_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()