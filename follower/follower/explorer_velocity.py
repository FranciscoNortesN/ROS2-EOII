import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist

from math import atan2, cos, sin, sqrt

from .shared_poses import SharedPoses

class ExplorerVelocity(Node):
    def __init__(self, shared_poses: SharedPoses):
        super().__init__('explorer_velocity')

        self.shared_poses = shared_poses

        self.group = ReentrantCallbackGroup()

        self.explorer_velocity_changer = self.create_publisher(
            Twist,
            "/explorer/cmd_vel",
            10,
            callback_group=self.group
        )
        self.explorer_velocity_timer = self.create_timer(
            0.05,
            self.update_explorer_velocity,
            callback_group=self.group
        )

        self.explorer_velocity_changer # prevent unused variable warning
        self.explorer_velocity_timer # prevent unused variable warning

    def update_explorer_velocity(self):
        turtle_pose, explorer_pose = self.shared_poses.get_poses()

        vel_msg = Twist()

        distance = sqrt((turtle_pose.x - explorer_pose.x) ** 2 +
                        (turtle_pose.y - explorer_pose.y) ** 2)

        angle_to_turtle = atan2(turtle_pose.y - explorer_pose.y,
                                turtle_pose.x - explorer_pose.x)
        
        angle_error = atan2(sin(angle_to_turtle - explorer_pose.theta),
                            cos(angle_to_turtle - explorer_pose.theta))
        
        vel_msg.linear.x = 1.0 * distance * cos(angle_error)
        vel_msg.angular.z = 4.0 * angle_error

        self.explorer_velocity_changer.publish(vel_msg)
        self.get_logger().debug(f'Published explorer velocity: linear={vel_msg.linear.x}, angular={vel_msg.angular.z}')


def main(args=None):
    from rclpy.executors import MultiThreadedExecutor

    from .create_explorer import spawn_explorer
    from .pose_savers import PoseSaver

    rclpy.init(args=args)
    
    spawn_explorer()

    shared_poses = SharedPoses()
    
    pose_saver = PoseSaver(shared_poses)
    explorer_velocity = ExplorerVelocity(shared_poses)
    
    executor = MultiThreadedExecutor()
    executor.add_node(pose_saver)
    executor.add_node(explorer_velocity)

    try:
        executor.spin()
    finally:
        pose_saver.destroy_node()
        explorer_velocity.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()