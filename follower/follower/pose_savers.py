import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from turtlesim.msg import Pose

from .shared_poses import SharedPoses

class PoseSaver(Node):
    
    def __init__(self, shared_poses: SharedPoses):
        
        super().__init__('pose_saver')
        
        self.shared_poses = shared_poses

        self.group = ReentrantCallbackGroup()
        
        self.turtle_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.turtle_callback,
            10,
            callback_group=self.group
        )
        self.explorer_sub = self.create_subscription(
            Pose,
            '/explorer/pose',
            self.explorer_callback,
            10,
            callback_group=self.group
        )

        self.turtle_sub  # prevent unused variable warning
        self.explorer_sub  # prevent unused variable warning

    def turtle_callback(self, msg: Pose):
        self.shared_poses.set_turtle_pose(msg)
        self.get_logger().debug(f'Turtle pose saved: {msg}')
  
    def explorer_callback(self, msg: Pose):
        self.shared_poses.set_explorer_pose(msg)
        self.get_logger().debug(f'Explorer pose saved: {msg}')
    
def main(args=None):
    from .create_explorer import spawn_explorer

    rclpy.init(args=args)
    
    spawn_explorer()

    shared_poses = SharedPoses()
    
    pose_saver = PoseSaver(shared_poses)
    
    rclpy.spin(pose_saver)
    
    pose_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()