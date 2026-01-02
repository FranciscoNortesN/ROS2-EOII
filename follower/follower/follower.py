import rclpy
from rclpy.executors import MultiThreadedExecutor

from .shared_poses import SharedPoses
from .create_explorer import spawn_explorer
from .pose_savers import PoseSaver
from .explorer_velocity import ExplorerVelocity
from .turtle_info_service import TurtleInfoService
from .catch_info_action import CatchInfoAction

def main(args=None):
    rclpy.init(args=args)

    spawn_explorer()

    shared_poses = SharedPoses()

    pose_saver = PoseSaver(shared_poses)
    explorer_velocity = ExplorerVelocity(shared_poses)
    turtle_info_service = TurtleInfoService(shared_poses)
    catch_info_action = CatchInfoAction(shared_poses)

    executor = MultiThreadedExecutor()

    executor.add_node(pose_saver)
    executor.add_node(explorer_velocity)
    executor.add_node(turtle_info_service)
    executor.add_node(catch_info_action)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    pose_saver.destroy_node()
    explorer_velocity.destroy_node()
    turtle_info_service.destroy_node()
    catch_info_action.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()