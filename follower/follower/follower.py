from math import atan2, cos, sin, sqrt
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class Follower(Node):
    def __init__(self):
        super().__init__("follower")
        self.turtle_pose = Pose()
        self.explorer_pose = Pose()

        self.turtle_subscription = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.turtle_listener_callback,
            10
        )
        self.explorer_subscription = self.create_subscription(
            Pose,
            "/explorer/pose",
            self.explorer_listener_callback,
            10
        )

        self.explorer_velocity_changer = self.create_publisher(
            Twist,
            "/explorer/cmd_vel",
            10
        )
        self.explorer_velocity_timer = self.create_timer(
            0.05,
            self.explorer_velocity_callback
        )

        self.turtle_subscription
        self.explorer_subscription
        self.explorer_velocity_changer
        self.explorer_velocity_timer

    def turtle_listener_callback(self, msg:Pose):
        self.turtle_pose = msg

    def explorer_listener_callback(self, msg:Pose):
        self.explorer_pose = msg

    def explorer_velocity_callback(self):
        msg = Twist()
        
        dx = self.turtle_pose.x - self.explorer_pose.x
        dy = self.turtle_pose.y - self.explorer_pose.y

        d = sqrt(dx*dx + dy*dy)

        theta_d = atan2(dy, dx)
        e_theta = atan2(sin(theta_d - self.explorer_pose.theta), cos(theta_d - self.explorer_pose.theta))
        
        v = 1.0 * d * cos(e_theta)
        w = 4.0 * e_theta

        # cómo la tortuga solo puede ir para adelante/atrás y girar hacia un lado u otro, el resto de valores no hacen nada
        msg.linear.x = v
        msg.angular.z = w

        self.explorer_velocity_changer.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    follower = Follower()

    rclpy.spin(follower)

    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
