import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist

from math import atan2, cos, sin, sqrt

from follower_interfaces.srv import TurtleInfo

class Follower(Node):
    def __init__(self):
        super().__init__("follower")

        # Declarar parámetros con valores por defecto
        self.declare_parameter('explorer_x', 2.0)
        self.declare_parameter('explorer_y', 2.0)
        
        # Obtener valores y validar límites (turtlesim: 0.0 a 11.0)
        x = self.get_parameter('explorer_x').value
        y = self.get_parameter('explorer_y').value
        
        if not (0.0 <= x <= 11.0 and 0.0 <= y <= 11.0):
            self.get_logger().error(f'Invalid position: ({x}, {y}). Must be between 0 and 11')
            x = min(max(x, 0.0), 11.0)
            y = min(max(y, 0.0), 11.0)
            self.get_logger().warn(f'Adjusted to: ({x}, {y})')
        
        # Crear cliente y esperar servicio
        spawn_client = self.create_client(Spawn, '/spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')
        
        # Llamar al servicio
        request = Spawn.Request(x=x, y=y, theta=0.0, name='explorer')
        spawn_client.call_async(request)

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
        
        self.turtle_info = self.create_service(
            TurtleInfo,
            "turtle_info",
            self.turtle_info_callback
        )

        self.turtle_subscription
        self.explorer_subscription
        self.explorer_velocity_changer
        self.explorer_velocity_timer
        self.turtle_info

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

        # como la tortuga solo puede ir para adelante/atrás y girar hacia un lado u otro, el resto de valores no hacen nada y se dejan a 0
        msg.linear.x = v
        msg.angular.z = w

        self.explorer_velocity_changer.publish(msg)

    def turtle_info_callback(self, request, response):
        response.turtle_x = self.turtle_pose.x
        response.turtle_y = self.turtle_pose.y
        response.explorer_x = self.explorer_pose.x
        response.explorer_y = self.explorer_pose.y

        response.turtle_theta = self.turtle_pose.theta
        response.explorer_theta = self.explorer_pose.theta

        response.turtle_linear_velocity = self.turtle_pose.linear_velocity
        response.turtle_angular_velocity = self.turtle_pose.angular_velocity
        response.explorer_linear_velocity = self.explorer_pose.linear_velocity
        response.explorer_angular_velocity = self.explorer_pose.angular_velocity

        dx = self.turtle_pose.x - self.explorer_pose.x
        dy = self.turtle_pose.y - self.explorer_pose.y

        d = sqrt(dx*dx + dy*dy)

        response.distance = d

        return response

def main(args=None):
    rclpy.init(args=args)

    follower = Follower()

    rclpy.spin(follower)

    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
