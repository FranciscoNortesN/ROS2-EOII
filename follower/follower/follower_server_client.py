import rclpy
from rclpy.node import Node

from follower_interfaces.srv import TurtleInfo

class TurtleInfoClient(Node):
    
    def __init__(self):
        super().__init__('turtle_info_client')
        
        self.turtle_info_client = self.create_client(
            TurtleInfo,
            'turtle_info'
        )

        while not self.turtle_info_client.wait_for_service(1.0):
            self.get_logger().info('Service not available, trying again ...')

        self.turtle_info_request = TurtleInfo.Request()

        self.create_timer(1.0, self.turtle_info_callback)
    
    def turtle_info_callback(self):
        """Llama al servicio de forma asíncrona cada segundo."""
        future = self.turtle_info_client.call_async(self.turtle_info_request)
        future.add_done_callback(self.handle_response)
    
    def handle_response(self, future):
        """Callback ejecutado cuando el servicio responde."""
        try:
            response = future.result()
            self.get_logger().info(
                f'\n===== Turtle Info =====\n'
                f'Turtle1: x={response.turtle_x:.2f}, y={response.turtle_y:.2f}, θ={response.turtle_theta:.2f}\n'
                f'Explorer: x={response.explorer_x:.2f}, y={response.explorer_y:.2f}, θ={response.explorer_theta:.2f}\n'
                f'Turtle1 Velocities: linear={response.turtle_linear_velocity:.2f}, angular={response.turtle_angular_velocity:.2f}\n'
                f'Explorer Velocities: linear={response.explorer_linear_velocity:.2f}, angular={response.explorer_angular_velocity:.2f}\n'
                f'Distance: {response.distance:.2f}\n'
                f'======================'
            )
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    turtle_info_client = TurtleInfoClient()
    
    rclpy.spin(turtle_info_client)
    
    turtle_info_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()