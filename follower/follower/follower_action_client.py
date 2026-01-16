"""Cliente de acción para monitorear el progreso de captura.

Este módulo implementa un cliente del servidor de acción CatchInfo que
envía un goal y muestra el feedback recibido sobre las posiciones y
velocidades de ambas tortugas.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from follower_interfaces.action import CatchInfo

class FollowerActionClient(Node):
    """Cliente ROS2 para el servidor de acción CatchInfo.
    
    Envía goals al servidor de acción y procesa el feedback y resultado,
    mostrando información detallada sobre el estado de las tortugas.
    
    Attributes:
        _action_client (ActionClient): Cliente de acción para CatchInfo.
    """
    
    def __init__(self):
        """Inicializa el cliente de acción."""
        super().__init__('follower_action_client')

        self._action_client = ActionClient(self, CatchInfo, 'catch_info')

        self.get_logger().info("Follower Action Client has been started.")

    def send_goal(self):
        """Envía un goal al servidor de acción CatchInfo.
        
        Crea y envía un goal vacío (CatchInfo no requiere parámetros de entrada)
        al servidor de acción y configura los callbacks para respuesta y feedback.
        """
        goal_msg = CatchInfo.Goal()

        self._action_client.wait_for_server()

        self.get_logger().info("Sending goal to catch_info action server...")

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback ejecutado cuando el servidor responde a la solicitud de goal.
        
        Args:
            future: Future que contiene el goal_handle de la respuesta.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback ejecutado cuando se recibe el resultado final de la acción.
        
        Args:
            future: Future que contiene el resultado de la acción.
        """
        result = future.result().result
        if result.caught:
            self.get_logger().info("The turtle has caught the explorer!")
        else:
            self.get_logger().info("The turtle failed to catch the explorer.")

    def feedback_callback(self, feedback_msg):
        """Callback ejecutado cuando se recibe feedback del servidor de acción.
        
        Muestra información detallada sobre las posiciones, orientaciones y
        velocidades de ambas tortugas, así como la distancia entre ellas.
        
        Args:
            feedback_msg: Mensaje de feedback recibido del servidor.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'\n===== Turtle Info =====\n'
            f'Turtle1: x={feedback.turtle_x:.2f}, y={feedback.turtle_y:.2f}, θ={feedback.turtle_theta:.2f}\n'
            f'Explorer: x={feedback.explorer_x:.2f}, y={feedback.explorer_y:.2f}, θ={feedback.explorer_theta:.2f}\n'
            f'Turtle1 Velocities: linear={feedback.turtle_linear_velocity:.2f}, angular={feedback.turtle_angular_velocity:.2f}\n'
            f'Explorer Velocities: linear={feedback.explorer_linear_velocity:.2f}, angular={feedback.explorer_angular_velocity:.2f}\n'
            f'Distance: {feedback.distance:.2f}\n'
            f'======================'
        )

def main(args=None):
    """Función main del cliente de acción.
    
    Inicializa el cliente de acción, envía un goal y procesa el feedback
    y resultado recibidos del servidor.
    
    Args:
        args: Argumentos de línea de comandos para rclpy.
    """
    rclpy.init(args=args)
    
    action_client = FollowerActionClient()
    action_client.send_goal()
    
    rclpy.spin(action_client)
    
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()