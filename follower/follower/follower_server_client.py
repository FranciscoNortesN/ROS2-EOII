"""Cliente del servicio TurtleInfo para consultar estado de las tortugas.

Este módulo implementa un cliente que consulta periódicamente el servicio
TurtleInfo para obtener y mostrar información sobre las posiciones, orientaciones
y velocidades de ambas tortugas.
"""

import rclpy
from rclpy.node import Node

from follower_interfaces.srv import TurtleInfo

class TurtleInfoClient(Node):
    """Cliente ROS2 para el servicio TurtleInfo.
    
    Realiza llamadas periódicas al servicio TurtleInfo cada segundo y
    muestra la información recibida sobre ambas tortugas.
    
    Attributes:
        turtle_info_client (Client): Cliente del servicio TurtleInfo.
        turtle_info_request (TurtleInfo.Request): Request reutilizable para el servicio.
    """
    
    def __init__(self):
        """Inicializa el cliente del servicio TurtleInfo.
        
        Crea el cliente, espera a que el servicio esté disponible y configura
        un timer para llamadas periódicas cada segundo.
        """
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
        """Llama al servicio de forma asíncrona cada segundo.
        
        Configura el callback para manejar la respuesta cuando esté disponible.
        """
        future = self.turtle_info_client.call_async(self.turtle_info_request)
        future.add_done_callback(self.handle_response)
    
    def handle_response(self, future):
        """Callback ejecutado cuando el servicio responde.
        
        Procesa la respuesta del servicio y muestra la información de ambas
        tortugas incluyendo posiciones, orientaciones, velocidades y distancia.
        
        Args:
            future: Future que contiene la respuesta del servicio.
        """
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
    """Función main del cliente del servicio.
    
    Inicializa el cliente del servicio TurtleInfo y realiza llamadas periódicas
    para obtener información sobre las tortugas.
    
    Args:
        args: Argumentos de línea de comandos para rclpy.
    """
    rclpy.init(args=args)
    
    turtle_info_client = TurtleInfoClient()
    
    rclpy.spin(turtle_info_client)
    
    turtle_info_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()