"""Servicio ROS2 para consultar información de las tortugas.

Este módulo implementa un servidor de servicio que proporciona información
completa sobre las posiciones, orientaciones, velocidades y distancia entre
ambas tortugas.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from math import sqrt

from follower_interfaces.srv import TurtleInfo

from .shared_poses import SharedPoses

class TurtleInfoService(Node):
    """Nodo ROS2 que implementa el servicio TurtleInfo.
    
    Proporciona un servicio que responde con información detallada sobre
    las poses, orientaciones y velocidades de ambas tortugas, así como
    la distancia entre ellas.
    
    Attributes:
        shared_poses (SharedPoses): Objeto compartido para acceder a las poses.
        callback_group (ReentrantCallbackGroup): Grupo de callbacks reentrantes.
        turtle_info_service (Service): Servidor del servicio TurtleInfo.
    """

    def __init__(self, shared_poses: SharedPoses):
        """Inicializa el servidor de servicio TurtleInfo.
        
        Args:
            shared_poses (SharedPoses): Instancia de SharedPoses para acceder a las poses.
        """
        super().__init__("turtle_info_service")

        self.shared_poses = shared_poses

        # ========== REENTRANT CALLBACK GROUP ==========
        # Los servicios pueden recibir múltiples peticiones de diferentes clientes.
        # ReentrantCallbackGroup permite que este servicio procese múltiples peticiones
        # simultáneamente en lugar de hacerlas esperar en cola.
        #
        # Sin ReentrantCallbackGroup (MutuallyExclusiveCallbackGroup por defecto):
        # - Las peticiones se procesan secuencialmente (una a la vez)
        # - Si llegan 3 peticiones, la tercera espera a que terminen las dos primeras
        # - Tiempo de respuesta aumenta linealmente con el número de peticiones
        #
        # Con ReentrantCallbackGroup:
        # - Múltiples peticiones pueden procesarse simultáneamente en diferentes threads
        # - Cada cliente recibe su respuesta más rápidamente
        # - Mejor escalabilidad si hay múltiples clientes
        #
        # Como handle_turtle_info solo lee de shared_poses (no modifica), es seguro
        # que múltiples threads ejecuten este callback simultáneamente. El lock en
        # SharedPoses garantiza que cada thread obtiene una copia consistente.
        self.callback_group = ReentrantCallbackGroup()


        self.turtle_info_service = self.create_service(
            TurtleInfo,
            "turtle_info",
            self.handle_turtle_info,
            callback_group=self.callback_group
        )

        self.get_logger().info("Turtle Info Service is ready.")
    
    def handle_turtle_info(self, request, response):
        """Maneja las solicitudes del servicio TurtleInfo.
        
        Obtiene las poses actuales de ambas tortugas y llena la respuesta
        con sus posiciones, orientaciones, velocidades y la distancia entre ellas.
        
        Args:
            request: Solicitud del servicio (vacía para TurtleInfo).
            response: Objeto de respuesta a llenar con la información.
            
        Returns:
            TurtleInfo.Response: Respuesta con toda la información de las tortugas.
        """
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
    """Función main standalone para testing del servicio de información.
    
    Inicializa el sistema con PoseSaver, ExplorerVelocity y TurtleInfoService
    para pruebas independientes del servicio.
    
    Args:
        args: Argumentos de línea de comandos para rclpy.
    Note:
        Se recomienda no usar el módulo como programa principal en despliegues
        reales, sino solo para pruebas independientes.
    """
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

    # MultiThreadedExecutor permite que el servicio responda a peticiones mientras
    # las suscripciones actualizan poses y el timer publica velocidades.
    # Ver comentarios detallados en follower.py.
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