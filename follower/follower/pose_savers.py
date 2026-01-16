"""Guardador de poses de las tortugas desde topics de turtlesim.

Este módulo implementa un nodo que se suscribe a los topics de pose de
ambas tortugas (turtle1 y explorer) y almacena sus poses en un objeto
SharedPoses para acceso thread-safe desde otros nodos.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from turtlesim.msg import Pose

from .shared_poses import SharedPoses

class PoseSaver(Node):
    """Nodo ROS2 que guarda las poses de turtle1 y explorer.
    
    Se suscribe a los topics /turtle1/pose y /explorer/pose y almacena
    las poses recibidas en un objeto SharedPoses compartido.
    
    Attributes:
        shared_poses (SharedPoses): Objeto compartido para almacenar las poses.
        group (ReentrantCallbackGroup): Grupo de callbacks reentrantes.
        turtle_sub (Subscription): Suscripción a /turtle1/pose.
        explorer_sub (Subscription): Suscripción a /explorer/pose.
    """
    
    def __init__(self, shared_poses: SharedPoses):
        """Inicializa el guardador de poses.
        
        Args:
            shared_poses (SharedPoses): Instancia de SharedPoses para almacenar las poses.
        """
        
        super().__init__('pose_saver')
        
        self.shared_poses = shared_poses

        # ========== REENTRANT CALLBACK GROUP ==========
        # Aunque las suscripciones son rápidas, usamos ReentrantCallbackGroup para permitir
        # que turtle_callback y explorer_callback puedan ejecutarse simultáneamente.
        #
        # Sin ReentrantCallbackGroup (comportamiento por defecto):
        # - Si turtle_callback está ejecutándose, explorer_callback debe esperar
        # - Las actualizaciones de pose podrían retrasarse innecesariamente
        #
        # Con ReentrantCallbackGroup:
        # - Ambos callbacks pueden ejecutarse en paralelo
        # - Las poses se actualizan más rápidamente
        # - Mejor sincronización entre las dos tortugas
        #
        # Como ambos callbacks escriben en shared_poses, el objeto SharedPoses
        # usa threading.Lock para garantizar que las escrituras sean thread-safe.
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
        """Callback ejecutado cuando se recibe una pose de turtle1.
        
        Args:
            msg (Pose): Mensaje de pose recibido de /turtle1/pose.
        """
        self.shared_poses.set_turtle_pose(msg)
        self.get_logger().debug(f'Turtle pose saved: {msg}')
  
    def explorer_callback(self, msg: Pose):
        """Callback ejecutado cuando se recibe una pose del explorador.
        
        Args:
            msg (Pose): Mensaje de pose recibido de /explorer/pose.
        """
        self.shared_poses.set_explorer_pose(msg)
        self.get_logger().debug(f'Explorer pose saved: {msg}')
    
def main(args=None):
    """Función main standalone para testing del guardador de poses.
    
    Inicializa solo el PoseSaver para pruebas independientes de guardado de poses.
    
    Args:
        args: Argumentos de línea de comandos para rclpy.
    Note:
        Se recomienda no usar el módulo como programa principal en despliegues
        reales, sino solo para pruebas independientes.
    """
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