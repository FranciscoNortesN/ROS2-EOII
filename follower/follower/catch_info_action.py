"""Servidor de acción para rastrear el progreso de captura entre tortugas.

Este módulo implementa un servidor de acción ROS2 que monitorea continuamente
la distancia entre la tortuga por defecto (turtle1) y el explorador, proporcionando
feedback en tiempo real hasta que la distancia sea menor a un umbral definido.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from math import sqrt
from time import sleep

from follower_interfaces.action import CatchInfo
from turtlesim.msg import Pose

from .shared_poses import SharedPoses

class CatchInfoAction(Node):
    """Nodo ROS2 que implementa el servidor de acción CatchInfo.
    
    Este nodo proporciona un servidor de acción que monitorea la distancia entre
    dos tortugas (turtle1 y explorer) y reporta feedback continuo sobre sus
    posiciones, orientaciones y velocidades hasta que están suficientemente cerca.
    
    Attributes:
        shared_poses (SharedPoses): Objeto compartido para acceder a las poses de las tortugas.
        group (ReentrantCallbackGroup): Grupo de callbacks reentrantes para concurrencia.
        catch_info_action_server (ActionServer): Servidor de acción para CatchInfo.
    """
    
    def __init__(self, shared_poses: SharedPoses):
        """Inicializa el servidor de acción CatchInfo.
        
        Args:
            shared_poses (SharedPoses): Instancia de SharedPoses para acceder a las poses.
        """
        super().__init__("catch_info_action")

        self.shared_poses = shared_poses

        # ========== REENTRANT CALLBACK GROUP ==========
        # Por defecto, ROS2 asigna todos los callbacks de un nodo a un MutuallyExclusiveCallbackGroup,
        # lo que significa que solo UN callback de ese nodo puede ejecutarse a la vez, incluso con
        # MultiThreadedExecutor.
        #
        # ReentrantCallbackGroup permite que múltiples callbacks del MISMO nodo se ejecuten
        # simultáneamente en diferentes threads. Esto es necesario cuando:
        # - Un callback puede tardar mucho tiempo (como catch_callback que espera en un bucle)
        # - Necesitamos que otros callbacks se ejecuten mientras uno está ocupado
        #
        # Sin ReentrantCallbackGroup:
        # - Si catch_callback está en su bucle de espera, NO se pueden procesar nuevos goals
        # - El action server quedaría bloqueado hasta que termine el goal actual
        #
        # Con ReentrantCallbackGroup:
        # - Múltiples goals pueden procesarse simultáneamente (aunque en este caso solo aceptamos uno)
        # - El servidor puede seguir respondiendo mientras procesa un goal
        #
        # NOTA: Al usar ReentrantCallbackGroup, debemos asegurar thread-safety en shared_poses.
        self.group = ReentrantCallbackGroup()

        self.catch_info_action_server = ActionServer(
            self,
            CatchInfo,
            "catch_info",
            self.catch_callback,
            callback_group=self.group
        )

    def catch_callback(self, goal_handle):
        """Callback ejecutado cuando se recibe un goal de acción.
        
        Monitorea continuamente la distancia entre turtle1 y explorer, publicando
        feedback con sus posiciones, orientaciones y velocidades hasta que la
        distancia sea menor al umbral definido o se cancele la acción.
        
        Args:
            goal_handle: Handle del goal de acción proporcionado por el ActionServer.
            
        Returns:
            CatchInfo.Result: Resultado indicando si el explorer fue capturado.
        """
        feedback_msg = CatchInfo.Feedback()
        
        # Esperar a que las poses se inicialicen con timeout de seguridad
        # 50 iteraciones × 0.05s = 2.5 segundos máximo de espera
        # Previene bloqueo infinito si turtlesim no publica poses
        tried = 0
        while not self.shared_poses.are_poses_initialized() and rclpy.ok() and tried < 50:
            sleep(0.05)
            tried += 1

        if tried >= 50:
            self.get_logger().error("Catch action aborted - Poses not initialized in time")
            goal_handle.abort()
            result = CatchInfo.Result()
            result.caught = False
            return result
        
        turtle_pose, explorer_pose = self.shared_poses.get_poses()
        
        self.get_logger().info(
            f"Starting catch action - Distance: {sqrt((explorer_pose.x - turtle_pose.x)**2 + (explorer_pose.y - turtle_pose.y)**2):.2f}"
        )

        while rclpy.ok() and not goal_handle.is_cancel_requested and not self._acceptable_caught_distance(turtle_pose, explorer_pose):
            turtle_pose, explorer_pose = self.shared_poses.get_poses()

            feedback_msg.turtle_x = turtle_pose.x
            feedback_msg.turtle_y = turtle_pose.y
            feedback_msg.turtle_theta = turtle_pose.theta

            feedback_msg.explorer_x = explorer_pose.x
            feedback_msg.explorer_y = explorer_pose.y
            feedback_msg.explorer_theta = explorer_pose.theta

            feedback_msg.distance = sqrt(
                (explorer_pose.x - turtle_pose.x) ** 2 +
                (explorer_pose.y - turtle_pose.y) ** 2
            )

            feedback_msg.turtle_linear_velocity = turtle_pose.linear_velocity
            feedback_msg.turtle_angular_velocity = turtle_pose.angular_velocity
            feedback_msg.explorer_linear_velocity = explorer_pose.linear_velocity
            feedback_msg.explorer_angular_velocity = explorer_pose.angular_velocity

            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().debug(
                f"Feedback - Distance: {feedback_msg.distance:.2f}, "
                f"Turtle: ({feedback_msg.turtle_x:.2f}, {feedback_msg.turtle_y:.2f}), "
                f"Explorer: ({feedback_msg.explorer_x:.2f}, {feedback_msg.explorer_y:.2f})"
            )
            
            sleep(1.0)
        
        goal_handle.succeed()
        result = CatchInfo.Result()
        result.caught = self._acceptable_caught_distance(turtle_pose, explorer_pose)
        
        if result.caught:
            self.get_logger().info("Catch action completed successfully - Explorer caught!")
        else:
            self.get_logger().info("Catch action completed - Explorer not caught")

        return result
    
    def _acceptable_caught_distance(self, pose1:Pose, pose2:Pose) -> bool:
        """Determina si dos tortugas están suficientemente cerca.
        
        Calcula la distancia euclidiana entre dos poses y compara con un umbral.
        
        Args:
            pose1 (Pose): Pose de la primera tortuga.
            pose2 (Pose): Pose de la segunda tortuga.
            
        Returns:
            bool: True si la distancia es menor a 0.1, False en caso contrario.
        """
        distance = sqrt(
            (pose1.x - pose2.x) ** 2 +
            (pose1.y - pose2.y) ** 2
        )
        # Umbral de 0.1 unidades (empírico)
        return distance < 0.1

def main(args=None):
    """Función main standalone para testing del servidor de acción.
    
    Inicializa el sistema con PoseSaver, ExplorerVelocity y CatchInfoAction
    para pruebas independientes del servidor de acción.
    
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
    catch_info_action = CatchInfoAction(shared_poses)

    # Usamos MultiThreadedExecutor porque catch_callback puede estar en un bucle largo
    # esperando a que las tortugas se acerquen. Con SingleThreadedExecutor, esto bloquearía
    # las actualizaciones de pose y velocidad. Ver comentarios en follower.py para más detalles.
    executor = MultiThreadedExecutor()

    executor.add_node(pose_saver)
    executor.add_node(explorer_velocity)
    executor.add_node(catch_info_action)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    pose_saver.destroy_node()
    explorer_velocity.destroy_node()
    catch_info_action.destroy_node()
    rclpy.shutdown()

