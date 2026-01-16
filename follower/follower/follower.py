"""Punto de entrada principal del paquete follower.

Este módulo inicializa y ejecuta todos los nodos necesarios para el sistema
de seguimiento de tortugas: guarda poses, controla velocidad del explorador,
proporciona servicio de información y servidor de acción de captura.
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .shared_poses import SharedPoses
from .create_explorer import spawn_explorer
from .pose_savers import PoseSaver
from .explorer_velocity import ExplorerVelocity
from .turtle_info_service import TurtleInfoService
from .catch_info_action import CatchInfoAction

def main(args=None):
    """Función principal que inicializa y ejecuta todos los nodos del sistema.
    
    Crea el explorador, inicializa el contenedor compartido de poses y lanza
    todos los nodos necesarios en un MultiThreadedExecutor para permitir
    callbacks concurrentes.
    
    Args:
        args: Argumentos de línea de comandos para rclpy.
    """
    rclpy.init(args=args)

    spawn_explorer()

    shared_poses = SharedPoses()

    pose_saver = PoseSaver(shared_poses)
    explorer_velocity = ExplorerVelocity(shared_poses)
    turtle_info_service = TurtleInfoService(shared_poses)
    catch_info_action = CatchInfoAction(shared_poses)

    # ========== MULTITHREADED EXECUTOR ==========
    # Por defecto, rclpy.spin() usa un SingleThreadedExecutor que ejecuta todos
    # los callbacks de forma secuencial en un solo thread. Esto significa que si
    # un callback está ocupado, los demás deben esperar su turno.
    #
    # En este proyecto, tenemos varios callbacks que pueden ejecutarse simultáneamente:
    # - Los callbacks de suscripción (turtle_callback, explorer_callback)
    # - El callback del timer (update_explorer_velocity)
    # - El callback del servicio (handle_turtle_info)
    # - El callback del action server (catch_callback)
    #
    # Si usáramos rclpy.spin() (SingleThreadedExecutor), tendríamos estos problemas:
    # 1. Si el servicio está procesando una petición, el timer no puede actualizar velocidades
    # 2. Si el action está publicando feedback, las poses no se actualizarían
    # 3. Posibles deadlocks si un callback necesita esperar a otro
    #
    # MultiThreadedExecutor crea un pool de threads que permite ejecutar múltiples
    # callbacks simultáneamente. Cada nodo puede procesar sus callbacks en paralelo,
    # mejorando la capacidad de respuesta del sistema.
    #
    # IMPORTANTE: Como ahora múltiples threads acceden a shared_poses simultáneamente,
    # necesitamos sincronización (threading.Lock) para evitar condiciones de carrera.
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