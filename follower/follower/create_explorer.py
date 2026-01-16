"""Utilidad para crear una tortuga exploradora en turtlesim.

Este módulo proporciona funciones para spawear (crear) tortugas adicionales
en el simulador turtlesim, con validación de posición y configuración mediante
parámetros ROS2.
"""

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


def spawn_explorer(name: str = 'explorer'):
    """
    Crea una tortuga explorer en turtlesim en la posición especificada.
    Lee los parámetros 'explorer_x' y 'explorer_y' para la posición.

    Utiliza un nodo temporal para llamar al servicio /spawn en vez de ser 
    un nodo persistente debido a que no se quiere que este módulo mantenga 
    un nodo activo en el sistema.
    
    Args:
        name (str): Nombre de la tortuga
    
    Returns:
        str: Nombre de la tortuga creada, o None si falla
    """
    # Crear nodo temporal
    temp_node = Node('temp_spawn_node')
    
    # ========== PARÁMETROS ROS2 ==========
    # Se usan parámetros ROS2 (en lugar de argumentos de función) porque:
    # 1. Es un requisito del proyecto
    # 2. Permite configurar la posición desde los archivos launch
    # 3. Sigue las convenciones estándar de ROS2 para configuración
    #
    # Valores por defecto: (2.0, 2.0) - esquina inferior izquierda del mundo
    temp_node.declare_parameter('explorer_x', 2.0)
    temp_node.declare_parameter('explorer_y', 2.0)
    
    # Obtener valores
    x = temp_node.get_parameter('explorer_x').value
    y = temp_node.get_parameter('explorer_y').value
    
    # ========== VALIDACIÓN DE LÍMITES ==========
    # El mundo de turtlesim es un cuadrado de 11×11 unidades (0 a 11)
    # Posiciones fuera de estos límites causarían:
    # - Error en el servicio /spawn
    # - Tortuga invisible o comportamiento indefinido
    # Por eso validamos y ajustamos si es necesario
    if not (0.0 <= x <= 11.0 and 0.0 <= y <= 11.0):
        temp_node.get_logger().warning(f'Invalid position: ({x}, {y}). Must be between 0 and 11')
        x = min(max(x, 0.0), 11.0)
        y = min(max(y, 0.0), 11.0)
        temp_node.get_logger().warning(f'Adjusted to: ({x}, {y})')
    
    # Crear cliente y esperar servicio
    spawn_client = temp_node.create_client(Spawn, '/spawn')
    while not spawn_client.wait_for_service(timeout_sec=1.0):
        temp_node.get_logger().info('Waiting for /spawn service...')
    
    # Llamar al servicio de forma síncrona
    request = Spawn.Request(x=x, y=y, theta=0.0, name=name)
    future = spawn_client.call_async(request)
    
    # Esperar respuesta
    rclpy.spin_until_future_complete(temp_node, future)
    
    result = None
    if future.result() is not None:
        result = future.result().name
        temp_node.get_logger().info(f'Turtle "{result}" spawned at ({x}, {y})')
    else:
        temp_node.get_logger().error('Failed to spawn turtle')
    
    # Limpiar
    temp_node.destroy_node()
    
    return result


def main(args=None):
    """Función main standalone para spawear el explorador.
    
    Inicializa ROS2, crea la tortuga exploradora y finaliza.
    
    Args:
        args: Argumentos de línea de comandos para rclpy.
    Note:
        Se recomienda no usar el módulo como programa principal en despliegues
        reales, sino solo para pruebas independientes.
    """
    rclpy.init(args=args)
    spawn_explorer()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
