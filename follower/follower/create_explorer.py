import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


def spawn_explorer(name='explorer'):
    """
    Crea una tortuga explorer en turtlesim en la posición especificada.
    Lee los parámetros 'explorer_x' y 'explorer_y' para la posición.
    
    Args:
        name (str): Nombre de la tortuga
    
    Returns:
        str: Nombre de la tortuga creada, o None si falla
    """
    # Crear nodo temporal
    temp_node = Node('temp_spawn_node')
    
    # Declarar parámetros con valores por defecto
    temp_node.declare_parameter('explorer_x', 2.0)
    temp_node.declare_parameter('explorer_y', 2.0)
    
    # Obtener valores
    x = temp_node.get_parameter('explorer_x').value
    y = temp_node.get_parameter('explorer_y').value
    
    # Validar límites
    if not (0.0 <= x <= 11.0 and 0.0 <= y <= 11.0):
        temp_node.get_logger().error(f'Invalid position: ({x}, {y}). Must be between 0 and 11')
        x = min(max(x, 0.0), 11.0)
        y = min(max(y, 0.0), 11.0)
        temp_node.get_logger().warn(f'Adjusted to: ({x}, {y})')
    
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
    rclpy.init(args=args)
    spawn_explorer()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
