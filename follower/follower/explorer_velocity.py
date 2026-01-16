"""Controlador de velocidad para la tortuga exploradora.

Este módulo implementa un nodo que calcula y publica comandos de velocidad
para que la tortuga exploradora persiga a turtle1 usando un controlador
prosporcional simple basado en distancia y error angular.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist

from math import atan2, cos, sin, sqrt

from .shared_poses import SharedPoses

class ExplorerVelocity(Node):
    """Nodo ROS2 que controla la velocidad del explorador para seguir a turtle1.
    
    Implementa un controlador proporcional que calcula velocidades lineales y
    angulares basadas en la distancia y el ángulo hacia turtle1.
    
    Attributes:
        shared_poses (SharedPoses): Objeto compartido para acceder a las poses.
        group (ReentrantCallbackGroup): Grupo de callbacks reentrantes.
        explorer_velocity_changer (Publisher): Publicador de comandos de velocidad.
        explorer_velocity_timer (Timer): Timer para actualización periódica.
    """
    
    def __init__(self, shared_poses: SharedPoses):
        """Inicializa el controlador de velocidad del explorador.
        
        Args:
            shared_poses (SharedPoses): Instancia de SharedPoses para acceder a las poses.
        """
        super().__init__('explorer_velocity')

        self.shared_poses = shared_poses

        # ========== REENTRANT CALLBACK GROUP ==========
        # En este nodo tenemos un publisher y un timer. Aunque el timer se ejecuta cada 50ms
        # (bastante rápido), usamos ReentrantCallbackGroup por estas razones:
        #
        # 1. Consistencia con el resto del sistema que usa MultiThreadedExecutor
        # 2. Si el cálculo de velocidad se retrasa, el siguiente tick del timer puede comenzar
        # 3. Evita bloqueos si en el futuro añadimos más callbacks a este nodo
        #
        # Sin ReentrantCallbackGroup:
        # - Si un tick del timer tarda >50ms, el siguiente tick esperaría
        # - Posible acumulación de retrasos
        #
        # Con ReentrantCallbackGroup:
        # - El timer puede ejecutarse incluso si el tick anterior no ha terminado
        # - Mejor control de tiempo para las actualizaciones de velocidad
        #
        # NOTA: Como leemos de shared_poses, el lock garantiza que obtenemos
        # una copia consistente de las poses.
        self.group = ReentrantCallbackGroup()

        self.explorer_velocity_changer = self.create_publisher(
            Twist,
            "/explorer/cmd_vel",
            10,
            callback_group=self.group
        )
        # Timer a 20 Hz (0.05 segundos = 50ms)
        # Frecuencia empírica: suficientemente rápida para control suave pero no excesiva
        # Menos frecuente (ej: 0.1s) → movimiento entrecortado
        # Más frecuente (ej: 0.01s) → carga innecesaria sin mejora notable
        self.explorer_velocity_timer = self.create_timer(
            0.05,
            self.update_explorer_velocity,
            callback_group=self.group
        )

        self.explorer_velocity_changer # prevent unused variable warning
        self.explorer_velocity_timer # prevent unused variable warning

    def update_explorer_velocity(self):
        """Calcula y publica la velocidad del explorador para perseguir a turtle1.
        
        Implementa un controlador proporcional que:
        - Velocidad lineal proporcional a la distancia (con modulación por coseno del error angular)
        - Velocidad angular proporcional al error angular
        
        La velocidad se publica en el topic /explorer/cmd_vel.
        """
        turtle_pose, explorer_pose = self.shared_poses.get_poses()

        vel_msg = Twist()

        # ========== ALGORITMO DE CONTROL DE SEGUIMIENTO ==========
        # Este controlador hace que el explorador persiga a turtle1 usando geometría 2D
        
        # 1. Calcular la distancia euclidiana entre las tortugas
        distance = sqrt((turtle_pose.x - explorer_pose.x) ** 2 +
                        (turtle_pose.y - explorer_pose.y) ** 2)

        # 2. Calcular el ángulo hacia turtle1 usando atan2
        # atan2(dy, dx) devuelve el ángulo en radianes [-π, π] desde el explorer hacia turtle1
        # Esto nos dice "hacia dónde debería mirar" el explorer para ver a turtle1
        angle_to_turtle = atan2(turtle_pose.y - explorer_pose.y,
                                turtle_pose.x - explorer_pose.x)
        
        # 3. Calcular el error angular (diferencia entre orientación actual y deseada)
        # explorer_pose.theta es hacia dónde mira actualmente el explorer
        # angle_to_turtle es hacia dónde DEBERÍA mirar
        # El truco de atan2(sin(diff), cos(diff)) normaliza el ángulo al rango [-π, π]
        # evitando problemas cuando cruzamos de -π a π (ej: -3.14 vs 3.14 son casi iguales)
        angle_error = atan2(sin(angle_to_turtle - explorer_pose.theta),
                            cos(angle_to_turtle - explorer_pose.theta))
        
        # 4. VELOCIDAD LINEAL (hacia adelante/atrás)
        # - Proporcional a la distancia: más lejos = más rápido
        # - Multiplicado por cos(angle_error): si no está mirando hacia turtle1, reduce velocidad
        #   * Si angle_error = 0° (mirando directo), cos(0) = 1.0 → velocidad máxima
        #   * Si angle_error = 90° (perpendicular), cos(90°) = 0 → se detiene
        #   * Si angle_error = 180° (mirando opuesto), cos(180°) = -1 → retrocede
        # - Ganancia 1.0: valor empírico que funciona bien en turtlesim
        vel_msg.linear.x = 1.0 * distance * cos(angle_error)
        
        # 5. VELOCIDAD ANGULAR (giro)
        # - Proporcional al error angular: cuanto más desalineado, gira más rápido
        # - Ganancia 4.0: valor empírico para que gire rápido sin oscilar demasiado
        #   (valores menores = giro lento, valores mayores = posible oscilación)
        vel_msg.angular.z = 4.0 * angle_error

        self.explorer_velocity_changer.publish(vel_msg)
        self.get_logger().debug(f'Published explorer velocity: linear={vel_msg.linear.x}, angular={vel_msg.angular.z}')


def main(args=None):
    """Función main standalone para testing del controlador de velocidad.
    
    Inicializa el sistema con PoseSaver y ExplorerVelocity para pruebas
    independientes del controlador de velocidad.
    
    Args:
        args: Argumentos de línea de comandos para rclpy.
    Note:
        Se recomienda no usar el módulo como programa principal en despliegues
        reales, sino solo para pruebas independientes.
    """
    from rclpy.executors import MultiThreadedExecutor

    from .create_explorer import spawn_explorer
    from .pose_savers import PoseSaver

    rclpy.init(args=args)
    
    spawn_explorer()

    shared_poses = SharedPoses()
    
    pose_saver = PoseSaver(shared_poses)
    explorer_velocity = ExplorerVelocity(shared_poses)
    
    # MultiThreadedExecutor permite que los callbacks de suscripción (actualización de poses)
    # y el timer (cálculo de velocidad) se ejecuten en paralelo sin bloquearse mutuamente.
    # Ver comentarios detallados en follower.py.
    executor = MultiThreadedExecutor()
    executor.add_node(pose_saver)
    executor.add_node(explorer_velocity)

    try:
        executor.spin()
    finally:
        pose_saver.destroy_node()
        explorer_velocity.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()