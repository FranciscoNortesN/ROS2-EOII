"""Contenedor thread-safe para compartir poses entre nodos.

Este módulo proporciona una clase que permite el acceso seguro y concurrente
a las poses de ambas tortugas desde múltiples nodos usando threading.Lock.
"""

import threading
from copy import deepcopy
from turtlesim.msg import Pose

class SharedPoses():
    """Contenedor thread-safe para las poses de turtle1 y explorer.
    
    Utiliza threading.Lock para garantizar acceso seguro desde múltiples threads.
    Las poses se copian profundamente (deepcopy) para evitar condiciones de carrera.
    
    Attributes:
        _turtle_pose (Pose): Pose actual de turtle1.
        _explorer_pose (Pose): Pose actual del explorador.
        _turtle_initialized (bool): Indica si se ha recibido al menos una pose de turtle1.
        _explorer_initialized (bool): Indica si se ha recibido al menos una pose del explorador.
        _lock (Lock): Lock de threading para sincronización.
    """
    
    def __init__(self):
        """Inicializa el contenedor de poses con valores por defecto."""
        self._turtle_pose = Pose()
        self._explorer_pose = Pose()
        self._turtle_initialized = False
        self._explorer_initialized = False
        
        # ========== THREADING.LOCK PARA SINCRONIZACIÓN ==========
        # Este Lock es ESENCIAL cuando usamos MultiThreadedExecutor, porque múltiples
        # threads pueden intentar leer/escribir las poses simultáneamente:
        #
        # Escrituras simultáneas (sin lock):
        # - Thread 1: turtle_callback modifica _turtle_pose
        # - Thread 2: explorer_callback modifica _explorer_pose
        # - Ambos pueden ejecutarse al mismo tiempo (OK con lock)
        #
        # Lecturas durante escrituras (sin lock - PROBLEMA):
        # - Thread 1: turtle_callback está modificando _turtle_pose
        # - Thread 2: handle_turtle_info lee _turtle_pose a mitad de la modificación
        # - Resultado: datos inconsistentes o corruptos
        #
        # El Lock garantiza que:
        # 1. Solo un thread puede modificar las poses a la vez
        # 2. Las lecturas obtienen una copia completa y consistente
        # 3. No hay condiciones de carrera (race conditions)
        #
        # Se usa threading.Lock (en lugar de RWLock) porque las operaciones son
        # rápidas (deepcopy de Pose) y el overhead de RWLock no justifica la complejidad.
        # Con MultiThreadedExecutor sin Lock → BUGS aleatorios y difíciles de depurar
        # Con MultiThreadedExecutor con Lock → Thread-safety garantizado
        self._lock = threading.Lock()
    
    def set_turtle_pose(self, pose: Pose):
        """Establece la pose de turtle1 de forma thread-safe.
        
        Args:
            pose (Pose): Nueva pose de turtle1.
        """
        with self._lock:
            # deepcopy es ESENCIAL para thread-safety
            # Sin deepcopy: self._turtle_pose = pose (solo copia la referencia)
            # - Si el callback modifica 'pose' después, _turtle_pose también cambia
            # - Múltiples threads podrían modificar el mismo objeto simultáneamente
            # Con deepcopy: crea una copia independiente del objeto
            # - Los cambios en 'pose' no afectan a _turtle_pose
            # - Cada thread tiene su propia copia, sin interferencias
            self._turtle_pose = deepcopy(pose)
            self._turtle_initialized = True

    def set_explorer_pose(self, pose: Pose):
        """Establece la pose del explorador de forma thread-safe.
        
        Args:
            pose (Pose): Nueva pose del explorador.
        """
        with self._lock:
            # deepcopy para evitar que múltiples threads compartan la misma referencia
            # Ver comentario en set_turtle_pose para explicación detallada
            self._explorer_pose = deepcopy(pose)
            self._explorer_initialized = True

    def set_pose(self, turtle: Pose, explorer: Pose):
        """Establece ambas poses simultáneamente de forma thread-safe.
        
        Args:
            turtle (Pose): Nueva pose de turtle1.
            explorer (Pose): Nueva pose del explorador.
        """
        with self._lock:
            self._turtle_pose = deepcopy(turtle)
            self._explorer_pose = deepcopy(explorer)

    def get_turtle_pose(self) -> Pose:
        """Obtiene la pose actual de turtle1 de forma thread-safe.
        
        Returns:
            Pose: Copia profunda de la pose de turtle1.
        """
        with self._lock:
            # deepcopy en lectura también es necesario
            # Sin deepcopy: return self._turtle_pose (devuelve referencia)
            # - El llamador podría modificar _turtle_pose directamente
            # - Rompe el encapsulamiento y thread-safety
            # Con deepcopy: el llamador recibe su propia copia
            # - Puede modificarla libremente sin afectar el estado interno
            return deepcopy(self._turtle_pose)

    def get_explorer_pose(self) -> Pose:
        """Obtiene la pose actual del explorador de forma thread-safe.
        
        Returns:
            Pose: Copia profunda de la pose del explorador.
        """
        with self._lock:
            return deepcopy(self._explorer_pose)

    def get_poses(self) -> tuple[Pose, Pose]:
        """Obtiene ambas poses simultáneamente de forma thread-safe.
        
        Returns:
            tuple[Pose, Pose]: Tupla con copias profundas de (turtle_pose, explorer_pose).
        """
        with self._lock:
            return deepcopy(self._turtle_pose), deepcopy(self._explorer_pose)
    
    def are_poses_initialized(self) -> bool:
        """Verifica si ambas poses han sido inicializadas.
        
        Returns:
            bool: True si se ha recibido al menos una pose de cada tortuga.
        """
        with self._lock:
            return self._turtle_initialized and self._explorer_initialized
