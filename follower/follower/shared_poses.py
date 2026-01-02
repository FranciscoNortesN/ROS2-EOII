import threading
from copy import deepcopy
from turtlesim.msg import Pose

class SharedPoses():
    
    def __init__(self):
        self._turtle_pose = Pose()
        self._explorer_pose = Pose()
        self._turtle_initialized = False
        self._explorer_initialized = False
        # Se usa threading.Lock en lugar de un Reader-Writer Lock porque
        # las operaciones de lectura/escritura son muy rápidas (deepcopy de Pose)
        # y el overhead de RWLock no justifica la complejidad adicional
        self.lock = threading.Lock()
    
    def set_turtle_pose(self, pose: Pose):
        with self.lock:
            self._turtle_pose = deepcopy(pose)
            self._turtle_initialized = True

    def set_explorer_pose(self, pose: Pose):
        with self.lock:
            self._explorer_pose = deepcopy(pose)
            self._explorer_initialized = True

    def set_pose(self, turtle: Pose, explorer: Pose):
        with self.lock:
            self._turtle_pose = deepcopy(turtle)
            self._explorer_pose = deepcopy(explorer)

    def get_turtle_pose(self) -> Pose:
        with self.lock:
            return deepcopy(self._turtle_pose)

    def get_explorer_pose(self) -> Pose:
        with self.lock:
            return deepcopy(self._explorer_pose)

    def get_poses(self) -> tuple[Pose, Pose]:
        with self.lock:
            return deepcopy(self._turtle_pose), deepcopy(self._explorer_pose)
    
    def are_poses_initialized(self) -> bool:
        with self.lock:
            return self._turtle_initialized and self._explorer_initialized
