# ROS2-EOII - Sistema de Seguimiento de Tortugas

Proyecto de Entornos Operativos para la Informática Industrial que implementa un sistema de seguimiento de tortugas usando ROS2 y turtlesim.

## 📝 Descripción

Este proyecto implementa un sistema ROS2 donde una tortuga "exploradora" persigue autónomamente a la tortuga por defecto de turtlesim (turtle1). El sistema proporciona:

- **Control de velocidad proporcional**: El explorador ajusta su velocidad según la distancia y orientación hacia turtle1
- **Servicio de información**: Consulta en tiempo real de poses, velocidades y distancia entre tortugas
- **Servidor de acción**: Monitoreo del progreso de captura con feedback continuo
- **Arquitectura multi-thread**: Uso de MultiThreadedExecutor para ejecución concurrente

## 📦 Estructura del Proyecto

```
ROS2-EOII/
├── follower/                    # Paquete principal (Python)
│   ├── follower/
│   │   ├── follower.py           # Punto de entrada principal
│   │   ├── pose_savers.py        # Guarda poses de ambas tortugas
│   │   ├── explorer_velocity.py  # Controlador de velocidad del explorador
│   │   ├── turtle_info_service.py # Servidor del servicio TurtleInfo
│   │   ├── catch_info_action.py  # Servidor de acción CatchInfo
│   │   ├── shared_poses.py       # Contenedor thread-safe de poses
│   │   ├── create_explorer.py    # Utilidad para crear el explorador
│   │   ├── follower_server_client.py  # Cliente del servicio
│   │   └── follower_action_client.py  # Cliente de acción
│   ├── launch/
│   │   ├── launch.xml                    # Launch básico
│   │   ├── launch_with_server_client.xml # Con cliente de servicio
│   │   └── launch_with_action_client.xml # Con cliente de acción
│   ├── package.xml
│   └── setup.py
└── follower_interfaces/        # Paquete de interfaces (CMake)
    ├── srv/
    │   └── TurtleInfo.srv        # Servicio de información de tortugas
    ├── action/
    │   └── CatchInfo.action      # Acción de monitoreo de captura
    ├── CMakeLists.txt
    └── package.xml
```

## ⚡ Requisitos

- **ROS2**: Humble, Jazzy o superior
- **Python**: 3.12+
- **Paquetes ROS2**:
  - `rclpy`
  - `turtlesim`
  - `geometry_msgs`

## 🔧 Compilación
```bash
# Compilar
cd ROS2-EOII
colcon build --packages-select follower_interfaces follower

# Source del workspace
source install/setup.bash
```

## 🚀 Uso

### Opción 1: Sistema Básico

Ejecuta el sistema completo de seguimiento:

```bash
ros2 launch follower launch.xml
```

Mueve turtle1 con las teclas de dirección:

```bash
ros2 run turtlesim turtle_teleop_key
```

### Opción 2: Con Cliente de Servicio

Además del seguimiento, muestra información cada segundo:

```bash
ros2 launch follower launch_with_server_client.xml
```

### Opción 3: Con Cliente de Acción

Monitorea el progreso de captura con feedback continuo:

```bash
ros2 launch follower launch_with_action_client.xml
```

### Ejecución Manual de Componentes

```bash
# Sistema principal
ros2 run turtlesim turtlesim_node
ros2 run follower follower --ros-args -p explorer_x:=3.0 -p explorer_y:=3.0

# Cliente de servicio (en otra terminal)
ros2 run follower follower_server_client

# Cliente de acción (en otra terminal)
ros2 run follower follower_action_client
```

## 🏛️ Arquitectura del Sistema

### Nodos

- **pose_saver**: Suscrito a `/turtle1/pose` y `/explorer/pose`, guarda las poses en SharedPoses
- **explorer_velocity**: Calcula y publica velocidades en `/explorer/cmd_vel` a 20 Hz
- **turtle_info_service**: Servidor del servicio `turtle_info`
- **catch_info_action**: Servidor de acción `catch_info`

### Interfaces Personalizadas

**TurtleInfo.srv** - Servicio que devuelve información completa de ambas tortugas:
- Posiciones (x, y)
- Orientaciones (theta)
- Velocidades (lineal, angular)
- Distancia entre tortugas

**CatchInfo.action** - Acción para monitorear el progreso de captura:
- Goal: Vacío (inicia el monitoreo)
- Result: `bool caught` (si se capturó)
- Feedback: Toda la información de TurtleInfo en tiempo real

### Concurrencia

El sistema usa **MultiThreadedExecutor** con **ReentrantCallbackGroup** para:
- Permitir que múltiples callbacks se ejecuten simultáneamente
- Evitar bloqueos entre suscripciones, timers, servicios y actions
- Garantizar thread-safety mediante `threading.Lock` en SharedPoses

## 🎮 Parámetros Configurables

- `explorer_x` (default: 2.0): Posición X inicial del explorador (rango: 0-11)
- `explorer_y` (default: 2.0): Posición Y inicial del explorador (rango: 0-11)

## 📊 Algoritmo de Control

El explorador usa un **controlador proporcional** simple:

- **Velocidad lineal**: `v = 1.0 * distancia * cos(error_angular)`
- **Velocidad angular**: `ω = 4.0 * error_angular`

Donde:
- `distancia`: Distancia euclidiana entre tortugas
- `error_angular`: Diferencia entre orientación actual y deseada (normalizada a [-π, π])
- Ganancias (1.0, 4.0): Valores empíricos ajustados para turtlesim

## 📝 Licencia

MIT License - Ver archivo LICENSE

## ✍️ Autores

Francisco Nortes Novikov - fnornov@etsinf.upv.es
Vicente Burdeus Sánchez - vbursan@etsinf.upv.es
