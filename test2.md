# ROS2 Robotics Interview Challenge

## Duration: 2.5 hours
## Tools Allowed: ROS2 Documentation, Python Standard Library Documentation

### Section 1: Sensor Data Processing (30 minutes)

1. Create a ROS2 package named `sensor_fusion` that implements:
   - A node that simulates multiple distance sensors (ultrasonic, infrared, lidar)
   - A fusion node that combines readings using weighted averaging
   - Custom message types for raw and processed sensor data

Required Message Definition (sensor_reading.msg):
```
uint32 sensor_id
string sensor_type
float64 distance
float64 confidence
time timestamp
```

Example node structure:
```python
class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        # Initialize subscribers for each sensor type
        # Create publisher for fused data
        # Set up parameter for fusion weights
```

### Section 2: Robot Navigation State Machine (40 minutes)

2. Develop a package `nav_state_machine` that implements:
   - A state machine for robot navigation using ROS2 lifecycles
   - States: IDLE, PLANNING, MOVING, OBSTACLE_DETECTED, ERROR
   - Support for emergency stop through a service call
   - State transition event logging

Required Service Definition (emergency_control.srv):
```
string command  # STOP, RESUME
---
bool success
string current_state
string message
```

Example State Implementation:
```python
from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import Transition

class NavigationLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('navigation_controller')
        # Initialize state machine
        # Set up services and publishers
```

### Section 3: Distributed Sensor Network (40 minutes)

3. Create a `sensor_network` package that:
   - Implements a discovery system for dynamic sensor nodes
   - Uses Quality of Service settings for reliable data delivery
   - Provides health monitoring for each sensor node
   - Implements data throttling based on network conditions

Example Network Monitor:
```python
class NetworkMonitor(Node):
    def __init__(self):
        super().__init__('network_monitor')
        self.declare_parameter('qos_reliability', 'RELIABLE')
        self.declare_parameter('health_check_interval', 1.0)
        # Initialize discovery system
        # Set up health monitoring
```

### Section 4: Data Recording and Replay (30 minutes)

4. Implement a package `data_recorder` that:
   - Records sensor data and state machine transitions to SQLite
   - Provides a service to query historical data
   - Implements data replay functionality with time controls
   - Handles data compression and storage management

Example Database Schema:
```sql
CREATE TABLE sensor_readings (
    id INTEGER PRIMARY KEY,
    timestamp DATETIME,
    sensor_id INTEGER,
    reading_type TEXT,
    value REAL,
    metadata JSON
);
```

Required Service Definition (query_history.srv):
```
time start_time
time end_time
string[] sensor_ids
---
sensor_reading[] readings
string status
```

### Section 5: Integration and Testing (10 minutes)

5. Create a launch file that:
   - Starts all system components with proper dependencies
   - Configures component parameters
   - Sets up necessary transformations
   - Implements system diagnostics

Example Launch Configuration:
```python
def generate_launch_description():
    config = LaunchConfiguration('config_file')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                get_package_share_directory('system_config'),
                'config',
                'default.yaml'
            )
        ),
        # Add node configurations and dependencies
    ])
```

## Evaluation Criteria

### 1. System Architecture (25%)
- Component isolation and modularity
- Inter-process communication design
- Resource management
- Scalability considerations

### 2. ROS2 Feature Utilization (25%)
- Appropriate use of ROS2 communication patterns
- QoS configuration for different use cases
- Lifecycle management
- Parameter handling

### 3. Real-world Considerations (25%)
- Error handling and recovery
- Performance optimization
- Resource monitoring
- Network resilience

### 4. Code Quality (25%)
- Clear documentation
- Type safety
- Testing approach
- Error handling

## Expected Project Structure
```
workspace/
├── src/
│   ├── sensor_fusion/
│   │   ├── sensor_fusion/
│   │   │   ├── __init__.py
│   │   │   ├── sensor_simulator.py
│   │   │   └── fusion_node.py
│   │   ├── msg/
│   │   │   └── SensorReading.msg
│   │   ├── test/
│   │   ├── setup.py
│   │   └── package.xml
│   ├── nav_state_machine/
│   │   ├── nav_state_machine/
│   │   │   ├── __init__.py
│   │   │   └── navigation_controller.py
│   │   ├── srv/
│   │   ├── test/
│   │   ├── setup.py
│   │   └── package.xml
│   ├── sensor_network/
│   │   └── [package contents]
│   └── data_recorder/
│       └── [package contents]
├── launch/
│   └── system.launch.py
└── config/
    └── default.yaml
```

## Implementation Requirements

### Code Style and Documentation
1. Use Python type hints throughout
2. Implement proper exception handling
3. Add docstrings for all classes and methods
4. Follow ROS2 naming conventions

### Testing Requirements
1. Unit tests for core functionality
2. Integration tests for node communication
3. Performance tests for data processing
4. Network resilience tests

### Example Node Implementation with Best Practices:

```python
from typing import List, Dict
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from custom_interfaces.msg import SensorReading

class SensorProcessor(Node):
    """
    Processes and validates sensor readings with configurable filtering.
    """
    def __init__(self) -> None:
        super().__init__('sensor_processor')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('filter_threshold', 0.5),
                ('sensor_timeout', 1.0),
                ('processing_rate', 10.0)
            ]
        )
        
        # QoS Profile
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # Publishers/Subscribers
        self._sensor_sub = self.create_subscription(
            Range,
            'raw_reading',
            self._sensor_callback,
            qos_profile=sensor_qos
        )
        
        self._processed_pub = self.create_publisher(
            SensorReading,
            'processed_reading',
            10
        )
        
        # Initialize state
        self._readings_buffer: List[float] = []
        self._last_processing_time: float = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info('Sensor processor initialized')
    
    def _sensor_callback(self, msg: Range) -> None:
        """
        Processes incoming sensor readings with validation.
        
        Args:
            msg: Raw sensor reading message
        """
        try:
            if self._validate_reading(msg.range):
                self._readings_buffer.append(msg.range)
                self._process_readings()
        except ValueError as e:
            self.get_logger().warning(f'Invalid reading: {e}')
    
    def _validate_reading(self, reading: float) -> bool:
        """
        Validates sensor reading against configured threshold.
        
        Args:
            reading: Sensor reading value
            
        Returns:
            bool: True if reading is valid
            
        Raises:
            ValueError: If reading is outside valid range
        """
        threshold = self.get_parameter('filter_threshold').value
        if reading < 0 or reading > threshold:
            raise ValueError(f'Reading {reading} outside valid range')
        return True

```

## Additional Notes

- Focus on demonstrating system design and architecture skills
- Show understanding of ROS2 best practices and patterns
- Consider real-world deployment challenges
- Prioritize reliability and maintainability
- Document any assumptions made during implementation

## Bonus Points

- Implementation of system resource monitoring
- CI/CD configuration
- Container deployment setup
- Performance profiling tools
- Security considerations
