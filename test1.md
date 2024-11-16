# ROS2 Coding Interview Test

## Duration: 2 hours
## Tools Allowed: Official ROS2 documentation

### Section 1: Basic ROS2 Concepts (20 minutes)

1. Create a simple ROS2 package named `basic_pubsub` with the following requirements:
   - Implement a publisher node that publishes random temperature readings
   - Implement a subscriber node that receives and processes these readings
   - Use a custom message type `TempReading` with fields: `float32 temperature`, `string unit`, `int64 timestamp`

Example solution structure:
```bash
ros2 pkg create --build-type ament_python basic_pubsub
cd basic_pubsub
# Create necessary files and implement nodes
```

### Section 2: Services and Parameters (30 minutes)

2. Extend the previous package to include:
   - A service server that provides temperature conversion (Celsius to Fahrenheit and vice versa)
   - Configurable parameters for:
     - Publishing rate
     - Temperature range limits
     - Unit preference

Example service definition (temp_convert.srv):
```
string input_unit
string output_unit
float32 temperature
---
float32 converted_temperature
string status
```

### Section 3: Action Server Implementation (40 minutes)

3. Create a new package `robot_monitor` that implements:
   - An action server for monitoring robot battery levels
   - The action should simulate battery charging over time
   - Provide feedback during the charging process
   - Allow for goal cancellation

Example action definition (Charge.action):
```
# Goal
float32 target_battery_level
---
# Result
float32 final_battery_level
duration total_charging_time
---
# Feedback
float32 current_battery_level
float32 charging_rate
```

### Section 4: Launch File and Composition (30 minutes)

4. Create a launch file that:
   - Starts all previously created nodes
   - Configures node parameters through the launch file
   - Implements proper namespacing
   - Includes error handling and logging configuration

Example launch file structure:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Add launch arguments and node configurations here
    pass
```

### Bonus Challenge: Integration Test

5. Write an integration test that:
   - Verifies the publisher-subscriber communication
   - Tests the temperature conversion service
   - Validates the charging action server behavior

Example test structure:
```python
import unittest
import rclpy
from rclpy.node import Node
# Add necessary imports

class TestBasicPubSub(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # Setup test node and dependencies

    def test_temp_publication(self):
        # Implement test logic
        pass

    @classmethod
    def tearDownClass(cls):
        # Cleanup
        pass
```

## Evaluation Criteria

Candidates will be evaluated on:

1. Code Quality (30%)
   - Clean, well-documented code
   - Proper error handling
   - Efficient implementation

2. ROS2 Knowledge (30%)
   - Correct usage of ROS2 concepts
   - Proper node lifecycle management
   - Understanding of ROS2 communication patterns

3. Problem Solving (20%)
   - Approach to implementing requirements
   - Solution architecture
   - Performance considerations

4. Testing & Validation (20%)
   - Test coverage
   - Error case handling
   - Integration testing approach

## Solution Guidelines

### Expected Directory Structure:
```
workspace/
├── src/
│   ├── basic_pubsub/
│   │   ├── basic_pubsub/
│   │   │   ├── __init__.py
│   │   │   ├── publisher_node.py
│   │   │   └── subscriber_node.py
│   │   ├── test/
│   │   ├── setup.py
│   │   └── package.xml
│   └── robot_monitor/
│       ├── robot_monitor/
│       │   ├── __init__.py
│       │   └── action_server.py
│       ├── test/
│       ├── setup.py
│       └── package.xml
└── launch/
    └── system.launch.py
```

### Key Implementation Requirements:

1. All nodes must implement proper lifecycle management
2. Use of ROS2 logging for debugging and error reporting
3. Proper parameter validation and error handling
4. Implementation of clean shutdown procedures
5. Use of type hints and proper documentation
6. Adherence to ROS2 coding style guidelines

### Sample Node Implementation:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Temperature publisher node initialized')

    def timer_callback(self):
        msg = Float32()
        msg.data = random.uniform(20.0, 30.0)
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Publishing temperature: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Notes

- Candidates should focus on demonstrating their understanding of ROS2 fundamentals rather than implementing complex algorithms
- Code should be written with maintainability and scalability in mind
- Proper use of ROS2 tools (ros2 CLI, rqt, etc.) during development is encouraged
- Documentation should include setup instructions and dependencies
- Bonus points for implementing unit tests and continuous integration setup
  