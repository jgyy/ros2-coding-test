import rclpy
from rclpy.node import Node
from basic_pubsub_interfaces.msg import TempReading
import random
import time


class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(TempReading, 'temperature', 10)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('temp_min', 20.0)
        self.declare_parameter('temp_max', 30.0)
        self.declare_parameter('default_unit', 'celsius')
        timer_period = 1.0 / self.get_parameter('publish_rate').value
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Temperature publisher node initialized')

    def timer_callback(self):
        msg = TempReading()
        temp_min = self.get_parameter('temp_min').value
        temp_max = self.get_parameter('temp_max').value
        msg.temperature = random.uniform(temp_min, temp_max)
        msg.unit = self.get_parameter('default_unit').value
        msg.timestamp = int(time.time())
        self.publisher_.publish(msg)
        self.get_logger().debug(
            f'Publishing temperature: {msg.temperature} {msg.unit}')


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
