import rclpy
from rclpy.node import Node
from basic_pubsub_interfaces.msg import TempReading


class TemperatureSubscriber(Node):
    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(
            TempReading, 'temperature', self.listener_callback, 10)
        self.get_logger().info('Temperature subscriber node initialized')

    def listener_callback(self, msg: TempReading):
        self.get_logger().info(
            f'Received temperature: {msg.temperature} {msg.unit} at timestamp {msg.timestamp}')


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
