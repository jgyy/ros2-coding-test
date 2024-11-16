import rclpy
from rclpy.node import Node
from basic_pubsub.srv import TempConvert
import sys


class TemperatureConversionClient(Node):
    def __init__(self):
        super().__init__('temperature_conversion_client')
        self.cli = self.create_client(TempConvert, 'convert_temperature')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = TempConvert.Request()

    def send_request(self, input_unit, output_unit, temperature):
        self.req.input_unit = input_unit
        self.req.output_unit = output_unit
        self.req.temperature = temperature
        return self.cli.call_async(self.req)


def main(args=None):
    rclcp.init(args=args)
    client = TemperatureConversionClient()
    future = client.send_request('celsius', 'fahrenheit', 25.0)
    rclpy.spin_until_future_complete(client, future)
    try:
        result = future.result()
        print(f'Result: {result.converted_temperature} ({result.status})')
    except Exception as e:
        client.get_logger().error(f'Service call failed: {str(e)}')
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
