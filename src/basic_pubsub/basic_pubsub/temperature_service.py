import rclpy
from rclpy.node import Node
from basic_pubsub.srv import TempConvert


class TemperatureConversionService(Node):
    def __init__(self):
        super().__init__('temperature_conversion_service')
        self.srv = self.create_service(
            TempConvert, 'convert_temperature',self.convert_temperature_callback)
        self.get_logger().info('Temperature conversion service initialized')

    def convert_temperature_callback(self, request, response):
        try:
            if request.input_unit == request.output_unit:
                response.converted_temperature = request.temperature
                response.status = 'success: units are the same'
                return response
            if request.input_unit == 'celsius' and request.output_unit = 'fahrenheit':
                response.converted_temperature = (request.temperature * 9 / 5) + 32
                response.status = 'success: converted C to F'
            elif request.input_unit == 'fahrenheit' and request.output_unit == 'celsius':
                response.converted_temperature = (request.temperature - 32) * 5 / 9
                response.status = 'success: converted F to C'
            else:
                response.converted_temperature = float('nan')
                response.status = f'error: unsupported conversion from {request.input_unit} to {request.output_unit}'
        except Exception as e:
            response.converted_temperature = float('nan')
            response.status = f'error: {str(e)}'
        return response


def main(args=None):
    rclcp.init(args=args)
    node = TemperatureConversionService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
