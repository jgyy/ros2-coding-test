import unittest
import rclpy
from rclpy.node import Node
from basic_pubsub.msg import TempReading
from basic_pubsub.srv import TempConvert
from robot_monitor.action import Charge
from rclpy.action import ActionClient
import time


class TestIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_node')

    def setUp(self):
        self.temp_readings = []
        self.subscription = self.node.create_subscription(TempReading, 'temperature',
            self.temp_callback, 10)
        self.client = self.node.create_client(TempConvert, 'convert_temperature')
        self.action_client = ActionClient(self.node, Charge, 'charge_battery')

    def temp_callback(self, msg):
        self.temp_readings.append(msg)

    def test_publisher_subscriber(self):
        time.sleep(2)
        self.assertGreater(len(self.temp_readings), 0)
        last_msg = self.temp_readings[-1]
        self.assertIsInstance(last_msg.temperature, float)
        self.assertIsInstance(last_msg.unit, str)
        self.assertIsInstance(last_msg.timestamp, int)

    def test_temperature_service(self):
        self.assertTrue(self.client.wait_for_service(timeout_sec=1.0))
        request = TempConvert.Request()
        request.input_unit = 'celsius'
        request.output_unit = 'fahrenheit'
        request.temperature = 0.0
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        self.assertAlmostEqual(response.converted_temperature, 32.0)
        self.assertEqual(response.status.split(':')[0], 'success')

    def test_battery_action(self):
        self.assertTrue(self.action_client.wait_for_server(timeout_sec=1.0))
        goal_msg = Charge.Goal()
        goal_msg.target_battery_level = 30.0
        goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future)
        goal_handle = goal_future.result()
        self.assertTrue(goal_handle.accepted)
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        self.assertGreaterEqual(
            result.final_battery_level, goal_msg.target_battery_level)

    def tearDown(self):
        self.node.destroy_subscription(self.subscription)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
