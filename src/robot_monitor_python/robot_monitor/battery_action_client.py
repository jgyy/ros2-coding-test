import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_monitor_interfaces.action import Charge
import sys


class BatteryActionClient(Node):
    def __init__(self):
        super().__init__('battery_action_client')
        self._action_client = ActionClient(self, Charge, 'charge_battery')

    def send_goal(self, target_level):
        goal_msg = Charge.Goal()
        goal_msg.target_battery_level = target_level
        self._action_client.wait_for_server()
        self.get_logger().info('Sending charging goal')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f'Final battery level: {result.final_battery_level}%,'
            f'Charging time: {result.total_charging_time} seconds')
        rclpy.shutdown()

    def feedback_callback(self,feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Current level: {feedback.current_battery_level}%, '
            f'Charging rate: {feedback.charging_rate}%/min')


def main(args=None):
    rclcp.init(args=args)
    action_client = BatteryActionClient()
    action_client.send_goal(90.0)
    rclpy.spin(action_client)
    action_server.destroy_node()


if __name__ == '__main__':
    main()
