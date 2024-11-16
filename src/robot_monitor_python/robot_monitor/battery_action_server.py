import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from robot_monitor_interfaces.action import Charge
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class BatteryActionServer(Node):
    def __init__(self):
        super().__init__('battery_action_server')
        self._action_server = ActionServer(self, Charge, 'charge_battery',
            self.execute_callback, callback_group=ReentrantCallbackGroup())
        self.current_battery = 20.0
        self.charging_rate = 10.0
        self.get_logger().info('Battery action server initialized')

    async def execute_callback(self):
        self.get_logger().info('Received charging goal')
        feedback_msg = Charge.Feedback()
        result = Charge.Result()
        start_time = time.time()
        if goal_handle.request.target_battery_level > 100.0:
            goal_handle.abort()
            return result
        while self.current_battery < goal_handle.request.target_battery_level:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.final_battery_level = self.current_battery
                result.total_charging_time = time.time() - start_time
                return result
            self.current_battery += (self.charging_rate / 60.0)
            if self.current_battery > 100.0:
                self.current_battery = 100.0
            feedback_msg.current_battery_level = self.current_battery
            feedback_msg.charging_rate = self.charging_rate
            goal_handle.publish_feedback(feedback_msg)
            await self.sleep(1)
        goal_handle.succeed()
        result.final_battery_level = self.current_battery
        result.total_charging_time = time.time() - start_time
        return result

    async def sleep(self, seconds):
        await self.create_rate(1 / seconds).timer.wait()


def main(args=None):
    rclcp.init(args=args)
    action_server = BatteryActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
