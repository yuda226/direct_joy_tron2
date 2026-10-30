import math
import os
import signal
import subprocess
import tempfile
import time
import unittest
import uuid

import rclpy
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Joy


class TestStandaloneLaunch(unittest.TestCase):
    def test_hold_release_turbo_timeout_and_invalid_input(self):
        context = Context()
        context.init(domain_id=191)
        node = rclpy.create_node("direct_joy_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        topic = "/direct_joy_test_" + uuid.uuid4().hex
        received = []
        publisher = node.create_publisher(Joy, topic + "/joy", 10)
        subscription = node.create_subscription(
            Twist, topic + "/cmd_vel", received.append, 10
        )
        env = dict(os.environ, ROS_DOMAIN_ID="191", ROS_LOCALHOST_ONLY="1")
        with tempfile.TemporaryFile(mode="w+") as log:
            process = subprocess.Popen(
                [
                    "ros2", "launch", "direct_joy", "direct_joy.launch.py",
                    "start_joy:=false",
                    "joy_topic:=" + topic + "/joy",
                    "cmd_vel_topic:=" + topic + "/cmd_vel",
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                env=env,
            )
            try:
                deadline = time.monotonic() + 15.0
                while (
                    publisher.get_subscription_count() == 0
                    or subscription.get_publisher_count() == 0
                    or not received
                ):
                    self.assertIsNone(process.poll(), "Launch exited before discovery")
                    self.assertLess(time.monotonic(), deadline, "Discovery timed out")
                    executor.spin_once(timeout_sec=0.05)

                def send(joy):
                    received.clear()
                    deadline = time.monotonic() + 0.3
                    while time.monotonic() < deadline:
                        publisher.publish(joy)
                        executor.spin_once(timeout_sec=0.02)
                    self.assertTrue(received, "No cmd_vel received")
                    return received[-1]

                joy = Joy(axes=[0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                          buttons=[0] * 11)
                self.assertEqual(send(joy).linear.x, 0.0)
                joy.buttons[5] = 1
                joy.axes[1] = 1.0
                joy.axes[3] = -0.5
                command = send(joy)
                self.assertAlmostEqual(command.linear.x, 1.0 / 1.2)
                self.assertAlmostEqual(command.angular.z, -0.5 / 1.2)
                self.assertEqual(command.linear.y, 0.0)
                joy.buttons[5] = 0
                self.assertEqual(send(joy).linear.x, 0.0)

                joy.axes[1] = 0.0
                joy.axes[3] = 0.0
                send(joy)
                joy.buttons[5] = 1
                joy.axes[1] = 1.0
                joy.axes[2] = -1.0
                self.assertAlmostEqual(send(joy).linear.x, 1.0)
                deadline = time.monotonic() + 0.7
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.02)
                self.assertEqual(received[-1].linear.x, 0.0)
                self.assertEqual(send(joy).linear.x, 0.0)

                joy.buttons[5] = 0
                joy.axes[1] = 0.0
                send(joy)
                joy.buttons[5] = 1
                joy.axes[1] = 1.0
                self.assertAlmostEqual(send(joy).linear.x, 1.0)
                joy.axes[1] = math.nan
                self.assertEqual(send(joy).linear.x, 0.0)
                self.assertIsNone(process.poll())
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
                executor.shutdown()
                node.destroy_node()
                context.shutdown()
                log.seek(0)
                print(log.read())
        self.assertEqual(process.returncode, 0)


if __name__ == "__main__":
    unittest.main()
