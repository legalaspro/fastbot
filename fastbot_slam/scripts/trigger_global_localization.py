#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

class GlobalLocTrigger(Node):
    def __init__(self):
        super().__init__('global_loc_trigger')

        # Parameters
        self.declare_parameter('vel_topic', '/fastbot/cmd_vel')
        self.declare_parameter('spin_duration', 10.0)
        self.declare_parameter('angular_speed', 0.4)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("service_name", "/reinitialize_global_localization")

        vel_topic = self.get_parameter("vel_topic").value
        self.spin_duration = float(self.get_parameter("spin_duration").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.service_name = str(self.get_parameter("service_name").value)

        self.cmd_pub = self.create_publisher(Twist, vel_topic, 10)
        self.cli = self.create_client(Empty, self.service_name)

        self._spin_timer = None
        self._service_check_timer = self.create_timer(0.2, self._check_service_ready)

        self._twist = Twist()
        self._twist.angular.z = self.angular_speed

        self._spin_end_time = None

        self.get_logger().info(f"Waiting for {self.service_name} ...")

    def _check_service_ready(self):
        if not self.cli.service_is_ready():
            return

        self._service_check_timer.cancel()
        self.get_logger().info(f"{self.service_name} is ready. Triggering global localization...")

        future = self.cli.call_async(Empty.Request())
        future.add_done_callback(self._on_global_loc_done)
    
    def _on_global_loc_done(self, future):
        try:
            future.result()
            self.get_logger().info("Global localization triggered.")
        except Exception as e:
            self.get_logger().warn(f"Global localization call failed: {e}")

        # Start spinning using ROS time (works with use_sim_time)
        if self.spin_duration <= 0.0:
            self.get_logger().info("spin_duration <= 0, skipping spin.")
            self._stop_and_exit()
            return

        now = self.get_clock().now()
        self._spin_end_time = now + Duration(seconds=self.spin_duration)

        period = 1.0 / max(self.publish_hz, 1e-6)
        self.get_logger().info(f"Spinning for {self.spin_duration:.2f}s at {self.publish_hz:.1f} Hz ...")
        self._spin_timer = self.create_timer(period, self._spin_step)

    def _spin_step(self):
        now = self.get_clock().now()
        if now < self._spin_end_time:
            self.cmd_pub.publish(self._twist)
            return

        self._stop_and_exit()

    def _stop_and_exit(self):
        if self._spin_timer is not None:
            self._spin_timer.cancel()

        # stop robot
        self.cmd_pub.publish(Twist())
        self.get_logger().info("Global localization + initial spin complete. Stopping.")
        rclpy.shutdown()
     
def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocTrigger()
    try:
        rclpy.spin(node)
    finally:
        # extra safety stop
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()

if __name__ == '__main__':
    main()