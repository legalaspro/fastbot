#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node 
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time as TimeMsg

def stamp_ns(stamp: TimeMsg) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ns_to_stamp(ns: int) -> TimeMsg:
    sec = ns // 1_000_000_000
    nanosec = ns % 1_000_000_000
    return TimeMsg(sec=int(sec), nanosec=int(nanosec))

class TimestampFilterNode(Node):
    def __init__(self):
        super().__init__('timestamp_filter_node')

        # Parameters 
        self.declare_parameter('scan_input', '/fastbot_1/scan')
        self.declare_parameter('scan_output', '/scan')
        self.declare_parameter('odom_input', '/fastbot_1/odom')
        self.declare_parameter('odom_output', '/odom')

        scan_in = self.get_parameter('scan_input').value
        scan_out = self.get_parameter('scan_output').value
        odom_in = self.get_parameter('odom_input').value
        odom_out = self.get_parameter('odom_output').value

        # QoS
        # /scan is sensor data: best effort + small queue is typical
        scan_qos = qos_profile_sensor_data

        # /odom is usually reliable; keep last few
        odom_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        #Publishers
        self.scan_pub = self.create_publisher(LaserScan, scan_out, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_out, 10)
        self.create_subscription(LaserScan, scan_in, self.scan_callback, scan_qos)
        self.create_subscription(Odometry, odom_in, self.odom_callback, odom_qos)

        # Last timestamps as integer ns
        self.last_scan_ns = -1
        self.last_odom_ns = -1

        self.get_logger().info(
            f"Timestamp filter ready. policy=nudge "
            f"scan: {scan_in}->{scan_out} odom: {odom_in}->{odom_out}"
        )

    def _fix_stamp(self, current_ns: int, last_ns: int):
        """Returns (fixed_ns)."""
        if last_ns < 0 or current_ns > last_ns:
            return current_ns

        # Nudge forward by 1ns to keep monotonic time
        fixed = last_ns + 1

        return fixed
    
    def scan_callback(self, msg: LaserScan):
        cur = stamp_ns(msg.header.stamp)
        fixed = self._fix_stamp(cur, self.last_scan_ns)
        if fixed != cur:
            msg.header.stamp = ns_to_stamp(fixed)
        
        self.last_scan_ns = fixed
        self.scan_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        cur = stamp_ns(msg.header.stamp)
        fixed = self._fix_stamp(cur, self.last_odom_ns)
        if fixed != cur:
            msg.header.stamp = ns_to_stamp(fixed)

        self.last_odom_ns = fixed
        self.odom_pub.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = TimestampFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

