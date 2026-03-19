import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class BaseLinkScanFilter(Node):
    def __init__(self):
        super().__init__('base_link_scan_filter')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            LaserScan, '/scan_raw', self.scan_callback, 10)
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        # Bounding box in base_link frame (meters)
        self.x_min = -0.26
        self.x_max = 0.26
        self.y_min = -0.1524
        self.y_max = 0.1524

        # Cache the static transform — look it up once, reuse forever
        self._dx = None
        self._dy = None
        self._cos_yaw = None
        self._sin_yaw = None

        # Try to grab the transform immediately, retry on first scans if not ready
        self._tf_ready = False

    def _lookup_static_transform(self, frame_id):
        """Look up the laser->base_link transform and cache it. Returns True on success."""
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link',
                frame_id,
                rclpy.time.Time()  # latest available
            )
            self._dx = t.transform.translation.x
            self._dy = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            self._cos_yaw = math.cos(yaw)
            self._sin_yaw = math.sin(yaw)
            self._tf_ready = True
            self.get_logger().info(
                f'Cached transform: {frame_id} -> base_link '
                f'dx={self._dx:.3f} dy={self._dy:.3f} yaw={yaw:.3f}rad'
            )
            return True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'Waiting for transform {frame_id} -> base_link: {e}',
                throttle_duration_sec=2.0  # only warn every 2s, not every scan
            )
            return False

    def scan_callback(self, msg):
        # Grab transform once on first scan, then reuse cache forever
        if not self._tf_ready:
            if not self._lookup_static_transform(msg.header.frame_id):
                return  # not ready yet, skip scan

        n = len(msg.ranges)

        # Build angle array for all rays at once — numpy vectorised
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float32)

        # Mask out invalid readings
        valid = (
            np.isfinite(ranges) &
            (ranges >= msg.range_min) &
            (ranges <= msg.range_max)
        )

        # Polar -> Cartesian in laser frame (vectorised)
        x_laser = np.where(valid, ranges * np.cos(angles), 0.0)
        y_laser = np.where(valid, ranges * np.sin(angles), 0.0)

        # Transform to base_link frame (vectorised)
        x_base = x_laser * self._cos_yaw - y_laser * self._sin_yaw + self._dx
        y_base = x_laser * self._sin_yaw + y_laser * self._cos_yaw + self._dy

        # Find points inside the robot bounding box
        in_box = (
            valid &
            (x_base >= self.x_min) & (x_base <= self.x_max) &
            (y_base >= self.y_min) & (y_base <= self.y_max)
        )

        # Zero out filtered points
        ranges[~valid] = 0.0   # also zero out invalids
        ranges[in_box] = 0.0

        # Build output message — reuse input header and metadata
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.intensities = msg.intensities
        filtered_msg.ranges = ranges.tolist()

        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    scan_filter = BaseLinkScanFilter()
    try:
        rclpy.spin(scan_filter)
    except KeyboardInterrupt:
        pass
    scan_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()