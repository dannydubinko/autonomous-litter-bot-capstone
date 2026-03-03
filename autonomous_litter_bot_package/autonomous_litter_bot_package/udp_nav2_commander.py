import rclpy
import math
import socket
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

# Import your parent class and compiled protobuf
from autonomous_litter_bot_package.pose_tracker import PoseTracker
try:
    from . import goal_position_pb2
except (ImportError, ValueError):
    # Fallback for when running as a standalone script
    import goal_position_pb2

class UdpNav2Commander(PoseTracker):
    def __init__(self, udp_ip="0.0.0.0", udp_port=25001):
        # 1. Initialize parent PoseTracker (subscribes to /amcl_pose)
        super().__init__()
        
        # 2. Setup Nav2 Client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 3. State Tracking: Stores (x, y, theta) of the last goal sent
        self._last_sent_goal = None 
        
        # 4. Setup Non-Blocking UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((udp_ip, udp_port))
        self.sock.setblocking(False) # Prevents freezing ROS 2 callbacks
        
        # 5. Setup Polling Timer (Checks UDP socket 10 times a second)
        self.poll_timer = self.create_timer(0.1, self.poll_udp_socket)
        self.get_logger().info(f"Listening for Protobuf on UDP {udp_ip}:{udp_port}...")

    def poll_udp_socket(self):
        """Rapidly checks for a UDP message without stopping ROS."""
        try:
            data, addr = self.sock.recvfrom(1024) 
            self.process_udp_goal(data)
        except BlockingIOError:
            # No data right now, pass cleanly
            pass
        except Exception as e:
            self.get_logger().error(f"UDP Error: {e}")

    def process_udp_goal(self, data):
        """Deserializes Protobuf, calculates absolute pose, and checks if it is new."""
        goal_msg = goal_position_pb2.GoalPosition()
        try:
            goal_msg.ParseFromString(data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse Protobuf: {e}")
            return
            
        # Get current absolute position from the parent class
        current_pos = self.capture_my_position_now()
        if current_pos is None:
            return
            
        curr_x, curr_y, curr_theta_deg = current_pos
        curr_theta_rad = math.radians(curr_theta_deg)

        # Calculate Absolute Target Position based on current heading
        rel_x = goal_msg.x
        rel_y = goal_msg.y
        rel_theta = goal_msg.theta
        
        target_x = curr_x + (rel_x * math.cos(curr_theta_rad) - rel_y * math.sin(curr_theta_rad))
        target_y = curr_y + (rel_x * math.sin(curr_theta_rad) + rel_y * math.cos(curr_theta_rad))
        target_theta_deg = curr_theta_deg + rel_theta

        # Check if this goal is significantly different from the last one
        if self._is_goal_different(target_x, target_y, target_theta_deg):
            
            # --- Clear, prominent terminal printout right before sending ---
            self.get_logger().info("\n" + "="*50)
            self.get_logger().info("🚀 NEW GOAL TRIGGERED")
            self.get_logger().info(f"📍 CURRENT: X: {curr_x:.3f} | Y: {curr_y:.3f} | Theta: {curr_theta_deg:.1f}°")
            self.get_logger().info(f"🎯 TARGET:  X: {target_x:.3f} | Y: {target_y:.3f} | Theta: {target_theta_deg:.1f}°")
            self.get_logger().info("="*50 + "\n")
            
            # Immediately save this so we don't spam the server on the next tick
            self._last_sent_goal = (target_x, target_y, target_theta_deg)
            
            # Send it to the action server
            self.send_absolute_goal(target_x, target_y, target_theta_deg)

    def _is_goal_different(self, t_x, t_y, t_theta):
        """Returns True if the new target is significantly different from the last sent target."""
        if self._last_sent_goal is None:
            return True 
            
        last_x, last_y, last_theta = self._last_sent_goal
        
        distance_diff = math.hypot(t_x - last_x, t_y - last_y)
        angle_diff = abs(t_theta - last_theta)
        
        # Tolerance: 5cm distance OR 2 degrees rotation
        if distance_diff > 0.05 or angle_diff > 2.0:
            return True
            
        return False

    def send_absolute_goal(self, x, y, theta_degrees):
        """Formats and sends the goal, attaching a callback to check acceptance."""
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 server not ready. Will try again next UDP packet.')
            self._last_sent_goal = None # Clear memory so it tries again
            return

        yaw_rad = math.radians(theta_degrees)
        # Using the utility method inherited from PoseTracker
        qx, qy, qz, qw = self.euler_to_quaternion(roll=0.0, pitch=0.0, yaw=yaw_rad)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        # Send goal asynchronously and attach the callback
        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Triggered when Nav2 responds to our goal request."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Nav2 REJECTED the goal! (Path blocked or invalid)')
            # Clear memory so UDP polling can attempt to send it again if requested
            self._last_sent_goal = None 
            return

        self.get_logger().info('✅ Goal ACCEPTED! Robot is navigating successfully.')


def main(args=None):
    rclpy.init(args=args)
    # Listen on port 5005 by default
    node = UdpNav2Commander(udp_port=25001)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()