import rclpy
import time
import socket
import threading
from rclpy.node import Node

# ROS 2 Messages
from action_msgs.msg import GoalStatusArray
from ros2_unitree_legged_msgs.msg import HighCmd

# Compiled Protobuf Module
try:
    from . import states_pb2
except (ImportError, ValueError):
    # Fallback for when running as a standalone script
    import states_pb2

# --- UDP Configuration ---
PI4_IP = '192.168.12.128'
PI4_PORT = 25003           # Port the Pi 4 is listening on
LOCAL_LISTEN_PORT = 25002  # Port this Pi 5 listens on for the Pi 4's reply

class Pi5Go1Commander(Node):
    def __init__(self):
        super().__init__('pi5_go1_commander')

        # State Machine & Goal Tracking
        self.state = 'NAVIGATION' # States: NAVIGATING, EXECUTING_SIT, WAITING_FOR_PICKUP, EXECUTING_STAND
        self.last_completed_goal_id = None

        # ROS 2 Publisher for the Go1's body
        self._cmd_pub = self.create_publisher(HighCmd, '/high_cmd', 10)
        
        # ROS 2 Subscriber for Nav2's action status
        self.create_subscription(
            GoalStatusArray, 
            '/navigate_to_pose/_action/status', 
            self.nav_status_callback, 
            10
        )

        # Setup UDP Sockets
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv.bind(('0.0.0.0', LOCAL_LISTEN_PORT))

        # Start a background thread to listen for the Pi 4 without blocking ROS 2
        self.listener_thread = threading.Thread(target=self.udp_listen_loop, daemon=True)
        self.listener_thread.start()

        self.get_logger().info('Pi 5 Go1 Commander Ready. State: NAVIGATION')

    def nav_status_callback(self, msg: GoalStatusArray):
        # Ignore navigation updates unless we are actively navigating
        if self.state != 'NAVIGATING' or not msg.status_list:
            return

        # Get the latest goal from the array
        latest_goal = msg.status_list[-1]
        goal_id = bytes(latest_goal.goal_info.goal_id.uuid).hex()
        status = latest_goal.status

        # Status 4 = SUCCEEDED
        if status == 4 and goal_id != self.last_completed_goal_id:
            self.get_logger().info(f'Nav2 Goal Reached! ID: {goal_id}')
            self.last_completed_goal_id = goal_id
            self.state = 'EXECUTING_SIT'
            
            # Run the physical sit sequence in a thread so we don't block the ROS 2 executor
            threading.Thread(target=self.execute_sit_sequence, daemon=True).start()

    def execute_sit_sequence(self):
        self.get_logger().info('Parking the Go1...')
        self._send_cmd_stream(mode=2, gait=0, duration=1.0) # Walk mode (prep)
        self._send_cmd_stream(mode=5, gait=0, duration=2.0) # Sit mode
        
        # Create and serialize the Protobuf message
        cmd_msg = states_pb2.States()
        cmd_msg.action = "sit"
        serialized_data = cmd_msg.SerializeToString()

        self.get_logger().info(f'Sending "sit" command to Pi 4 ({PI4_IP})...')
        self.sock_send.sendto(serialized_data, (PI4_IP, PI4_PORT))
        
        self.state = 'WAITING_FOR_PICKUP'
        self.get_logger().info('State: WAITING_FOR_PICKUP')

    def udp_listen_loop(self):
        """Runs in a background thread, listening for Pi 4 Protobuf messages."""
        while rclpy.ok():
            try:
                # Blocks the thread, but NOT the main ROS 2 executor
                data, addr = self.sock_recv.recvfrom(1024)
                
                # Parse the incoming Protobuf byte stream
                received_msg = states_pb2.States()
                received_msg.ParseFromString(data)

                if received_msg.action == 'can picked up' and self.state == 'WAITING_FOR_PICKUP':
                    self.get_logger().info('Received "can picked up" from Pi 4. Initiating stand...')
                    self.state = 'EXECUTING_STAND'
                    
                    # Run the physical stand sequence in a thread
                    threading.Thread(target=self.execute_stand_sequence, daemon=True).start()
                    
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'UDP Listener Error: {e}')

    def execute_stand_sequence(self):
        self.get_logger().info('Standing the Go1 up...')
        self._send_cmd_stream(mode=6, gait=0, duration=1.5) # Stand up
        self._send_cmd_stream(mode=1, gait=0, duration=1.0) # Force stand
        self._send_cmd_stream(mode=2, gait=1, duration=1.0) # Walk mode (Trot)

        # Create and serialize the final Protobuf message
        cmd_msg = states_pb2.States()
        cmd_msg.action = "stand"
        serialized_data = cmd_msg.SerializeToString()

        self.get_logger().info(f'Sending "stand" command to Pi 4 ({PI4_IP})...')
        self.sock_send.sendto(serialized_data, (PI4_IP, PI4_PORT))

        # Reset state machine to accept the next Nav2 goal
        self.state = 'NAVIGATING'
        self.get_logger().info('Resuming Navigation. State: NAVIGATING')

    # ------------------------------------------------------------------ #
    #  Stream Helper: Feeds the Unitree Watchdog Timeout                 #
    # ------------------------------------------------------------------ #
    def _send_cmd_stream(self, mode: int, gait: int, duration: float):
        start_time = time.time()
        while time.time() - start_time < duration:
            cmd = HighCmd()
            cmd.head[0] = 0xFE
            cmd.head[1] = 0xEF
            cmd.level_flag = 0xEE
            cmd.mode        = mode
            cmd.gait_type   = gait
            cmd.velocity    = [0.0, 0.0]
            cmd.yaw_speed   = 0.0
            cmd.body_height = 0.0
            cmd.euler       = [0.0, 0.0, 0.0]
            self._cmd_pub.publish(cmd)
            time.sleep(0.02) # ~50Hz publish rate


def main(args=None):
    rclpy.init(args=args)
    socket.setdefaulttimeout(1.0) # Allows the UDP thread to exit cleanly on Ctrl+C
    node = Pi5Go1Commander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()