import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros2_unitree_legged_msgs.msg import HighCmd

class TwistToHighCmd(Node):
    def __init__(self):
        super().__init__('twist_to_high_cmd_node')
        
        # Subscribes to standard Nav2/Teleop output
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.twist_callback,
            10)
            
        # Publishes to the Unitree Bridge input
        self.publisher_ = self.create_publisher(HighCmd, '/high_cmd', 10)
        
        # Internal state to keep the dog in "Sport Mode" while moving
        self.get_logger().info('Twist to HighCmd Translator Started')

    def twist_callback(self, msg):
        high_cmd = HighCmd()
        
        # --- The "Magic Numbers" for Go1 Safety ---
        high_cmd.head[0] = 0xFE
        high_cmd.head[1] = 0xEF
        high_cmd.level_flag = 0xEE
        
        # --- Mode Logic ---
        # If we receive any velocity, we must be in Sport Mode (2)
        # If velocity is zero, we stay in Mode 2 to keep balance active
        high_cmd.mode = 2 
        high_cmd.gait_type = 1 # Trot gait
        
        # --- Mapping Twist to HighCmd ---
        # Twist linear.x (m/s) -> HighCmd velocity[0]
        # Twist linear.y (m/s) -> HighCmd velocity[1]
        # Twist angular.z (rad/s) -> HighCmd yaw_speed
        
        high_cmd.velocity[0] = msg.linear.x
        high_cmd.velocity[1] = msg.linear.y
        high_cmd.yaw_speed = msg.angular.z
        
        # Optional: Add a safety limit (Go1 max speed is ~1.5m/s)
        if high_cmd.velocity[0] > 1.0: high_cmd.velocity[0] = 1.0
        
        self.publisher_.publish(high_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToHighCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()