import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json
import os
import sys

# Standard ROS2 way to import from the same package directory
try:
    from . import detection_pb2
except (ImportError, ValueError):
    # Fallback for when running as a standalone script
    import detection_pb2

class ProtoSender(Node):
    def __init__(self):
        super().__init__('proto_sender')
        
        # Parameters
        self.declare_parameter('target_ip', '127.0.0.1')
        self.declare_parameter('target_port', 25000)
        
        self.target_ip = self.get_parameter('target_ip').value
        self.target_port = self.get_parameter('target_port').value
        
        # Socket setup (UDP)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'detected_objects',
            self.listener_callback,
            10)
        
        self.get_logger().info(f"Proto Sender initialized. Sending to {self.target_ip}:{self.target_port}")

    def listener_callback(self, msg):
        try:
            # 1. Parse JSON
            data = json.loads(msg.data)
            
            # 2. Build Protobuf message
            frame = detection_pb2.DetectionFrame()
            for obj in data:
                proto_obj = frame.objects.add()
                proto_obj.x_pos = float(obj.get('x_pos', 0))
                proto_obj.y_pos = float(obj.get('y_pos', 0))
                proto_obj.dist_meters = float(obj.get('dist_meters', 0))
                proto_obj.width_cm = float(obj.get('width_cm', 0))
                proto_obj.length_cm = float(obj.get('length_cm', 0))
                proto_obj.angle = float(obj.get('angle', 0))
            
            # 3. Serialize and Send
            binary_data = frame.SerializeToString()
            self.sock.sendto(binary_data, (self.target_ip, self.target_port))
            
        except Exception as e:
            self.get_logger().error(f"Failed to send proto: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ProtoSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()