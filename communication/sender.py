import socket
import message_pb2
import time

# Configuration
# PI5_IP = "10.216.25.11" # Replace with your Pi 4's actual IP
PI4_IP = "192.168.12.128"
PORT = 25000

# Create the UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_point(val_x, val_y, val_depth, val_angle, val_width, val_height):
    # 1. Create the message object
    msg = message_pb2.LitterPoint()

    # 2. Assign your changing values to the Protobuf fields
    msg.x = val_x
    msg.y = val_y
    msg.depth = val_depth
    msg.angle = val_angle
    msg.width = val_width
    msg.height = val_height

    # 3. Turn it into binary for the UDP socket
    binary_payload = msg.SerializeToString()
    
    # 4. Send it
    sock.sendto(binary_payload, (PI4_IP, PORT))

try:
    print("Sending to: ", PI4_IP)
    while True:
        send_point(0.0, 1.0, 2.0, 3.0, 4.0, 5.0)
        time.sleep(0.5) #TODO
except KeyboardInterrupt:
    print("Stopping sender...")