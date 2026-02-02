import socket
import message_pb2 # Ensure this file is in the same folder

# Configuration
LISTEN_IP = "0.0.0.0" # Listens on all available network interfaces
PORT = 25000

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, PORT))

print(f"Receiver started! Listening on port {PORT}...")

try:
    while True:
        # 1. Receive binary data
        data, addr = sock.recvfrom(1024) 
        
        # 2. Create an empty message object
        msg = message_pb2.LitterPoint()
        
        # 3. Parse the binary data back into the object
        msg.ParseFromString(data)
        
        # 4. Print the results
        print(f"--- Received from {addr} ---")
        print(f"X: {msg.x}, Y: {msg.y}, Depth: {msg.depth}, Angle: {msg.angle}")
        
except KeyboardInterrupt:
    print("\nStopping receiver...")
finally:
    sock.close()