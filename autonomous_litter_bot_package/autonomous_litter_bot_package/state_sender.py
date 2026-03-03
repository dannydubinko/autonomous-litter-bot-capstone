import socket 
import states_pb2

PI5_IP = "127.0.0.1"
PORT = 25002

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

state = states_pb2.States()
state.action = "can picked up"

serialized_data = state.SerializeToString()
sock.sendto(serialized_data, (PI5_IP, PORT))

print(f"Goal Sent to : {PI5_IP} {PORT}")
