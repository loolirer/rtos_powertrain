import socket
import struct

UDP_IP = "0.0.0.0" 
UDP_PORT = 4321

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for Pico Telemetry on {UDP_IP}:{UDP_PORT}...")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        
        if len(data) == 8:
            left_speed, right_speed = struct.unpack('ff', data)

            print(f"M0 (Left): {left_speed:5.2f} rad/s | M1 (Right): {-right_speed:5.2f} rad/s", end='\r') # Right is mirrored

        else:
            print(f"Received unexpected packet size: {len(data)} bytes from {addr[0]}")

except KeyboardInterrupt:
    print("\nListener stopped.")
    sock.close()