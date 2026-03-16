import threading
import socket
import struct
import time
import numpy as np
from inputs import get_gamepad, UnpluggedError

class XboxController:
    def __init__(self):
        # Status
        self.plugged = True

        # Buttons
        self.A = 0
        self.B = 0
        self.X = 0
        self.Y = 0
        self.LB = 0
        self.RB = 0
        self.L3 = 0
        self.R3 = 0

        # Triggers
        self.LT_R = 0
        self.RT_R = 0
        self.LT = 0.0
        self.RT = 0.0

        # Joysticks
        self.LJ_X = 0
        self.LJ_Y = 0
        self.RJ_X = 0
        self.RJ_Y = 0
        self.LJ = (0.0, 0.0)
        self.RJ = (0.0, 0.0)

        # D-pad
        self.D_PAD_X = 0
        self.D_PAD_Y = 0
        self.D_PAD = (0, 0)

        # Create and start the reading thread
        reading_thread = threading.Thread(target=self.read_gamepad, daemon=True)
        reading_thread.start()

    def read_gamepad(self):
        while True:
            try:
                # Get all current events
                events = {event.code: event.state for event in get_gamepad()}
                self.plugged = True

                # Update buttons
                self.A = events.get("BTN_SOUTH", self.A)
                self.B = events.get("BTN_EAST", self.B)
                self.X = events.get("BTN_NORTH", self.X)
                self.Y = events.get("BTN_WEST", self.Y)
                self.LB = events.get("BTN_TL", self.LB)
                self.RB = events.get("BTN_TR", self.RB)
                self.L3 = events.get("BTN_THUMBL", self.L3)
                self.R3 = events.get("BTN_THUMBR", self.R3)

                # Update triggers
                self.LT_R = events.get("ABS_Z", self.LT_R)
                self.RT_R = events.get("ABS_RZ", self.RT_R)
                self.LT = self.LT_R / 1023
                self.RT = self.RT_R / 1023

                # Update joysticks
                self.LJ_X = events.get("ABS_X", self.LJ_X)
                self.LJ_Y = events.get("ABS_Y", self.LJ_Y)
                self.RJ_X = events.get("ABS_RX", self.RJ_X)
                self.RJ_Y = events.get("ABS_RY", self.RJ_Y)

                self.LJ = (self.LJ_X / 32768.0, self.LJ_Y / 32768.0)
                self.RJ = (self.RJ_X / 32768.0, self.RJ_Y / 32768.0)

                # Update D-pad
                self.D_PAD_X = events.get("ABS_HAT0X", self.D_PAD_X)
                self.D_PAD_Y = events.get("ABS_HAT0Y", self.D_PAD_Y)
                self.D_PAD = (self.D_PAD_X, -self.D_PAD_Y)

            except UnpluggedError:
                self.plugged = False


PICO_IP = "192.168.1.103"
PORT = 1234

max_theta_dot = 50.0 # rad/s
r = 0.016 # m
s = 0.10 # m
max_v_B = max_theta_dot * r  
max_omega = (r / s) * (max_theta_dot * 2)
deadzone = 0.15

def main():
    print("Starting Xbox Controller...")
    XC = XboxController()
    
    time.sleep(1)
    if not XC.plugged:
        print("Waiting for controller to be plugged in...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"Sending binary setpoints to {PICO_IP}:{PORT}")
    print("Press 'B' button on the controller to exit.")

    try:
        while True:
            LJ = XC.LJ[0] if abs(XC.LJ[0]) > deadzone else 0.0
            omega = -LJ * max_omega
            v_B = (XC.RT - XC.LT) * max_v_B
            
            theta_dot_L, theta_dot_R = np.linalg.inv(
                [
                    [r/2.0, r/2.0], 
                    [-r/s,  r/s]
                ]) @ np.array([v_B, omega]
            )

            theta_dot_L = max(-max_theta_dot, min(max_theta_dot, theta_dot_L))
            theta_dot_R = max(-max_theta_dot, min(max_theta_dot, theta_dot_R))

            payload = struct.pack('<ff', theta_dot_L, -theta_dot_R) # Right is mirrored

            sock.sendto(payload, (PICO_IP, PORT))

            print(f"M0 (Left): {theta_dot_L:5.2f} rad/s | M1 (Right): {theta_dot_R:5.2f} rad/s", end='\r')

            if XC.B == 1:
                print("\nExiting...")

                sock.sendto(struct.pack('<ff', 0.0, 0.0), (PICO_IP, PORT))
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nForce quit. Sending emergency stop command...")
        sock.sendto(struct.pack('<ff', 0.0, 0.0), (PICO_IP, PORT))

    except:
        print("\nController died. Sending emergency stop command...")
        sock.sendto(struct.pack('<ff', 0.0, 0.0), (PICO_IP, PORT))

if __name__ == "__main__":
    main()