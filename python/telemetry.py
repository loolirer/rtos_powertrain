#!/usr/bin/env python3
import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

UDP_IP = "0.0.0.0"
TELEMETRY_PORT = 4321 
COMMAND_PORT = 1234   
SAMPLES = 100 

current_l_set = 0.0
current_r_set = 0.0

l_set_hist = deque([0.0] * SAMPLES, maxlen=SAMPLES)
l_meas_hist = deque([0.0] * SAMPLES, maxlen=SAMPLES)
r_set_hist = deque([0.0] * SAMPLES, maxlen=SAMPLES)
r_meas_hist = deque([0.0] * SAMPLES, maxlen=SAMPLES)

sock_t = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_t.bind((UDP_IP, TELEMETRY_PORT))
sock_t.setblocking(False)

sock_c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_c.bind((UDP_IP, COMMAND_PORT))
sock_c.setblocking(False)

fig, (ax_l, ax_r) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
fig.suptitle('RTPT MONITOR', fontsize=16, fontweight='bold', y=0.96)
line_l_m, = ax_l.plot(np.arange(SAMPLES), [0]*SAMPLES, 'b-', lw=2, label='Actual')
line_l_s, = ax_l.plot(np.arange(SAMPLES), [0]*SAMPLES, 'c--', lw=1.5, label='Target')
line_r_m, = ax_r.plot(np.arange(SAMPLES), [0]*SAMPLES, 'r-', lw=2, label='Actual')
line_r_s, = ax_r.plot(np.arange(SAMPLES), [0]*SAMPLES, 'm--', lw=1.5, label='Target')

ax_l.set_title('Left Motor Performance', loc='left', fontsize=12, pad=10)
ax_r.set_title('Right Motor Performance', loc='left', fontsize=12, pad=10)

for ax in [ax_l, ax_r]:
    ax.set_ylim(-60, 60)
    ax.set_xlim(0, SAMPLES)
    ax.legend(loc='upper right')

def sync_data():
    global current_l_set, current_r_set
    
    while True:
        try:
            data, _ = sock_c.recvfrom(1024)
            if len(data) == 8:
                current_l_set, current_r_set = struct.unpack('ff', data)
        except BlockingIOError:
            break

    while True:
        try:
            data, _ = sock_t.recvfrom(1024)
            if len(data) == 8:
                lm, rm = struct.unpack('ff', data)
                
                l_meas_hist.append(lm)
                l_set_hist.append(current_l_set)
                
                r_meas_hist.append(-rm) # Right is mirrored
                r_set_hist.append(-current_r_set) # Right is mirrored
        except BlockingIOError:
            break

def update_plot(frame):
    sync_data()
    line_l_m.set_ydata(list(l_meas_hist))
    line_l_s.set_ydata(list(l_set_hist))
    line_r_m.set_ydata(list(r_meas_hist))
    line_r_s.set_ydata(list(r_set_hist))

    return line_l_m, line_l_s, line_r_m, line_r_s

ani = FuncAnimation(fig, update_plot, interval=20, blit=True)
plt.get_current_fig_manager().full_screen_toggle()
plt.show()