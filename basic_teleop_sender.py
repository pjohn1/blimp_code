#############################################################################
# THIS SCRIPT IS FOR OPTITRACK FORWARDING TO WSL                            #
# INTENDED FOR A WINDOWS SYSTEM USING ROS2 ON WSL                           #
# THIS SCRIPT MUST BE LOCATED ON THE WINDOWS SYSTEM (NOT IN WSL FILES)      #
# AND RUN FROM POWERSHELL                                                   #
#############################################################################

import threading, time, sys
import socket
import signal
import sys
from collections import deque
import pygame

WSL_IP = "172.27.169.213" #In WSL's ip a command, this is likely eth0
# WSL_IP = "0.0.0.0"
TARGET_PORT = 1515 #Port to receive on WSL system, in optitrack node
DEADZONE = 0.1

sending = True
q = deque([]) #Use deque for O(1) queuing
MAX_QUEUE = 5

t0 = time.time()

pygame.init()
pygame.joystick.init()

def clip_cmd(v): #checks deadzone and bounds
    if abs(v) > DEADZONE:
        return v if abs(v) < 1 else v/abs(v)*1
    return 0

if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
else:
    joystick = None

def start_receiving():
    while True:
        pygame.event.pump()
        if joystick:
            axes = [clip_cmd(joystick.get_axis(i)) for i in range(joystick.get_numaxes())]
            buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            if len(q) < MAX_QUEUE:
                q.append((axes,buttons))
        time.sleep(0.01)

'''
AXES:
0: Left Joystick Left/Right (-1 left)
1: Left Joystick Up/Down (-1 up)
2: Right Joystick Left/Right (-1 left)
3: Right Joystick Up/Down (-1 up)

BUTTONS:
0: A
1: B
2: X
3: Y
4: LB
5: RB
'''


def send():
    # Background thread empties queue and sends to WSL receiver
    global sending, t0
    try:
        while sending:
            if len(q) > 0:
                axes,buttons = q.popleft()
                to_send = f"{axes[0]},{axes[1]},{axes[2]},{axes[3]},{buttons[0]},{buttons[1]},{buttons[2]},{buttons[3]},{buttons[4]},{buttons[5]}\n"
                to_send = to_send.encode("utf-8") #Encode to bytestring
                sock.sendto(to_send, (WSL_IP, TARGET_PORT))

    except KeyboardInterrupt:
        return
        

def _signal_handler(sig,frame):
    global sending
    try:
        sending=False
        send_thread.join()
        sock.close()
    except:
        pass
    sys.exit(0)

signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

send_thread = threading.Thread(target=send,daemon=True)
send_thread.start()

start_receiving()