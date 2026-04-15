#############################################################################
# THIS SCRIPT IS FOR OPTITRACK FORWARDING TO WSL                            #
# INTENDED FOR A WINDOWS SYSTEM USING ROS2 ON WSL                           #
# THIS SCRIPT MUST BE LOCATED ON THE WINDOWS SYSTEM (NOT IN WSL FILES)      #
# AND RUN FROM POWERSHELL                                                   #
#############################################################################

import threading, time, sys
from NatNetClient import NatNetClient #OptiTrack's software that receives the multicast data
import socket
import signal
import sys
from collections import deque

SERVER_IP = "192.168.0.104"   # Motive PC IP (Can be found in OptiTrack software under data streaming panel)
LOCAL_IP  = "192.168.0.101"   # This machine's IP (Can be found using "ip a" in terminal or on TPLink domain 192.168.0.1)
WSL_IP = "172.27.169.213" #Found in OptiTrack data streaming panel
# WSL_IP = "0.0.0.0"
TARGET_PORT = 1511 #Port to receive on WSL system, in optitrack node

# ids = {0,1}
goals = {62,63}
counts = {i:0 for i in goals} #keep track of goal ids

sending = True
q = deque([]) #Use deque for O(1) queuing
MAX_QUEUE = 5

t0 = time.time()

def send():
    # Background thread empties queue and sends to WSL receiver
    global sending, t0
    try:
        while sending:
            if len(q) > 0:
                id_,position,rotation= q.popleft()
                to_send = f"{id_},{position[0]},{position[1]},{position[2]},{rotation[0]},{rotation[1]},{rotation[2]},{rotation[3]}\n"
                to_send = to_send.encode("utf-8") #Encode to bytestring
                sock.sendto(to_send, (WSL_IP, TARGET_PORT))

    except KeyboardInterrupt:
        return


def receiveRigidBodyFrame(id_, position, rotation):
    global count,t0
    if len(q) < MAX_QUEUE:
        if (id_ in goals and counts[id_]%100 == 0): #if a goal, only send once every 100 updates (about 2Hz)
            q.append((id_,position,rotation)) #Manage a queue to avoid data loss
        elif id_ not in goals:
            q.append((id_,position,rotation))
        elif id_ in goals:
            counts[id_]+=1
        

def natnet_runner():
    try:
        print("Starting blocking run()")
        client.run()
    except Exception as e:
        print("client.run() exception:", repr(e))

def _signal_handler(sig,frame):
    global sending
    try:
        client.shutdown()
        sending=False
        send_thread.join()
        sock.close()
    except:
        pass
    sys.exit(0)

signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)

client = NatNetClient(SERVER_IP,LOCAL_IP)
client.rigidBodyListener = receiveRigidBodyFrame

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

send_thread = threading.Thread(target=send,daemon=True)
send_thread.start()

try:
    natnet_runner()
except:
    pass
while True:
    pass