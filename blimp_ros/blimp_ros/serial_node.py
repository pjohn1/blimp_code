import serial
import struct
import threading
import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class SerialNode(Node):
    def __init__(self):

        super().__init__('serial_node')
        ###################################

        #Serial read/write constants ####################
        # Format: < - little endian 
        # B   : unsigned char (1 byte)
        # 8s  : 8-byte string
        # h   : short (2 bytes)
        # b   : signed char (1 byte)
        # f   : float (4 bytes)
        # c   : signed char (1 byte)
        # d   : double (8 bytes)
        self.receive_fmt = '<B8sBhbfffffffff' 
        self.receive_packet_size = struct.calcsize('<B8sBhbfffffffff')
        self.send_fmt = '<B8sddddddc'
        self.send_packet_size = struct.calcsize(self.send_fmt)

        ############################################


        self.declare_parameter('agents',['COM7','agent_61']) #This gets overwritten by launch file
        a = self.get_parameter('agents').value
        
        self.mapping_dict = {}

        # Maps each agent to the corresponding serial port, initializes the subscribers for reading voltage commands
        for i in range(len(a)-1):
            if i%2 == 0:
                sub = self.create_subscription(Float32MultiArray,f'/{a[i+1]}/serial/voltages',self.write_motor_commands,10)
                ser = serial.Serial(a[i],921600,timeout=0.1,bytesize=8, parity='N', stopbits=1,
                    write_timeout=1, xonxoff=False, rtscts=False, dsrdtr=False)
                self.mapping_dict[a[i+1]] = ser

    
    def write_motor_commands(self, msg):
        '''
        Take motor commands and write them to serial (ground station)

        msg : Float32MultiArray of Length 6 mapping to motor controls
        '''
        msg_ = 'MOTORCTL'
        data = msg.data
        ns = f'agent_{int(data[-1])}' # Blimp namespace
        uid = int(data[-1]) # Blimp ID
        mtrs =[0.0 if abs(d)<1e-6 else d/abs(d)*min(0.9,abs(round(d,2))) for d in data[:-1]] #Motor voltages, clipped
        pwm_1, pwm_2, pwm_3, pwm_4, pwm_5, pwm_6 = mtrs
        signal = 'n'
        to_send = f'{uid},{msg_},{pwm_1},{pwm_2},{pwm_3},{pwm_4},{pwm_5},{pwm_6},{signal},\n' #Newline sends command
        to_send = to_send.encode('utf-8')

        ser = self.mapping_dict[ns]
        ser.write(to_send)
        ser.flush() #flush buffer, commands send instantly and we do not want buildup


    # Telemetry read function for when sensors are used
    # def read_telemetry(self):
    #     '''

    #     Read telemetry from blimp sent to ground station

    #     '''
    #     while rclpy.ok():
    #         # t0 = self.get_clock().now().nanoseconds/1e9
    #         b = self.ser.read(1) #Look for header instead of reading full packet
    #         if b==b'\xAA': #Check header, telemtry msg starts with this
    #             packet = self.ser.read(self.receive_packet_size)
    #             # unpack telemetry structure
    #             struct_ = struct.unpack(self.receive_fmt,packet)
    #             uid,msg,isWorking,altitude,altitudeAccuracy,roll,\
    #             pitch,yaw,ax,ay,az,gx,gy,gz = struct_

    #             # publish telemetry
    #             msg = Float32MultiArray()
    #             msg.data = list(np.array([altitude,altitudeAccuracy,pitch,roll,yaw,gx,gy,gz]).astype(float))
    #             self.publisher.publish(msg)
    #             # t = self.get_clock().now().nanoseconds/1e9
    #             # self.get_logger().info(f"Serial read operating at {round(1/(t - t0),2)}Hz")
                # t0 = t
    
    def shutdown(self):
        for ns in self.mapping_dict:
            ser = self.mapping_dict[ns]
            for i in range(5): #Send multiple times in case the first one gets lost
                to_send = f'0,MOTORCTRL,0.0,0.0,0.0,0.0,0.0,0.0,n,\n'
                to_send = to_send.encode('utf-8')
                ser.write(to_send)
                time.sleep(0.05)
            ser.close()
            

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        pass
    serial_node.shutdown()
    serial_node.destroy_node()
        
if __name__ == '__main__':
    main()