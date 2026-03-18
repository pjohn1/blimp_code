import serial
import struct
import threading
import struct
import time
import re

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray
import numpy as np
from rcl_interfaces.msg import SetParametersResult

AGENT_PATTERN = re.compile(r"^agent_\d+$")

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


        self.declare_parameter('agents', Parameter.Type.STRING_ARRAY)
        a = self.get_parameter('agents').value

        self.config_lock = threading.Lock()
        self.voltage_subscribers = []
        self.mapping_dict = {}
        self._motor_log_last = {}  # ns -> last log time
        self._motor_log_interval = 0.5  # seconds between logs per agent
        self._configure_agents(a)
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _validate_agents(self, agents):
        if len(agents) % 2 != 0:
            return False, "Parameter 'agents' must contain port/agent pairs."
        ports = set()
        names = set()
        for i in range(0, len(agents), 2):
            port = str(agents[i]).strip()
            name = str(agents[i + 1]).strip()
            if not port:
                return False, "Serial port cannot be empty."
            if not AGENT_PATTERN.match(name):
                return False, f"Invalid agent name '{name}'. Expected format agent_<id>."
            if port in ports:
                return False, f"Duplicate serial port '{port}'."
            if name in names:
                return False, f"Duplicate agent name '{name}'."
            ports.add(port)
            names.add(name)
        return True, ""

    def _close_all_serial(self):
        for ser in self.mapping_dict.values():
            if ser is not None and ser.is_open:
                ser.close()
        self.mapping_dict = {}

    def _destroy_all_subscriptions(self):
        for sub in self.voltage_subscribers:
            self.destroy_subscription(sub)
        self.voltage_subscribers = []

    def _configure_agents(self, agents):
        valid, reason = self._validate_agents(agents)
        if not valid:
            raise ValueError(reason)

        with self.config_lock:
            self._destroy_all_subscriptions()
            self._close_all_serial()

            opened = 0
            for i in range(0, len(agents), 2):
                port = str(agents[i]).strip()
                agent = str(agents[i + 1]).strip()
                sub = self.create_subscription(
                    Float32MultiArray,
                    f'/{agent}/serial/voltages',
                    self.write_motor_commands,
                    10,
                )
                try:
                    ser = serial.Serial(
                        port,
                        921600,
                        timeout=0.1,
                        bytesize=8,
                        parity='N',
                        stopbits=1,
                        write_timeout=1,
                        xonxoff=False,
                        rtscts=False,
                        dsrdtr=False,
                    )
                except serial.SerialException as exc:
                    self.get_logger().warn(
                        f"Skipping '{agent}' on '{port}': could not open/configure serial port ({exc})"
                    )
                    self.destroy_subscription(sub)
                    continue
                self.voltage_subscribers.append(sub)
                self.mapping_dict[agent] = ser
                opened += 1

            if opened == 0:
                self.get_logger().warn(
                    "No usable serial ports were configured. "
                    "Node will stay alive and can be updated from GUI/parameters."
                )

    def _on_set_parameters(self, params):
        for param in params:
            if param.name == 'agents':
                if param.type_ != param.Type.STRING_ARRAY:
                    return SetParametersResult(
                        successful=False,
                        reason="Parameter 'agents' must be a string array.",
                    )
                valid, reason = self._validate_agents(param.value)
                if not valid:
                    return SetParametersResult(successful=False, reason=reason)
                try:
                    self._configure_agents(param.value)
                except Exception as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
                self.get_logger().info(f"Updated serial agent mapping: {list(param.value)}")
        return SetParametersResult(successful=True)

    
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

        with self.config_lock:
            ser = self.mapping_dict.get(ns)
        if ser is None:
            self.get_logger().warn(f"Received command for unknown namespace '{ns}'")
            return
        try:
            ser.write(to_send)
            ser.flush()  # flush buffer, commands send instantly and we do not want buildup
            now_t = time.monotonic()
            last = self._motor_log_last.get(ns, 0.0)
            if now_t - last >= self._motor_log_interval:
                self.get_logger().info(
                    f"Motor command {ns}: mtrs=[{pwm_1:.2f}, {pwm_2:.2f}, {pwm_3:.2f}, "
                    f"{pwm_4:.2f}, {pwm_5:.2f}, {pwm_6:.2f}]"
                )
                self._motor_log_last[ns] = now_t
        except serial.SerialException as exc:
            self.get_logger().warn(
                f"Serial write failed for '{ns}': {exc}. "
                "Check cable/power and refresh mappings from GUI."
            )


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
        for ns in list(self.mapping_dict.keys()):
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