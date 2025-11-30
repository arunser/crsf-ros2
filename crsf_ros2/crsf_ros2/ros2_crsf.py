#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
from .submodules.crsf import *
from rc_msgs.msg import RCMessage
from rc_msgs.srv import CommandBool
from crsf_msgs.msg import BatterySensor
from crsf_msgs.msg import Attitude
from crsf_msgs.msg import FlightMode



SERIAL_PORT = "/dev/ttyUSB0"
#BAUDRATE = 400000
BAUDRATE = 5200000


DEFAULT_ROLL_VALUE = 1500
DEFAULT_PITCH_VALUE = 1500
DEFAULT_YAW_VALUE = 1500
DEFAULT_THROTTLE_VALUE = 988


class RosCsrf(Node):

    def __init__(self):
        super().__init__('ros2_csrf')

        self.CMDS = {
            "roll": DEFAULT_ROLL_VALUE,
            "pitch": DEFAULT_PITCH_VALUE,
            "throttle": DEFAULT_THROTTLE_VALUE,
            "yaw": DEFAULT_YAW_VALUE,
            "aux1": 988,
            "aux2": 988,
            "aux3": 988,
            "aux4": 988,
        }

        timer_period = 0.020  # seconds

        self.get_logger().info("Ros2_CSRF node started")

        self.rc_sub = self.create_subscription(RCMessage, "/drone/rc_command", self.rc_command_topic_callback, 10)
        self.arming_srv = self.create_service(CommandBool, "/drone/cmd/arming", self.arming_service_callback)

        self.batt_pub = self.create_publisher(BatterySensor, "/drone/battery_info", 1)
        self.mode_pub = self.create_publisher(FlightMode, "/drone/flight_mode", 1)
        self.attd_pub = self.create_publisher(Attitude, "/drone/attitude", 1)

        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=2)
        self.input = bytearray()
        self.unique = []

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def crsf_msgs_publisher(self, crsf_msg):
        if crsf_msg == None:
            pass
        else:
            msg_type = crsf_msg[0]
            msg = crsf_msg[1:]
            
            if msg_type == 1:
                attitude_msg = Attitude()
                attitude_msg.pitch = msg[0]
                attitude_msg.roll = msg[1]
                attitude_msg.yaw = msg[2]
                self.attd_pub.publish(attitude_msg)

            elif msg_type == 2:
                mode_msg = FlightMode()
                mode_msg.flight_mode = msg[0]
                self.mode_pub.publish(mode_msg)

            elif msg_type == 3:
                batt_msg = BatterySensor()
                batt_msg.vbat = msg[0]
                batt_msg.curr = msg[1]
                batt_msg.mah = msg[2]
                batt_msg.pct = msg[3]
                self.batt_pub.publish(batt_msg)

            else:
                pass

    def rc_command_topic_callback(self, msg):
        time.sleep(0.020)
        self.CMDS["roll"] = msg.rc_roll
        self.CMDS["pitch"] = msg.rc_pitch
        self.CMDS["throttle"] = msg.rc_throttle
        self.CMDS["yaw"] = msg.rc_yaw
        self.get_logger().info(str(self.CMDS))

    def arming_service_callback(self, request, response):

        if request.value:
            self.arm()
            response.data = "UAV is ARMED"
        else:
            self.disarm()
            response.data = "UAV is DISARMED"

        return response
    
    def arm(self):
        self.CMDS 
        self.get_logger().info("Arming UAV")
        self.CMDS["roll"] = DEFAULT_ROLL_VALUE
        self.CMDS["pitch"] = DEFAULT_PITCH_VALUE
        self.CMDS["throttle"] = DEFAULT_THROTTLE_VALUE
        self.CMDS["yaw"] = DEFAULT_YAW_VALUE
        #for arming, use aux1
        self.CMDS["aux1"] = 2000
        #for angle mode, use aux2
        self.CMDS["aux2"] = 1500
        #for headless mode, use aux3
        self.CMDS["aux3"] = 1500
        self.get_logger().info(str(self.CMDS))

    def disarm(self):
        self.get_logger().info("Disarming UAV")
        self.CMDS["aux1"] = 988
        self.CMDS["aux2"] = 988
        self.CMDS["aux3"] = 988
        self.get_logger().info(str(self.CMDS))
        
    def pwm_to_csrf(self):
        self.PWM = []
        for value in self.CMDS.values():
            pwm = int(((820/512)*(value-1500)) + 992)
            self.PWM.append(pwm)
        for i in range(8):
            self.PWM.append(172)
        return self.PWM
    
    def timer_callback(self):
        TX = True
        fm = 0
        CH = self.pwm_to_csrf()
        if self.ser.in_waiting > 0:
            self.input.extend(self.ser.read(self.ser.in_waiting))
        else:
            if TX:
                self.ser.write(channelsCrsfToChannelsPacket(CH))
                
        if len(self.input) > 2:
            # This simple parser works with malformed CRSF streams
            # it does not check the first byte for SYNC_BYTE, but
            # instead just looks for anything where the packet length
            # is 4-64 bytes, and the CRC validates
            expected_len = self.input[1] + 2
            if expected_len > 64 or expected_len < 4:
                self.input = []
            elif len(self.input) >= expected_len:
                single = self.input[:expected_len] # copy out this whole packet
                self.input = self.input[expected_len:] # and remove it from the buffer

                if not crsf_validate_frame(single): # single[-1] != crc:
                    packet = ' '.join(map(hex, single))
                    print(f"crc error: {packet}")
                #All packets are in the CRSF format [sync] [len] [type] [payload] [crc8] with a maximum total size of 64 bytes.
                else:
                    
                    msg = handleCrsfPacket(single[2], single)
                    #for publishing telemetry onto ROS2 topics
                    self.crsf_msgs_publisher(msg)
                
                    if single[2] not in self.unique:
                        self.unique.append(single[2])
                    print(self.unique)

def main(args=None):
    rclpy.init()
    ros2_csrf = RosCsrf()  
    rclpy.spin(ros2_csrf)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            

            
