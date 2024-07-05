#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
from .submodules.crsf import *
from rc_msgs.msg import RCMessage
from rc_msgs.srv import CommandBool


SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 400000


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

        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=2)
        self.input = bytearray()
        self.unique = []

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def rc_command_topic_callback(self, msg):
        time.sleep(0.020)
        self.CMDS["pitch"] = msg.rc_pitch
        self.CMDS["throttle"] = msg.rc_throttle
        self.CMDS["yaw"] = msg.rc_yaw
        self.CMDS["aux3"] = msg.aux3
        self.CMDS["aux4"] = msg.aux4

        self.get_logger().info(str(self.CMDS))

    def arming_service_callback(self, request, response):

        if request.value:
            self.arm()
            response.data = "Drone is ARMED"
        else:
            self.disarm()
            response.data = "Drone is DISARMED"

        return response
    
    def arm(self):
        self.get_logger().info("Arming drone")
        self.CMDS["roll"] = DEFAULT_ROLL_VALUE
        self.CMDS["pitch"] = DEFAULT_PITCH_VALUE
        self.CMDS["throttle"] = DEFAULT_THROTTLE_VALUE
        self.CMDS["yaw"] = DEFAULT_YAW_VALUE
        self.CMDS["aux2"] = 2000
        self.CMDS["aux4"] = 2000
        self.get_logger().info(str(self.CMDS))

    def disarm(self):
        self.get_logger().info("Disarming drone")
        self.CMDS["aux2"] = 988
        self.CMDS["aux4"] = 1200
        
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
                else:
                    handleCrsfPacket(single[2], single)
                    #print(single[2])
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
            

            