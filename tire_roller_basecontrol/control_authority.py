# Copyright 2024 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.
#
# 타이어롤러 제어권한을 가져오기위한 노드
import threading

import can
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from roller_base_interfaces.msg import DriveControl


class ControlAuthority(Node):

    def __init__(self):
        super().__init__('can_parser')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')

        self.cnt = 0

        self.drive_msg = DriveControl()
        self.remote_msg_subscriber = self.create_subscription(
            DriveControl, 'drive_control', self.recv_drive, qos_profile_system_default)

        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=250000)
        self.thread_canrecv = threading.Thread(target=self.work_canrecv)
        self.thread_canrecv.daemon = True
        self.thread_canrecv.start()

    def recv_drive(self, msg: DriveControl):
        self.drive_msg = msg

    def work_canrecv(self):
        while True:
            try:
                msg = self.bus.recv(timeout=1)
                if msg is None:
                    print('Timeout')
                    pass
                print(f'MSG: {msg}')
                if msg.arbitration_id == 0x60B:
                    print(f'{msg}')
                    msg_58 = can.Message(
                        arbitration_id=0x58B,
                        is_extended_id=False,
                        data=[0x60, 0x17, 0x10, 0, 0, 0, 0, 0])
                    print(msg_58)
                    self.bus.send(msg_58)
                if msg.arbitration_id == 0x000:
                    print(f'{msg}')
                    msg_70 = can.Message(
                        arbitration_id=0x70B,
                        is_extended_id=False,
                        data=[0x05])
                    print(msg_70)
                    self.bus.send(msg_70)
                if msg.arbitration_id == 0x141:
                    print(f'{msg}')
                if msg.arbitration_id == 0x142:
                    print(f'{msg}')
                    self.send_lever_msg()
            except AttributeError:
                pass
            except Exception as e:
                print(f'recv error {e}')

    def send_lever_msg(self):
        try:
            # 레버 전후진
            if self.drive_msg.fnr == 1: # forward
                joy_pos = 5
            elif self.drive_msg.fnr == 2: # reverse
                joy_pos = 6
            else:
                joy_pos = 4

            accel = self.drive_msg.accel

            msg_101 = can.Message(
                arbitration_id=0x101,
                is_extended_id=False,
                data=[0x00, joy_pos, accel, 0xFF, 0x00, 0x00, 0x00, self.cnt])
            self.cnt = self.cnt + 1
            self.cnt = self.cnt % 256
            msg_102 = can.Message(
                arbitration_id=0x102,
                is_extended_id=False,
                data=[0, 0, 0, 0, 0, 0, 0, 0])
            msg_102.data[0] = 255 - msg_101.data[0]
            msg_102.data[1] = 255 - msg_101.data[1]
            msg_102.data[2] = 255 - msg_101.data[2]
            msg_102.data[3] = 255 - msg_101.data[3]
            msg_102.data[4] = 255 - msg_101.data[4]
            msg_102.data[5] = 255 - msg_101.data[5]
            msg_102.data[6] = 255 - msg_101.data[6]
            msg_102.data[7] = 255 - msg_101.data[7]
            msg_103 = can.Message(
                arbitration_id=0x103,
                is_extended_id=False,
                data=[0, 0, 0, 0, 0, 0, 0, msg_101.data[-1]])
            msg_104 = can.Message(
                arbitration_id=0x104,
                is_extended_id=False,
                data=[0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, msg_102.data[-1]])
            msg_105 = can.Message(
                arbitration_id=0x105,
                is_extended_id=False,
                data=[0, 0x5C, 0x5F, 0, 0, 0, 0, msg_101.data[-1]])
            msg_106 = can.Message(
                arbitration_id=0x106,
                is_extended_id=False,
                data=[0xFF, 0xA3, 0xA0, 0xFF, 0xFF, 0xFF, 0xFF, msg_102.data[-1]])
            msg_70 = can.Message(arbitration_id=0x70B, is_extended_id=False, data=[0x05])

            self.bus.send(msg_70)
            self.bus.send(msg_101)
            self.bus.send(msg_102)
            self.bus.send(msg_103)
            self.bus.send(msg_104)
            self.bus.send(msg_105)
            self.bus.send(msg_106)

            print(f'{msg_70}')
            print(f'{msg_101}')
            print(f'{msg_102}')
            print(f'{msg_103}')
            print(f'{msg_104}')
            print(f'{msg_105}')
            print(f'{msg_106}')
        except Exception as e:
            print(f'{e}')


def main(args=None):
    try:
        rclpy.init(args=args)
        control = ControlAuthority()
        rclpy.spin(control)
    except KeyboardInterrupt as e:
        print(e)
    finally:
        control.bus.shutdown()
        if rclpy.ok():
            control.destroy_node()
