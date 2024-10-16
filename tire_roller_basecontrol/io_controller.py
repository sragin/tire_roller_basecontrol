# Copyright 2024 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


# import time

from pyModbusTCP.client import ModbusClient
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from roller_base_interfaces.msg import AnmControl
from roller_base_interfaces.msg import DriveControl
from std_msgs.msg import Bool
from std_msgs.msg import String


class IOController(Node):

    def __init__(self):
        super().__init__('io_controller')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')

        self.modbusclient_DO = ModbusClient(host='192.168.181.10', timeout=1)

        self.status = 'e_stop'
        self.msg_drive = DriveControl()
        self.msg_anm = AnmControl()

        self.count = 0
        self.can2_cnt = 0
        self.log_display_cnt = 100

        self.initialized = False
        self.anm_msg_subscriber = self.create_subscription(
            AnmControl, 'anm_msg', self.recv_anm, qos_profile_system_default)
        self.drive_msg_subscriber = self.create_subscription(
            DriveControl, 'drive_control', self.recv_drive, qos_profile_system_default)
        self.status_msg_subscriber = self.create_subscription(
            String, 'status', self.recv_status, qos_profile_system_default)
        self.estop_wireless_publisher = self.create_publisher(
            Bool, 'estop_wireless', qos_profile_system_default)

        self.io_control_frequency = 20
        self.timer_control_io = self.create_timer(
            1/self.io_control_frequency, self.control_io)

    def init_module(self):
        if not self.modbusclient_DO.is_open:
            self.get_logger().warn('DO module is not opened')
            self.modbusclient_DO.close()
            self.modbusclient_DO.open()
            self.get_logger().warn('DO module is opened')
        # 연결된 후 IO초기화 진행
        # Seat, CAN mux, BTS, Parking, Brake Analog
        self.modbusclient_DO.write_multiple_coils(0, [True, True, True, False, False])
        self.initialized = True

    def recv_status(self, msg: String):
        self.get_logger().debug(f'Status: {self.status}')
        self.status = msg.data

    def recv_drive(self, msg: DriveControl):
        self.get_logger().debug(f'{msg}')
        self.msg_drive = msg

    def recv_anm(self, msg: AnmControl):
        self.get_logger().debug(f'{msg}')
        self.msg_anm = msg

    def control_io(self):
        # Watchdog
        # self.modbusclient_DO.read_coils(15)
        # estop_wireless = Bool()
        # tmp = self.modbusclient_AIO.read_discrete_inputs(0)
        # self.get_logger().info(f'Wireless E-Stop released: {tmp[0]} {(tmp[0] is False)}')
        # estop_wireless.data = (tmp[0] is False)
        # self.estop_wireless_publisher.publish(estop_wireless)

        # if self.status == 'e_stop':
        #     self.get_logger().warn('E-STOP Pressed')
        #     if self.initialized:
        #         self.modbusclient_DO.write_multiple_coils(
        #             0,
        #             [
        #                 False, False, True, False, False,
        #                 False, True, False, False, False,
        #                 False, True, False
        #             ]
        #         )
        #         time.sleep(0.1)
        #         self.modbusclient_DO.write_single_coil(2, False)
        #         self.initialized = False
        #     return
        if self.status == 'manual':
            self.get_logger().warn(
                'No IO control in manual mode',
                throttle_duration_sec=0.99
            )
            if self.initialized:
                # Seat, CAN mux, BTS, Parking, Brake Analog
                self.modbusclient_DO.write_multiple_coils(0, [False, False, False, False, False])
                self.initialized = False
            return
        elif (self.status == 'remote' or self.status == 'auto') and not self.initialized:
            self.get_logger().warn(
                'Auto Mode',
                throttle_duration_sec=0.99
            )
            self.init_module()
            return


def main(args=None):
    rclpy.init(args=args)
    try:
        io_controller = IOController()
        rclpy.spin(io_controller)
    except KeyboardInterrupt:
        io_controller.get_logger().warn('Keyboard interrrupt (SIGINT)')
    finally:
        io_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
