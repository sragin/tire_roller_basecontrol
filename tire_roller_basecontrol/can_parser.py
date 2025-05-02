# Copyright 2024 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.
#
# 타이어롤러용 CAN 메시지 수신 노드


from array import array

from ament_index_python import get_package_share_directory
from can_msgs.msg import Frame
import cantools
import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from roller_base_interfaces.msg import RemoteControl
from std_msgs.msg import Float32


class CanParser(Node):

    def __init__(self):
        super().__init__('can_parser')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        self.candb_remote = cantools.db.load_file(
            get_package_share_directory('tire_roller_basecontrol') + '/remote_240122.dbc')
        self.can_msg_remote_lever = self.candb_remote.get_message_by_name('Lever')
        self.can_msg_remote_switch = self.candb_remote.get_message_by_name('Switch')
        self.candb_encoder = cantools.db.load_file(
            get_package_share_directory('tire_roller_basecontrol') + '/steerEnc_220531.dbc')
        self.can_msg_encoder = self.candb_encoder.get_message_by_name('SteerEncoder')

        self.can2_msg_subscriber = self.create_subscription(
            Frame,
            'can2/from_can_bus',
            self.recv_can_bus2,
            qos_profile=10
        )
        self.can3_msg_subscriber = self.create_subscription(
            Frame,
            'can3/from_can_bus',
            self.recv_can_bus3,
            qos_profile=10
        )

        self.remote_mgs_publish_frequency = 20  # 입력주파수가 20Hz 이어서 출력 주파수도 20Hz로 설정함
        self.remote_publisher = self.create_publisher(
            RemoteControl, 'remote_msg', qos_profile_sensor_data)
        self.timer_publish_remote_msg = self.create_timer(
            1/self.remote_mgs_publish_frequency, self.publish_remote_msg)
        self.encoder_mgs_publish_frequency = 20
        self.encoder_publisher = self.create_publisher(
            Float32, 'encoder_msg', qos_profile_sensor_data)
        self.timer_publish_encoder_msg = self.create_timer(
            1/self.encoder_mgs_publish_frequency, self.publish_encoder_msg)

        self.remote_joystick = array('B', [0]*8)
        self.remote_switch = array('B', [0]*27)

        self.encoder_degree = 0.0

    def recv_can_bus2(self, msg: Frame):
        # self.get_logger().debug(f'{msg}')
        if msg.id == self.can_msg_remote_lever.frame_id:
            _cur = self.can_msg_remote_lever.decode(msg.data.tobytes())
            for n in range(0, 8):
                i = f'AN{n+1}'
                self.remote_joystick[n] = _cur[i]
        elif msg.id == self.can_msg_remote_switch.frame_id:
            _cur = self.can_msg_remote_switch.decode(msg.data.tobytes())
            self.remote_cnt = _cur['CNT']
            for n in range(27):
                if n == 15:
                    continue
                i = f'S{n:02}'
                self.remote_switch[n] = _cur[i]
        elif msg.id == self.can_msg_encoder.frame_id:
            _cur = self.can_msg_encoder.decode(msg.data.tobytes())
            self.encoder_cnt = _cur['SingleTurn']
            self.encoder_error = _cur['Error']
            # 16384 pulse / 360 degree / 140:25 gear ratio
            self.encoder_degree = self.encoder_cnt / 16384 * 360 / 140 * 25

        self.get_logger().debug(
            f'JOY: {self.remote_joystick} SW: {self.remote_switch}'
            f' ENC: {self.encoder_degree}',
            throttle_duration_sec=1.0
        )

    def publish_remote_msg(self):
        msg = RemoteControl()
        msg.remote_joystick = self.remote_joystick
        msg.remote_switch = self.remote_switch
        self.remote_publisher.publish(msg)

    def publish_encoder_msg(self):
        encoder_msg = Float32()
        encoder_msg.data = self.encoder_degree
        self.encoder_publisher.publish(encoder_msg)

    def recv_can_bus3(self, msg: Frame):
        pass

    # def publish_danfoss_msg(self):
    #     msg = DanfossFB()
    #     msg.boom_dn_c_p = self.boom_dn_cp
    #     msg.boom_up_c_p = self.boom_up_cp
    #     msg.bucket_in_c_p = self.bucket_in_cp
    #     msg.bucket_out_c_p = self.bucket_out_cp

    #     if -2000 <= self.boom_dn_fb <= 2000:
    #         msg.boom_dn_current = self.boom_dn_fb
    #     else:
    #         msg.boom_dn_current = 0
    #     if -2000 <= self.boom_up_fb <= 2000:
    #         msg.boom_up_current = self.boom_up_fb
    #     else:
    #         msg.boom_up_current = 0
    #     if -2000 <= self.bucket_in_fb <= 2000:
    #         msg.bucket_in_current = self.bucket_in_fb
    #     else:
    #         msg.bucket_in_current = 0
    #     if -2000 <= self.bucket_out_fb <= 2000:
    #         msg.bucket_out_current = self.bucket_out_fb
    #     else:
    #         msg.bucket_out_current = 0
    #     self.danfoss_publisher.publish(msg)

    #     self.boom_dn_cp = 0.0
    #     self.boom_up_cp = 0.0
    #     self.bucket_in_cp = 0.0
    #     self.bucket_out_cp = 0.0

    #     self.boom_dn_fb = 0
    #     self.boom_up_fb = 0
    #     self.bucket_in_fb = 0
    #     self.bucket_out_fb = 0


def main(args=None):
    rclpy.init(args=args)
    try:
        can_parser = CanParser()
        rclpy.spin(can_parser)
    except KeyboardInterrupt:
        can_parser.get_logger().warn('Keyboard interrrupt (SIGINT)')
    finally:
        can_parser.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
