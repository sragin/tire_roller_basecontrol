# Copyright 2024 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.
#
# 휠로더용 버킷 제어 노드
# 상위제어기나 리모콘에서 입력된 메시지를 변환하여 댄포스로 전송한다


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from std_msgs.msg import String
from wheelloader_interfaces.msg import AnmControl
from wheelloader_interfaces.msg import BucketControl
from wheelloader_interfaces.msg import RemoteControl


class BucketController(Node):

    def __init__(self):
        super().__init__('bucket_controller')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')

        self.anm_msg = AnmControl()
        self.remote_msg = RemoteControl()
        self.status = 'idle'
        self.bucket_log_cnt = 0

        self.anm_msg_subscriber = self.create_subscription(
            AnmControl, 'anm_msg', self.recv_anm, qos_profile_system_default)
        self.remote_msg_subscriber = self.create_subscription(
            RemoteControl, 'remote_msg', self.recv_remote, qos_profile_sensor_data)
        self.status_msg_subscriber = self.create_subscription(
            String, 'status', self.recv_status, qos_profile_system_default)

        self.bucket_control_frequency = 50
        self.bucket_control_msg_publisher = self.create_publisher(
            BucketControl, 'bucket_control', qos_profile_system_default)
        self.timer_control_bucket = self.create_timer(
            1/self.bucket_control_frequency, self.control_bucket)

    def recv_status(self, msg: String):
        self.status = msg.data

    def recv_anm(self, msg: AnmControl):
        self.anm_msg = msg

    def recv_remote(self, msg: RemoteControl):
        self.remote_msg = msg

    def control_bucket(self):
        if self.status == 'auto':
            boom_up_duty = self.anm_msg.boom_up_duty
            boom_dn_duty = self.anm_msg.boom_dn_duty
            bucket_load_duty = self.anm_msg.bucket_in_duty
            bucket_dump_duty = self.anm_msg.bucket_out_duty
        elif self.status == 'remote':
            msg = self.remote_msg
            # boom
            if msg.remote_joystick[0] < 127 and msg.remote_switch[1] == 1:
                boom_dn_duty = (127 - msg.remote_joystick[0]) / 126 * 100
                boom_up_duty = 0
            elif msg.remote_joystick[0] > 127 and msg.remote_switch[0] == 1:
                boom_dn_duty = 0
                boom_up_duty = (msg.remote_joystick[0] - 127) / 126 * 100
            else:
                boom_up_duty = 0
                boom_dn_duty = 0

            # bucket
            if msg.remote_joystick[1] < 127 and msg.remote_switch[3] == 1:
                bucket_load_duty = 0
                bucket_dump_duty = (127 - msg.remote_joystick[1]) / 126 * 100
            elif msg.remote_joystick[1] > 127 and msg.remote_switch[2] == 1:
                bucket_load_duty = (msg.remote_joystick[1] - 127) / 126 * 100
                bucket_dump_duty = 0
            else:
                bucket_dump_duty = 0
                bucket_load_duty = 0
        else:
            boom_up_duty = 0
            boom_dn_duty = 0
            bucket_load_duty = 0
            bucket_dump_duty = 0

        msg_bucket = BucketControl()
        msg_bucket.boom_up_duty = int(boom_up_duty)
        msg_bucket.boom_dn_duty = int(boom_dn_duty)
        msg_bucket.bucket_load_duty = int(bucket_load_duty)
        msg_bucket.bucket_dump_duty = int(bucket_dump_duty)
        self.bucket_control_msg_publisher.publish(msg_bucket)

        if self.bucket_log_cnt >= self.bucket_control_frequency:
            self.get_logger().info(
                f'Boom UP: {boom_up_duty:.1f} DOWN: {boom_dn_duty:.1f} '
                f'Bucket LOAD: {bucket_load_duty:.1f} DUMP: {bucket_dump_duty:.1f}'
            )
            self.bucket_log_cnt = 0
        else:
            self.bucket_log_cnt += 1


def main(args=None):
    rclpy.init(args=args)
    try:
        bucket_controller = BucketController()
        rclpy.spin(bucket_controller)
    except KeyboardInterrupt:
        bucket_controller.get_logger().warn('Keyboard interrrupt (SIGINT)')
    finally:
        bucket_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
