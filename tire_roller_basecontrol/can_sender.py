# Copyright 2024 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.
#
# 타이어롤러용 CAN 메시지 송신 노드


from ament_index_python import get_package_share_directory
from can_msgs.msg import Frame
import cantools
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from roller_base_interfaces.msg import DriveControl


class CanSender(Node):

    def __init__(self):
        super().__init__('can_sender')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')
        self.candb_danfoss = cantools.db.load_file(
            get_package_share_directory('tire_roller_basecontrol') + '/r_danfoss_220420.dbc')
        self.can_msg_danfoss_steer_cmd = self.candb_danfoss.get_message_by_name('Steering_Cmd')

        self.drive_msg_subscriber = self.create_subscription(
            DriveControl, 'drive_control', self.recv_drive, qos_profile_sensor_data)
        self.can0_publish_frequency = 50
        self.can0_msg_publisher = self.create_publisher(
            Frame, 'can0/to_can_bus', qos_profile_system_default)
        self.timer_publish_can_bus0_msg = self.create_timer(
            1/self.can0_publish_frequency, self.publish_can_bus0_msg)

        self.can1_publish_frequency = 50
        self.can1_msg_publisher = self.create_publisher(
            Frame, 'can1/to_can_bus', qos_profile_system_default)
        self.timer_publish_can_bus1_msg = self.create_timer(
            1/self.can1_publish_frequency, self.publish_can_bus1_msg)

        self.can3_publish_frequency = 1
        self.can3_msg_publisher = self.create_publisher(
            Frame, 'can3/to_can_bus', qos_profile_system_default)
        self.timer_publish_can_bus3_msg = self.create_timer(
            1/self.can3_publish_frequency, self.publish_can_bus3_msg)

        self.drive_control_msg = DriveControl()

        self.can0_cnt = 0

    def recv_drive(self, msg: DriveControl):
        self.drive_control_msg = msg
        self.get_logger().info(f'DRIVE CONTROL MSG: {msg}', throttle_duration_sec=0.99)

    # lever 메시지 전송
    def publish_can_bus0_msg(self):
        # 레버 전후진
        if self.drive_control_msg.fnr == 1:  # forward
            joy_pos = 5
        elif self.drive_control_msg.fnr == 2:  # reverse
            joy_pos = 6
        else:
            joy_pos = 4
        accel = self.drive_control_msg.accel

        msg_101 = Frame(
            id=0x101,
            is_extended=False,
            dlc=8,
            data=[0x00, joy_pos, accel, 0xFF, 0x00, 0x00, 0x00, self.can0_cnt])
        self.can0_cnt = self.can0_cnt + 1
        self.can0_cnt = self.can0_cnt % 256
        msg_102 = Frame(
            id=0x102,
            is_extended=False,
            dlc=8,
            data=[0, 0, 0, 0, 0, 0, 0, 0])
        msg_102.data[0] = 255 - msg_101.data[0]
        msg_102.data[1] = 255 - msg_101.data[1]
        msg_102.data[2] = 255 - msg_101.data[2]
        msg_102.data[3] = 255 - msg_101.data[3]
        msg_102.data[4] = 255 - msg_101.data[4]
        msg_102.data[5] = 255 - msg_101.data[5]
        msg_102.data[6] = 255 - msg_101.data[6]
        msg_102.data[7] = 255 - msg_101.data[7]
        msg_103 = Frame(
            id=0x103,
            is_extended=False,
            dlc=8,
            data=[0, 0, 0, 0, 0, 0, 0, msg_101.data[-1]])
        msg_104 = Frame(
            id=0x104,
            is_extended=False,
            dlc=8,
            data=[0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, msg_102.data[-1]])
        msg_105 = Frame(
            id=0x105,
            is_extended=False,
            dlc=8,
            data=[0, 0x5C, 0x5F, 0, 0, 0, 0, msg_101.data[-1]])
        msg_106 = Frame(
            id=0x106,
            is_extended=False,
            dlc=8,
            data=[0xFF, 0xA3, 0xA0, 0xFF, 0xFF, 0xFF, 0xFF, msg_102.data[-1]])
        msg_70 = Frame(
            id=0x70B,
            is_extended=False,
            dlc=1,
            data=[0x05, 0, 0, 0, 0, 0, 0, 0])

        self.can0_msg_publisher.publish(msg_70)
        self.can0_msg_publisher.publish(msg_101)
        self.can0_msg_publisher.publish(msg_102)
        self.can0_msg_publisher.publish(msg_103)
        self.can0_msg_publisher.publish(msg_104)
        self.can0_msg_publisher.publish(msg_105)
        self.can0_msg_publisher.publish(msg_106)
        self.get_logger().info(
            f'LEVER CMD:\t\tid:{msg_101.id}, data:{msg_101.data}',
            throttle_duration_sec=0.99)

    # steering cmd 전송
    def publish_can_bus1_msg(self):
        data = self.can_msg_danfoss_steer_cmd.encode({
            'Left_duty': self.drive_control_msg.steer_left,
            'Right_duty': self.drive_control_msg.steer_right
        })
        msg_danfoss_steer_cmd = Frame()
        msg_danfoss_steer_cmd.id = self.can_msg_danfoss_steer_cmd.frame_id
        msg_danfoss_steer_cmd.dlc = self.can_msg_danfoss_steer_cmd.length
        msg_danfoss_steer_cmd.data[:msg_danfoss_steer_cmd.dlc] = list(data)
        self.can1_msg_publisher.publish(msg_danfoss_steer_cmd)
        self.get_logger().info(
            f'DANFOSS STEER CMD:\tid:{msg_danfoss_steer_cmd.id}, '
            f'data:{msg_danfoss_steer_cmd.data}',
            throttle_duration_sec=0.99)

    # ENCODER를 ENABLE 해준다
    def publish_can_bus3_msg(self):
        msg_sync = Frame()
        msg_sync.id = 0x80
        msg_sync.dlc = 1
        msg_sync.data[:msg_sync.dlc] = [0]
        self.can3_msg_publisher.publish(msg_sync)
        self.get_logger().info(f'MSG SYNC: {msg_sync}', once=True)


def main(args=None):
    rclpy.init(args=args)
    try:
        can_sender = CanSender()
        rclpy.spin(can_sender)
    except KeyboardInterrupt:
        can_sender.get_logger().warn('Keyboard interrrupt (SIGINT)')
    finally:
        can_sender.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
