# Copyright 2024 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.
#
# 타이어롤러용 주행 제어 노드
# 상위제어기나 리모콘에서 입력된 메시지를 변환하여 롤러와 댄포스로 전송한다


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from roller_base_interfaces.msg import AnmControl
from roller_base_interfaces.msg import DriveControl
from roller_base_interfaces.msg import RemoteControl
from std_msgs.msg import String


class DriveController(Node):

    def __init__(self):
        super().__init__('drive_controller')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')

        self.anm_msg = AnmControl()
        self.remote_msg = RemoteControl()
        self.drive_msg = DriveControl()
        self.status = 'idle'
        self.drive_log_cnt = 0

        self.anm_msg_subscriber = self.create_subscription(
            AnmControl, 'anm_msg', self.recv_anm, qos_profile_system_default)
        self.remote_msg_subscriber = self.create_subscription(
            RemoteControl, 'remote_msg', self.recv_remote, qos_profile_sensor_data)
        self.status_msg_subscriber = self.create_subscription(
            String, 'status', self.recv_status, qos_profile_system_default)

        self.drive_control_frequency = 20
        self.drive_control_msg_publisher = self.create_publisher(
            DriveControl, 'drive_control', qos_profile_system_default)
        self.timer_publish_drive = self.create_timer(
            1/self.drive_control_frequency, self.publish_drive)

    def recv_status(self, msg: String):
        self.status = msg.data
        self.get_logger().info(f'Status: {self.status}', throttle_duration_sec=1.0)

    def recv_anm(self, msg: AnmControl):
        self.anm_msg = msg
        self.get_logger().info(f'ANM cmd:{msg}', throttle_duration_sec=1.0)

    def recv_remote(self, msg: RemoteControl):
        self.remote_msg = msg

    def publish_drive(self):
        if self.status == 'auto':
            accel = self.anm_msg.accel
            brake = self.anm_msg.brake
            steer_left = self.anm_msg.steer_left
            steer_right = self.anm_msg.steer_right
            fnr = self.anm_msg.fnr
        elif self.status == 'remote':
            msg = self.remote_msg
            # drive
            if msg.remote_joystick[0] < 127 and msg.remote_switch[1]:
                accel = (127 - msg.remote_joystick[0]) / 126 * 255
                brake = 0
                fnr = 1  # forward
            elif msg.remote_joystick[0] > 127 and msg.remote_switch[0]:
                accel = (msg.remote_joystick[0] - 127) / 127 * 255
                brake = 0
                fnr = 2  # reverse
            else:
                accel = 0
                brake = 0
                fnr = 0

            # steer
            if msg.remote_joystick[3] < 127 and msg.remote_switch[9]:
                steer_right = (127 - msg.remote_joystick[3]) / 126 * 50
                steer_left = 0
            elif msg.remote_joystick[3] > 127 and msg.remote_switch[8]:
                steer_right = 0
                steer_left = (msg.remote_joystick[3] - 127) / 127 * 50
            else:
                steer_left = 0
                steer_right = 0
        else:
            accel = 0
            brake = 0
            fnr = 0
            steer_left = 0
            steer_right = 0

        self.drive_msg = DriveControl()
        self.drive_msg.accel = int(accel)
        self.drive_msg.brake = int(brake)
        self.drive_msg.fnr = fnr
        self.drive_msg.steer_left = int(steer_left)
        self.drive_msg.steer_right = int(steer_right)
        self.drive_control_msg_publisher.publish(self.drive_msg)

        self.get_logger().info(
            f'Drive: {accel} Brake: {brake} FNR: {fnr} '
            f'Steer Left: {steer_left} Right: {steer_right}',
            throttle_duration_sec=0.99)


def main(args=None):
    rclpy.init(args=args)
    try:
        drive_controller = DriveController()
        rclpy.spin(drive_controller)
    except KeyboardInterrupt:
        drive_controller.get_logger().warn('Keyboard interrrupt (SIGINT)')
    finally:
        drive_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
