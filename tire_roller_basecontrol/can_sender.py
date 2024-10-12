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
# from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from roller_base_interfaces.msg import DriveControl
# from std_msgs.msg import Float32
# from std_msgs.msg import Int8
# from std_msgs.msg import String
# from roller_base_interfaces.msg import BucketControl
# from roller_base_interfaces.msg import DanfossFB


class CanSender(Node):

    def __init__(self):
        super().__init__('can_sender')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')
        self.candb_danfoss = cantools.db.load_file(
            get_package_share_directory('tire_roller_basecontrol') + '/r_danfoss_220420.dbc')
        self.can_msg_danfoss_steer_cmd = self.candb_danfoss.get_message_by_name('Steering_Cmd')
        # self.can_msg_danfoss_boom_cmd = self.candb_danfoss.get_message_by_name('D_Boom_Cmd')
        # self.can_msg_danfoss_bucket_cmd = self.candb_danfoss.get_message_by_name('D_Bkt_Cmd')
        # self.candb_anm = cantools.db.load_file(
        #     get_package_share_directory('tire_roller_basecontrol') + '/anm_240219.dbc')

        # self.bucket_msg_subscriber = self.create_subscription(
        #     BucketControl, 'bucket_control', self.recv_bucket, qos_profile_system_default)
        self.drive_msg_subscriber = self.create_subscription(
            DriveControl, 'drive_control', self.recv_drive, qos_profile_system_default)
        # self.danfoss_msg_subscriber = self.create_subscription(
        #     DanfossFB, 'danfoss_msg', self.recv_danfoss, qos_profile_sensor_data)
        # self.status_msg_subscriber = self.create_subscription(
        #     String, 'status', self.recv_status, qos_profile_system_default)
        # self.brake_msg_subscriber = self.create_subscription(
        #     Int8, 'brake_pos', self.recv_brake, qos_profile_sensor_data)
        # self.accel_msg_subscriber = self.create_subscription(
        #     Int8, 'accel_pos', self.recv_accel, qos_profile_sensor_data)

        self.can3_publish_frequency = 1
        self.can3_msg_publisher = self.create_publisher(
            Frame, 'can3/to_can_bus', qos_profile_system_default)
        self.timer_publish_can_bus3_msg = self.create_timer(
            1/self.can3_publish_frequency, self.publish_can_bus3_msg)

        self.can1_publish_frequency = 50
        self.can1_msg_publisher = self.create_publisher(
            Frame, 'can1/to_can_bus', qos_profile_system_default)
        self.timer_publish_can_bus1_msg = self.create_timer(
            1/self.can1_publish_frequency, self.publish_can_bus1_msg)

        # self.status = 'e_stop'
        # self.bucket_control_msg = BucketControl()
        # self.danfoss_fb_msg = DanfossFB()
        self.drive_control_msg = DriveControl()

        # self.encoder_degree = 0.0
        # self.brake_pos = 0
        # self.accel_pos = 0

        # self.response_msg_count = 0
        # self.can3_cnt = 0
        # self.can2_cnt = 0
        # self.initTreeze()

    # def initTreeze(self):
    #     data = self.can_msg_treeze_brk_cmd.encode({
    #         'MOTOR_CMD_EN': 1,
    #         'MOTOR_CMD_POS': 100,
    #         'MOTOR_CMD_POS_EN': 0
    #     })
    #     msg_brk = Frame()
    #     msg_brk.id = self.can_msg_treeze_brk_cmd.frame_id
    #     msg_brk.dlc = self.can_msg_treeze_brk_cmd.length
    #     msg_brk.data[:msg_brk.dlc] = list(data)
    #     msg_acc = Frame()
    #     msg_acc.id = self.can_msg_treeze_acc_cmd.frame_id
    #     msg_acc.dlc = self.can_msg_treeze_acc_cmd.length
    #     msg_acc.data[:msg_acc.dlc] = list(
    #         self.can_msg_treeze_acc_cmd.encode({
    #             'APS_Signal': 0,
    #             'APS_On': 0
    #         }))
    #     self.can3_msg_publisher.publish(msg_brk)
    #     self.can3_msg_publisher.publish(msg_acc)

    # def recv_status(self, msg: String):
    #     self.status = msg.data

    # def recv_bucket(self, msg: BucketControl):
    #     self.bucket_control_msg = msg

    def recv_drive(self, msg: DriveControl):
        self.drive_control_msg = msg
        self.get_logger().info(f'DRIVE CONTROL MSG: {msg}', throttle_duration_sec=0.99)

    # def recv_danfoss(self, msg: DanfossFB):
    #     self.danfoss_fb_msg = msg

    # def recv_accel(self, msg: Int8):
    #     self.accel_pos = msg.data

    # def recv_brake(self, msg: Int8):
    #     self.brake_pos = msg.data

    # def publish_can_bus1_msg(self):
    #     # make response status data
    #     mode_fb = 255
    #     if self.status == 'manual':
    #         mode_fb = 0
    #     elif self.status == 'remote':
    #         mode_fb = 1
    #     elif self.status == 'auto':
    #         mode_fb = 2

    #     status = 1
    #     if self.status == 'e_stop':
    #         status = 3

    #     data = self.can_msg_danfoss_steer_cmd.encode({
    #         'MODE_FB': mode_fb,
    #         'STATUS': status,
    #         'ERROR_CODE': 0,
    #         'STEER_ANGLE': self.encoder_degree,
    #         'ACC_FB': self.accel_pos,
    #         'BRAKE_FB': self.brake_pos,
    #         'CNT_FB': self.response_msg_count
    #     })
    #     msg_danfoss_steer_cmd = Frame()
    #     msg_danfoss_steer_cmd.id = self.can_msg_danfoss_steer_cmd.frame_id
    #     msg_danfoss_steer_cmd.dlc = self.can_msg_danfoss_steer_cmd.length
    #     msg_danfoss_steer_cmd.data[:msg_danfoss_steer_cmd.dlc] = list(data)
    #     self.can1_msg_publisher.publish(msg_danfoss_steer_cmd)

    #     data = self.can_msg_response_cylinderp.encode({
    #         'BOOM_UP_C_P': self.danfoss_fb_msg.boom_up_c_p,
    #         'BOOM_DOWN_C_P': self.danfoss_fb_msg.boom_dn_c_p,
    #         'BUCKET_IN_C_P': self.danfoss_fb_msg.bucket_in_c_p,
    #         'BUCKET_OUT_C_P': self.danfoss_fb_msg.bucket_out_c_p
    #     })
    #     msg_response_cylinderp = Frame()
    #     msg_response_cylinderp.id = self.can_msg_response_cylinderp.frame_id
    #     msg_response_cylinderp.dlc = self.can_msg_response_cylinderp.length
    #     msg_response_cylinderp.data[:msg_response_cylinderp.dlc] = list(data)
    #     self.can1_msg_publisher.publish(msg_response_cylinderp)

    #     self.response_msg_count = self.response_msg_count + 1
    #     if self.response_msg_count > 255:
    #         self.response_msg_count = 0
    #         self.get_logger().info(f'{msg_danfoss_steer_cmd}')
    #         self.get_logger().info(f'{msg_response_cylinderp}')

    # def publish_can_bus3_msg(self):
    #     # boom
    #     data = self.can_msg_danfoss_boom_cmd.encode({
    #         'Boom_up_duty': self.bucket_control_msg.boom_up_duty,
    #         'Boom_dn_duty': self.bucket_control_msg.boom_dn_duty
    #     })
    #     msg_boom = Frame()
    #     msg_boom.id = self.can_msg_danfoss_boom_cmd.frame_id
    #     msg_boom.dlc = self.can_msg_danfoss_boom_cmd.length
    #     msg_boom.data[:msg_boom.dlc] = list(data)

    #     # bucket
    #     data = self.can_msg_danfoss_bucket_cmd.encode({
    #         'Bkt_in_duty': self.bucket_control_msg.bucket_load_duty,
    #         'Bkt_out_duty': self.bucket_control_msg.bucket_dump_duty
    #     })
    #     msg_bucket = Frame()
    #     msg_bucket.id = self.can_msg_danfoss_bucket_cmd.frame_id
    #     msg_bucket.dlc = self.can_msg_danfoss_bucket_cmd.length
    #     msg_bucket.data[:msg_bucket.dlc] = list(data)

    #     # acc & brake
    #     if self.status == 'auto':
    #         brake_data = self.can_msg_treeze_brk_cmd.encode({
    #             'MOTOR_CMD_EN': 1,
    #             'MOTOR_CMD_POS': self.drive_control_msg.brake,
    #             'MOTOR_CMD_POS_EN': 1
    #         })
    #         acc_data = self.can_msg_treeze_acc_cmd.encode({
    #             'APS_Signal': self.drive_control_msg.accel,
    #             'APS_On': 1
    #         })
    #     elif self.status == 'remote':
    #         brake = 100
    #         if (self.drive_control_msg.fnr != 0 or
    #                 self.drive_control_msg.steer_left > 0 or
    #                 self.drive_control_msg.steer_right > 0):
    #             brake = 0
    #         brake_data = self.can_msg_treeze_brk_cmd.encode({
    #             'MOTOR_CMD_EN': 1,
    #             'MOTOR_CMD_POS': brake,
    #             'MOTOR_CMD_POS_EN': 1
    #         })
    #         acc_data = self.can_msg_treeze_acc_cmd.encode({
    #             'APS_Signal': self.drive_control_msg.accel,
    #             'APS_On': 1
    #         })
    #     elif self.status == 'manual' or self.status == 'idle':
    #         brake_data = self.can_msg_treeze_brk_cmd.encode({
    #             'MOTOR_CMD_EN': 1,
    #             'MOTOR_CMD_POS': 0,
    #             'MOTOR_CMD_POS_EN': 1
    #         })
    #         acc_data = self.can_msg_treeze_acc_cmd.encode({
    #             'APS_Signal': 0,
    #             'APS_On': 0
    #         })
    #     else:  # E-Stop
    #         brake_data = self.can_msg_treeze_brk_cmd.encode({
    #             'MOTOR_CMD_EN': 1,
    #             'MOTOR_CMD_POS': 100,
    #             'MOTOR_CMD_POS_EN': 1
    #         })
    #         acc_data = self.can_msg_treeze_acc_cmd.encode({
    #             'APS_Signal': 0,
    #             'APS_On': 0
    #         })
    #     msg_brk = Frame()
    #     msg_brk.id = self.can_msg_treeze_brk_cmd.frame_id
    #     msg_brk.dlc = self.can_msg_treeze_brk_cmd.length
    #     msg_brk.data[:msg_brk.dlc] = list(brake_data)
    #     msg_acc = Frame()
    #     msg_acc.id = self.can_msg_treeze_acc_cmd.frame_id
    #     msg_acc.dlc = self.can_msg_treeze_acc_cmd.length
    #     msg_acc.data[:msg_acc.dlc] = list(acc_data)

    #     self.can3_msg_publisher.publish(msg_boom)
    #     self.can3_msg_publisher.publish(msg_bucket)
    #     self.can3_msg_publisher.publish(msg_brk)
    #     self.can3_msg_publisher.publish(msg_acc)

    #     if self.can3_cnt == self.can3_publish_frequency / 2:
    #         self.get_logger().info(
    #             f'MSG BOOM: {msg_boom.data} BUCKET: {msg_bucket.data}\n'
    #             f'BRAKE: {msg_brk.data} ACCEL: {msg_acc.data}'
    #         )
    #         self.can3_cnt = 0
    #     else:
    #         self.can3_cnt = self.can3_cnt + 1
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
            f'DANFOSS STEER CMD: {msg_danfoss_steer_cmd}',
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
