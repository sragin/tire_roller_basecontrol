# Copyright 2024 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.
#
# 타이어롤러용 유한상태머신 노드
# python-statemachine 설치필요 (https://pypi.org/project/python-statemachine/)

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from roller_base_interfaces.msg import RemoteControl
from statemachine import State, StateMachine
from statemachine.exceptions import TransitionNotAllowed
from std_msgs.msg import Bool
from std_msgs.msg import String


class TireRollerStateMachine(StateMachine):
    idle = State(initial=True)
    e_stop = State()
    manual = State()
    remote = State()
    auto = State()

    to_idle = e_stop.to(idle)
    to_estop = (
        remote.to(e_stop)
        | auto.to(e_stop)
        | e_stop.to(e_stop)
    )
    to_manual = (
        idle.to(manual)
        | manual.to(manual)
        | remote.to(manual)
        | auto.to(manual)
        | e_stop.to(manual)
    )
    to_remote = (
        manual.to(remote)
        | auto.to(remote)
    )
    to_auto = (
        manual.to(auto)
        | remote.to(auto)
    )

    def __init__(self, nav):
        self.navigator: Navigator = nav
        super(TireRollerStateMachine, self).__init__()

    def on_enter_idle(self):
        self.navigator.get_logger().warn(
            'Idle state',
            throttle_duration_sec=1.0
        )

    def on_enter_e_stop(self):
        self.navigator.get_logger().warn(
            'E-Stop state',
            throttle_duration_sec=1.0
        )

    def on_enter_manual(self):
        self.navigator.get_logger().warn(
            'Manual state',
            throttle_duration_sec=1.0
        )

    def on_enter_remote(self):
        self.navigator.get_logger().warn(
            'Remote state',
            throttle_duration_sec=1.0
        )

    def on_enter_auto(self):
        self.navigator.get_logger().warn(
            'Auto state',
            throttle_duration_sec=1.0
        )


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')

        self.remote_msg_subscriber = self.create_subscription(
            RemoteControl, 'remote_msg', self.recv_remote, qos_profile_sensor_data)
        self.estop_wireless_subscriber = self.create_subscription(
            Bool, 'estop_wireless', self.recv_wireless, qos_profile_system_default)

        self.status_publisher = self.create_publisher(String, 'status', qos_profile_system_default)
        self.timer_publish_status = self.create_timer(1/10, self.publish_status)
        self.timer_state_manager = self.create_timer(1/10, self.manage_state)

        self.sm = TireRollerStateMachine(self)
        self.remote_msg = RemoteControl()
        self.estop_wireless = Bool()

    def recv_remote(self, msg: RemoteControl):
        self.remote_msg = msg

    def recv_wireless(self, msg: Bool):
        self.estop_wireless = msg

    def publish_status(self):
        msg = String()
        msg.data = self.sm.current_state.id
        self.status_publisher.publish(msg)

    def manage_state(self):
        try:
            if (self.estop_wireless.data or self.remote_msg.remote_switch[12] == 1):
                self.sm.to_estop()
            elif not self.estop_wireless.data and self.remote_msg.remote_switch[12] != 1 \
                    and self.sm.current_state.id == 'e_stop':
                self.sm.to_idle()
            elif self.remote_msg.remote_switch[22] == 1:
                self.sm.to_manual()
            elif self.remote_msg.remote_switch[21] == 1 \
                and self.remote_msg.remote_switch[19] == 0:
                self.sm.to_remote()
            elif self.remote_msg.remote_switch[21] == 1 \
                and self.remote_msg.remote_switch[19] == 1:
                self.sm.to_auto()
        except TransitionNotAllowed as e:
            self.get_logger().warn(
                f'{e}',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    try:
        navigator = Navigator()
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().warn('Keyboard interrrupt (SIGINT)')
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
