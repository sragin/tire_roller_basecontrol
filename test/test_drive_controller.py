import pytest

import rclpy
from roller_base_interfaces.msg import RemoteControl
from tire_roller_basecontrol.drive_controller import DriveController


@pytest.fixture
def drive_controller():
    rclpy.init()
    drv = DriveController()
    yield drv
    drv.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def remote_switch():
    remote_switch = [0]*27
    return remote_switch


@pytest.fixture
def remote_joystick():
    remote_joystick = [0]*8
    return remote_joystick


def test_drive_neutral(drive_controller: DriveController, remote_switch, remote_joystick):
    drive_controller.status = 'remote'
    remote_switch[1] = 0
    remote_joystick[0] = 127
    drive_controller.recv_remote(
        RemoteControl(
            remote_switch=remote_switch,
            remote_joystick=remote_joystick
        )
    )
    drive_controller.publish_drive()

    assert drive_controller.drive_msg.fnr == 0
    assert drive_controller.drive_msg.accel == 0


def test_drive_forward(drive_controller: DriveController, remote_switch, remote_joystick):
    drive_controller.status = 'remote'
    remote_switch[1] = 1
    remote_joystick[0] = 0
    drive_controller.recv_remote(
        RemoteControl(
            remote_switch=remote_switch,
            remote_joystick=remote_joystick
        )
    )
    drive_controller.publish_drive()

    assert drive_controller.drive_msg.fnr == 1
    assert drive_controller.drive_msg.accel == 100


def test_drive_reverse(drive_controller: DriveController, remote_switch, remote_joystick):
    drive_controller.status = 'remote'
    remote_switch[0] = 1
    remote_joystick[0] = 255
    drive_controller.recv_remote(
        RemoteControl(
            remote_switch=remote_switch,
            remote_joystick=remote_joystick
        )
    )
    drive_controller.publish_drive()

    assert drive_controller.drive_msg.fnr == 2
    assert drive_controller.drive_msg.accel == 100


def test_steer_right(drive_controller: DriveController, remote_switch, remote_joystick):
    drive_controller.status = 'remote'
    remote_switch[9] = 1
    remote_joystick[3] = 0
    drive_controller.recv_remote(
        RemoteControl(
            remote_switch=remote_switch,
            remote_joystick=remote_joystick
        )
    )
    drive_controller.publish_drive()

    assert drive_controller.drive_msg.steer_left == 0
    assert drive_controller.drive_msg.steer_right == 100


def test_steer_left(drive_controller: DriveController, remote_switch, remote_joystick):
    drive_controller.status = 'remote'
    remote_switch[8] = 1
    remote_joystick[3] = 255
    drive_controller.recv_remote(
        RemoteControl(
            remote_switch=remote_switch,
            remote_joystick=remote_joystick
        )
    )
    drive_controller.publish_drive()

    assert drive_controller.drive_msg.steer_left == 100
    assert drive_controller.drive_msg.steer_right == 0
