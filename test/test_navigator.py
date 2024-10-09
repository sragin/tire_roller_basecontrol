import pytest

import rclpy
from statemachine.exceptions import TransitionNotAllowed
from std_msgs.msg import Bool
from rollertire_control.navigator import Navigator
from wheelloader_interfaces.msg import AnmControl
from wheelloader_interfaces.msg import RemoteControl


@pytest.fixture
def navigator():
    rclpy.init()
    nav = Navigator()
    yield nav
    nav.destroy_node()
    rclpy.shutdown()


def test_idle_state_transitions(navigator):
    sm = navigator.sm

    sm.current_state = sm.idle
    sm.to_manual()
    assert sm.current_state == sm.manual

    sm.current_state = sm.idle
    with pytest.raises(TransitionNotAllowed):
        sm.to_remote()

    sm.current_state = sm.idle
    with pytest.raises(TransitionNotAllowed):
        sm.to_auto()

    sm.current_state = sm.idle
    with pytest.raises(TransitionNotAllowed):
        sm.to_estop()


def test_estop_state_transitions(navigator):
    sm = navigator.sm

    sm.current_state = sm.e_stop
    sm.to_idle()
    assert sm.current_state == sm.idle

    sm.current_state = sm.e_stop
    sm.to_manual()
    assert sm.current_state == sm.manual

    sm.current_state = sm.e_stop
    with pytest.raises(TransitionNotAllowed):
        sm.to_remote()

    sm.current_state = sm.e_stop
    with pytest.raises(TransitionNotAllowed):
        sm.to_auto()


def test_manual_state_transitions(navigator):
    sm = navigator.sm

    sm.current_state = sm.manual
    with pytest.raises(TransitionNotAllowed):
        sm.to_idle()

    sm.current_state = sm.manual
    with pytest.raises(TransitionNotAllowed):
        sm.to_estop()

    sm.current_state = sm.manual
    sm.to_remote()
    assert sm.current_state == sm.remote

    sm.current_state = sm.manual
    sm.to_auto()
    assert sm.current_state == sm.auto


def test_remote_state_transitions(navigator):
    sm = navigator.sm

    sm.current_state = sm.remote
    with pytest.raises(TransitionNotAllowed):
        sm.to_idle()

    sm.current_state = sm.remote
    sm.to_estop()
    assert sm.current_state == sm.e_stop

    sm.current_state = sm.remote
    sm.to_manual()
    assert sm.current_state == sm.manual

    sm.current_state = sm.remote
    sm.to_auto()
    assert sm.current_state == sm.auto


def test_auto_state_transitions(navigator):
    sm = navigator.sm

    sm.current_state = sm.auto
    with pytest.raises(TransitionNotAllowed):
        sm.to_idle()

    sm.current_state = sm.auto
    sm.to_estop()
    assert sm.current_state == sm.e_stop

    sm.current_state = sm.auto
    sm.to_remote()
    assert sm.current_state == sm.remote

    sm.current_state = sm.auto
    with pytest.raises(TransitionNotAllowed):
        sm.to_auto()


def test_invalid_transitions(navigator):
    sm = navigator.sm

    sm.current_state = sm.idle
    with pytest.raises(TransitionNotAllowed):
        sm.to_idle()

    sm.current_state = sm.e_stop
    with pytest.raises(TransitionNotAllowed):
        sm.to_remote()
    with pytest.raises(TransitionNotAllowed):
        sm.to_auto()

    sm.current_state = sm.remote
    with pytest.raises(TransitionNotAllowed):
        sm.to_idle()
    with pytest.raises(TransitionNotAllowed):
        sm.to_remote()

    sm.current_state = sm.auto
    with pytest.raises(TransitionNotAllowed):
        sm.to_idle()
    with pytest.raises(TransitionNotAllowed):
        sm.to_auto()


def test_state_machine_idle(navigator):
    sm = navigator.sm
    assert sm.current_state == sm.idle


@pytest.fixture
def remote_switch():
    remote_switch = [0]*27
    return remote_switch


def test_state_machine_estop(navigator, remote_switch):
    sm = navigator.sm

    sm.current_state = sm.remote
    navigator.recv_anm(AnmControl(e_stop=True))
    navigator.manage_state()
    print(navigator.anm_msg)
    assert sm.current_state == sm.e_stop

    sm.current_state = sm.remote
    navigator.recv_anm(AnmControl(e_stop=False))
    navigator.recv_wireless(Bool(data=True))
    navigator.manage_state()
    assert sm.current_state == sm.e_stop

    sm.current_state = sm.remote
    remote_switch[12] = 1  # remote sw e-stop button
    navigator.recv_anm(AnmControl(e_stop=False))
    navigator.recv_wireless(Bool(data=False))
    navigator.recv_remote(RemoteControl(remote_switch=remote_switch))
    navigator.manage_state()
    assert sm.current_state == sm.e_stop


def test_state_machine_manual_switch(navigator, remote_switch):
    sm = navigator.sm
    remote_switch[22] = 1
    navigator.recv_remote(RemoteControl(remote_switch=remote_switch))
    navigator.manage_state()
    assert sm.current_state == sm.manual


def test_state_machine_remote_switch(navigator, remote_switch):
    sm = navigator.sm

    remote_switch[22] = 1
    navigator.recv_remote(RemoteControl(remote_switch=remote_switch))
    navigator.manage_state()
    assert sm.current_state == sm.manual
