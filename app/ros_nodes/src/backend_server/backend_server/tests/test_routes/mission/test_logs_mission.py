import asyncio
from unittest.mock import patch, MagicMock
from app.ros_nodes.src.backend_server.backend_server.websocket.logs import LogType
import pytest
from backend_server.websocket.event_handlers.logs_mission import LogSubscriber, start_record_logs


@patch('logs_mission.rclpy')
@patch('logs_mission.LogSubscriber')
@patch('logs_mission.send_log')
@patch('logs_mission.asyncio.sleep', return_value=None)
@pytest.fixture
def log_subscriber():
    return LogSubscriber()

#                                                   TEST CASES FOR get_robot_id
def test_get_robot_id_normal():
    log_subscriber = LogSubscriber()
    message = "LoremipsumLoremrobot99trr992924ur489mission_switch"
    assert log_subscriber.get_robot_id(message) == 99
    # Write your test cases here

def test_get_robot_id_no_id():
    log_subscriber = LogSubscriber()
    message = "LoremipsumLoremrobot99trr992924ur489mission_switch"
    assert log_subscriber.get_robot_id(message) == 0

#                                                   TEST CASES FOR get_severity
def test_get_severity_debug():
    log_msg = MagicMock()
    log_msg.level = 10
    assert LogSubscriber().get_severity(log_msg) == 'DEBUG'

def test_get_severity_info():
    log_msg = MagicMock()
    log_msg.level = 20
    assert LogSubscriber().get_severity(log_msg) == 'INFO'

def test_get_severity_warn():
    log_msg = MagicMock()
    log_msg.level = 30
    assert LogSubscriber().get_severity(log_msg) == 'WARN'

def test_get_severity_error():
    log_msg = MagicMock()
    log_msg.level = 40
    assert LogSubscriber().get_severity(log_msg) == 'ERROR'

def test_get_severity_fatal():
    log_msg = MagicMock()
    log_msg.level = 50
    assert LogSubscriber().get_severity(log_msg) == 'FATAL'

def test_get_severity_unknown():
    log_msg = MagicMock()
    log_msg.level = 999
    assert LogSubscriber().get_severity(log_msg) == 'UNKNOWN'

#                                                   TEST CASES FOR get_event_type

def test_get_event_type_command(instance):
    log_msg = MagicMock(msg="Some command message")
    event_type = instance.get_event_type(log_msg)
    assert event_type == LogType.COMMAND

def test_get_event_type_log(instance):
    log_msg = MagicMock(msg="Some regular log message")
    event_type = instance.get_event_type(log_msg)
    assert event_type == LogType.LOG

def test_get_event_type_empty_msg(instance):
    log_msg = MagicMock(msg="")
    event_type = instance.get_event_type(log_msg)
    assert event_type == LogType.LOG



def test_listener_callback():
    log_subscriber = LogSubscriber()
    # Mock the raw_log object
    raw_log = MagicMock()
    raw_log.msg = 'Incoming request, command: stop, current state: State.ON'
    raw_log.name = 'robot1'
    log_subscriber.listener_callback(raw_log)
    assert log_subscriber.isFinished == True
    # Write more test cases based on your requirements

@patch('logs_mission.send_log')
@patch('logs_mission.rclpy.ok')
@patch('logs_mission.rclpy.spin_once')
@patch('logs_mission.rclpy.shutdown')
def test_start_record_logs(mock_shutdown, mock_spin_once, mock_ok, mock_send_log):
    # Mock necessary objects and functions
    mock_ok.side_effect = [True, False]  # To exit the loop
    log_subscriber = MagicMock()
    log_subscriber.isNewLog = True
    log_subscriber.lastRosLog = ("Test message", 1, "command")
    with patch('logs_mission.LogSubscriber', return_value=log_subscriber):
        start_record_logs()
    assert mock_send_log.called
    assert mock_shutdown.called

