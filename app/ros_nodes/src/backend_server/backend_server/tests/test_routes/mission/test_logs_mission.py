import asyncio
from unittest.mock import patch, MagicMock
from app.ros_nodes.src.backend_server.backend_server.websocket.logs import LogType
import pytest
from backend_server.websocket.event_handlers.logs_mission import LogSubscriber, RosLog, start_record_logs


@patch('logs_mission.rclpy')
@patch('logs_mission.LogSubscriber')
@patch('logs_mission.send_log')
@patch('logs_mission.asyncio.sleep', return_value=None)
@pytest.fixture
def log_subscriber():
    return LogSubscriber()

#                                                   TEST CASES FOR get_robot_id
def test_get_robot_id(log_subscriber):    
    message = "LoremipsumLoremrobot99trr992924ur489mission_switch"
    assert log_subscriber.get_robot_id(message) == 99

def test_get_robot_id_no_id(log_subscriber):    
    message = "LoremipsumLoremrobottrr992924ur489mission_switch"
    assert log_subscriber.get_robot_id(message) == 0

#                                                   TEST CASES FOR get_severity
def test_get_severity_debug(log_subscriber):
    log_msg = MagicMock()
    log_msg.level = 10
    assert log_subscriber.get_severity(log_msg) == 'DEBUG'

def test_get_severity_info(log_subscriber):
    log_msg = MagicMock()
    log_msg.level = 20
    assert log_subscriber.get_severity(log_msg) == 'INFO'

def test_get_severity_warn(log_subscriber):
    log_msg = MagicMock()
    log_msg.level = 30
    assert log_subscriber.get_severity(log_msg) == 'WARN'

def test_get_severity_error(log_subscriber):
    log_msg = MagicMock()
    log_msg.level = 40
    assert log_subscriber.get_severity(log_msg) == 'ERROR'

def test_get_severity_fatal(log_subscriber):
    log_msg = MagicMock()
    log_msg.level = 50
    assert log_subscriber.get_severity(log_msg) == 'FATAL'

def test_get_severity_unknown(log_subscriber):
    log_msg = MagicMock()
    log_msg.level = 999
    assert log_subscriber.get_severity(log_msg) == 'UNKNOWN'

#                                                   TEST CASES FOR get_event_type

def test_get_event_type_command(log_subscriber):
    log_msg = MagicMock(msg="Some command message")
    event_type = log_subscriber.get_event_type(log_msg)
    assert event_type == LogType.COMMAND

def test_get_event_type_log(log_subscriber):
    log_msg = MagicMock(msg="Some regular log message")
    event_type = log_subscriber.get_event_type(log_msg)
    assert event_type == LogType.LOG

def test_get_event_type_empty_msg(log_subscriber):
    log_msg = MagicMock(msg="")
    event_type = log_subscriber.get_event_type(log_msg)
    assert event_type == LogType.LOG


#                                                   TEST CASES FOR listener_callback

@patch(LogSubscriber, 'get_robot_id','get_severity','get_event_type')
def test_listener_callback(mock_get_robot_id, mock_get_severity, mock_get_event_type ,log_subscriber):    
    # Mock the raw_log object
    raw_log = MagicMock()
    raw_log.msg = 'This is a normal LOG message'
    raw_log.name = 'publisher'
    mock_get_robot_id.return_value = 0
    mock_get_event_type.return_value = LogType.LOG
    mock_get_severity.return_value = 'DEBUG'

    log_subscriber.listener_callback(raw_log)
    assert log_subscriber.isFinished == False
    assert log_subscriber.isNewLog == True
    assert log_subscriber.lastRosLog == RosLog(0,"DEBUG: publisher: This is a normal LOG message", LogType.LOG)

@patch(LogSubscriber, 'get_robot_id','get_severity','get_event_type')
def test_listener_callback_stop(mock_get_robot_id, mock_get_severity, mock_get_event_type ,log_subscriber):    
    # Mock the raw_log object
    raw_log = MagicMock()
    raw_log.msg = 'Incoming request, command: stop, current state: State.ON'
    raw_log.name = 'robot1'
    mock_get_robot_id.return_value = 1
    mock_get_event_type.return_value = LogType.COMMAND
    mock_get_severity.return_value = 'INFO'

    log_subscriber.listener_callback(raw_log)
    assert log_subscriber.isFinished == True
    assert log_subscriber.isNewLog == True
    assert log_subscriber.lastRosLog == RosLog(1,"INFO: robot1: Incoming request, command: stop, current state: State.ON", LogType.COMMAND)

#                                                   TEST CASES FOR start_record_logs

# @patch('logs_mission.send_log')
# @patch('logs_mission.rclpy.ok')
# @patch('logs_mission.rclpy.spin_once')
# @patch('logs_mission.rclpy.shutdown')
# def test_start_record_logs(mock_shutdown, mock_spin_once, mock_ok, mock_send_log):
#     # Mock necessary objects and functions
#     mock_ok.side_effect = [True, False]  # To exit the loop
#     log_subscriber = MagicMock()
#     log_subscriber.isNewLog = True
#     log_subscriber.lastRosLog = ("Test message", 1, "command")
#     with patch('logs_mission.LogSubscriber', return_value=log_subscriber):
#         start_record_logs()
#     assert mock_send_log.called
#     assert mock_shutdown.called

