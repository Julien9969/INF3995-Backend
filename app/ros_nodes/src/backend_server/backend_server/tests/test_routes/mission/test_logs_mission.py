import asyncio, fastapi
from unittest.mock import  Mock, PropertyMock, patch, MagicMock
import pytest
from backend_server.ros.subscribers.logs import LogSubscriber, RosLog
from backend_server.ros.managers.logs import LogManager
from backend_server.classes.common import LogType



#                                                   TEST CASES FOR get_robot_id
@patch("rclpy.node.Node.create_subscription")
def test_get_robot_id(create_subscription_mock): 
    create_subscription_mock.return_value = MagicMock()
    log_subscriber = LogSubscriber()
    message = "LoremipsumLoremrobot99trr992924ur489mission_switch"
    result = log_subscriber.get_robot_id(message)
    log_subscriber = LogSubscriber()
    assert result == 99


@patch("rclpy.node.Node.create_subscription")
def test_get_robot_id_no_id(create_subscription_mock):
    create_subscription_mock.return_value = MagicMock()
    message = "LoremipsumLoremrobottrr992924ur489mission_switch"
    log_subscriber = LogSubscriber()
    assert log_subscriber.get_robot_id(message) == 0

#                                                   TEST CASES FOR get_severity
@patch("rclpy.node.Node.create_subscription")
def test_get_severity_debug(create_subscription_mock): 
    create_subscription_mock.return_value = MagicMock()
    level = 10
    log_subscriber = LogSubscriber()
    assert log_subscriber.get_severity(level) == 'DEBUG'

@patch("rclpy.node.Node.create_subscription")
def test_get_severity_info(create_subscription_mock): 
    create_subscription_mock.return_value = MagicMock()
    level = 20
    log_subscriber = LogSubscriber()
    assert log_subscriber.get_severity(level) == 'INFO'

@patch("rclpy.node.Node.create_subscription")
def test_get_severity_warn(create_subscription_mock): 
    create_subscription_mock.return_value = MagicMock()
    level = 30
    log_subscriber = LogSubscriber()
    assert log_subscriber.get_severity(level) == 'WARN'

@patch("rclpy.node.Node.create_subscription")
def test_get_severity_error(create_subscription_mock): 
    create_subscription_mock.return_value = MagicMock()
    level = 40
    log_subscriber = LogSubscriber()
    assert log_subscriber.get_severity(level) == 'ERROR'

@patch("rclpy.node.Node.create_subscription")
def test_get_severity_fatal(create_subscription_mock): 
    create_subscription_mock.return_value = MagicMock()
    level = 50
    log_subscriber = LogSubscriber()
    assert log_subscriber.get_severity(level) == 'FATAL'

@patch("rclpy.node.Node.create_subscription")
def test_get_severity_unknown(create_subscription_mock): 
    create_subscription_mock.return_value = MagicMock()
    level = 999
    log_subscriber = LogSubscriber()
    assert log_subscriber.get_severity(level) == 'UNKNOWN'

#                                                   TEST CASES FOR get_event_type

@patch("rclpy.node.Node.create_subscription")
def test_get_event_type_command(create_subscription_mock): 
    create_subscription_mock.return_value = MagicMock()
    log_msg = "Some command message"
    log_subscriber = LogSubscriber()
    event_type = log_subscriber.get_event_type(log_msg)
    assert event_type == LogType.COMMAND

@patch("rclpy.node.Node.create_subscription")
def test_get_event_type_log(create_subscription_mock): 
    create_subscription_mock.return_value = MagicMock()
    log_msg ="Some regular log message"
    log_subscriber = LogSubscriber()
    event_type = log_subscriber.get_event_type(log_msg)
    assert event_type == LogType.LOG

@patch("rclpy.node.Node.create_subscription")
def test_get_event_type_empty_msg(create_subscription_mock): 
    create_subscription_mock.return_value = MagicMock()
    log_msg = ""
    log_subscriber = LogSubscriber()
    event_type = log_subscriber.get_event_type(log_msg)
    assert event_type == LogType.LOG


#                                                   TEST CASES FOR listener_callback

@patch("rclpy.node.Node.create_subscription")
def test_listener_callback(create_subscription_mock ): 
    create_subscription_mock.return_value = MagicMock()
    log_subscriber = LogSubscriber()

    raw_log = Mock()
    raw_log.name = "publisher"
    raw_log.msg = "This is a normal LOG message"
    raw_log.level = 10
    
    log_subscriber.listener_callback(raw_log)
    assert log_subscriber.is_new_log == True
    assert log_subscriber.last_ros_log == RosLog(0,"DEBUG: publisher: This is a normal LOG message", LogType.LOG)

#                                                   TEST CASES FOR LogManager
    
#                                                   TEST CASES FOR start_record_logs

@pytest.mark.asyncio
@patch.object(fastapi.concurrency, 'run_in_threadpool', return_value=None)
@patch("backend_server.ros.managers.logs.LogSubscriber", return_value=MagicMock())
@patch("backend_server.ros.managers.logs.send_log")
async def test_start_record_logs(send_log_mock, logSubscriber_mock, run_in_threadpool_mock):
    
    async def mock_side_effect():
        await asyncio.sleep(1)

    run_in_threadpool_mock.side_effect = mock_side_effect 

    with patch('backend_server.ros.managers.logs.LogManager.is_recording', new_callable=PropertyMock) as is_recording_mock:
        is_recording_mock.side_effect = [True, False]

        logSubscriber_mock.return_value.isNewLog = MagicMock(return_value=True)
        logSubscriber_mock.last_ros_log = MagicMock()
        logSubscriber_mock.last_ros_log.message.ret = "message"
        logSubscriber_mock.last_ros_log.source_id = 88
        logSubscriber_mock.last_ros_log.logType = LogType.LOG

        task = asyncio.create_task(LogManager.start_record_logs())

        try:
            await asyncio.wait_for(task, timeout=0.1) 
        except asyncio.TimeoutError:
            pass

        send_log_mock.assert_called()

def test_stop_record_logs():
    LogManager.is_recording = True
    LogManager.stop_record_logs()
    assert not LogManager.is_recording
