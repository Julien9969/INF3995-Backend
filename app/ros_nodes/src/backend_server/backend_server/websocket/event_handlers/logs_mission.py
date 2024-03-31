import logging
import re
from collections import namedtuple

import rclpy
from fastapi.concurrency import run_in_threadpool
from rcl_interfaces.msg import Log
from rclpy.node import Node

from ..logs import LogType, send_log

logging.basicConfig(level=logging.DEBUG)
RosLog = namedtuple('RosLog', ['source_id', 'message', 'logType'])


class LogSubscriber(Node):

    def __init__(self):
        super().__init__('logs_subscriber')
        self.subscription = self.create_subscription(
            Log,
            'rosout',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.last_ros_log: RosLog = None
        self.is_new_log = False

    def get_robot_id(self, name: str):

        pattern = r"robot(\d+)"
        match = re.search(pattern, name)
        if match:
            robot_id = match.group(1)
            return int(robot_id)
        else:
            return 0

    def get_severity(self, level: int):
        severity_levels = {
            10: 'DEBUG',
            20: 'INFO',
            30: 'WARN',
            40: 'ERROR',
            50: 'FATAL'
        }
        severity = severity_levels.get(level, 'UNKNOWN')
        return severity

    def get_event_type(self, msg):
        if "command" in msg:
            event_type = LogType.COMMAND
        else:
            event_type = LogType.LOG
        return event_type

    def listener_callback(self, raw_log):
        source_id: int = self.get_robot_id(raw_log.name)
        logType = self.get_event_type(raw_log.msg)
        formatted_message = f"{self.get_severity(raw_log.level)}: {raw_log.name}: {raw_log.msg}"
        logging.debug(formatted_message)
        self.last_ros_log = RosLog(source_id, formatted_message, logType)
        self.is_new_log = True


class LogManager():
    is_recording = True

    @staticmethod
    async def start_record_logs():
        LogManager.is_recording = True
        log_subscriber = LogSubscriber()
        while LogManager.is_recording:
            try:
                await run_in_threadpool(lambda: rclpy.spin_once(log_subscriber, timeout_sec=4))
                if log_subscriber.is_new_log and log_subscriber.last_ros_log is not None:
                    log = log_subscriber.last_ros_log
                    await send_log(log.message, log.source_id, log.logType)
                    log_subscriber.is_new_log = False
            except Exception as e:
                pass
        log_subscriber.destroy_node()

    @staticmethod
    def stop_record_logs():
        LogManager.is_recording = False
        logging.debug("stopping recording")
