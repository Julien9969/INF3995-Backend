import asyncio
from collections import namedtuple
import re
from typing import List
from backend_server.api.mission.mission_base import MissionState, MissionData
from fastapi.concurrency import run_in_threadpool
from ..logs import LogType, send_log
import logging
import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Log

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
        self.lastRosLog : RosLog = None
        self.isNewLog = False

    def get_robot_id(self,log_msg):

        pattern = r"robot(\d+)"
        match = re.search(pattern, log_msg.name)
        if match:
            robot_id = match.group(1)
        else:
            robot_id = 0
        return robot_id

    def get_severity(self,log_msg):
        severity_levels = {
            10: 'DEBUG',
            20: 'INFO',
            30: 'WARN',
            40: 'ERROR',
            50: 'FATAL'
        }
        severity = severity_levels.get(log_msg.level, 'UNKNOWN')
        return severity
    
    def get_event_type(self,log_msg):
        if "command" in log_msg.msg:
            event_type = LogType.COMMAND
        else:
            event_type = LogType.LOG
        return event_type
        

    def listener_callback(self, raw_log):
        source_id : int = self.get_robot_id(raw_log)
        logType = self.get_event_type(raw_log)
        formatted_message = f"{self.get_severity(raw_log)}: {raw_log.name}: {raw_log.msg}"
        logging.debug(formatted_message)
        self.lastRosLog = RosLog(source_id, formatted_message, logType)
        self.isNewLog = True
        
class LogManager():
    isRecording = True
    @staticmethod
    async def start_record_logs():
        LogManager.isRecording = True
        logSubscriber = LogSubscriber()
        while LogManager.isRecording:
            try:
                await run_in_threadpool(lambda:rclpy.spin_once(logSubscriber, timeout_sec=4))
                # rclpy.spin_once(logSubscriber, timeout_sec=4)
                if logSubscriber.isNewLog and logSubscriber.lastRosLog is not None:
                    log = logSubscriber.lastRosLog
                    await send_log(log.message, log.source_id, log.logType)
                    logSubscriber.isNewLog = False
            except Exception as e:
                pass
        logSubscriber.destroy_node()

    @staticmethod
    def stop_record_logs():
        LogManager.isRecording = False
        logging.debug("stopping recording")
    
