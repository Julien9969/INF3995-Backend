import asyncio
from collections import namedtuple
import re
from typing import List
from backend_server.api.mission.mission_base import MissionState, MissionData
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
        self.isFinished = False
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
        logging.debug("listener called")
        
        # Check if the message indicates the end of the process
        if raw_log.msg == 'Incoming request, command: stop, current state: State.ON': 
            self.isFinished = True
        
        source_id : int = self.get_robot_id(raw_log)
        logType = self.get_event_type(raw_log)
        formatted_message = f"{self.get_severity(raw_log)}: {raw_log.name}: {raw_log.msg}"
        
        self.lastRosLog = RosLog(source_id, formatted_message, logType)
        self.isNewLog = True
        

        
async def start_record_logs():
    logSubscriber = LogSubscriber()
    while(rclpy.ok()):
        try:
            rclpy.spin_once(logSubscriber, timeout_sec=2)
            if logSubscriber.isNewLog and logSubscriber.lastRosLog is not None:
                log = logSubscriber.lastRosLog
                await send_log(log.message, log.source_id, log.logType)
                logSubscriber.isNewLog = False
            await asyncio.sleep(2)
            if (logSubscriber.isFinished): break
        except KeyboardInterrupt:
            pass

    if(rclpy.ok()): 
        logging.debug("not shut down yet..............................")
    logSubscriber.destroy_node()
    rclpy.shutdown()
