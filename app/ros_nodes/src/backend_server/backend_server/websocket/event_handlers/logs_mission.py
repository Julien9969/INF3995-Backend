import asyncio
from backend_server.api.mission.mission_base import MissionState, MissionData
from ..logs import send_log
import logging
import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Log

logging.basicConfig(level=logging.DEBUG)
class LogSubscriber(Node):

    def __init__(self):
        super().__init__('logs_subscriber')
        self.subscription = self.create_subscription(
            Log,
            'rosout',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.messageList = []

    def parse_log_message(self, log_msg):
        # Extract severity level
        severity_levels = {
            10: 'DEBUG',
            20: 'INFO',
            30: 'WARN',
            40: 'ERROR',
            50: 'FATAL'
        }
        severity = severity_levels.get(log_msg.level, 'UNKNOWN')

        # Extract robot ID from logger name
        if log_msg.name.split("robot") is not None and 1 < len(log_msg.name.split("robot")):
            robot_id = log_msg.name.split("robot")[1][0] 
        else:
            robot_id = 2

        # Extract log message
        message = log_msg.data

        #Extract log name
        name = log_msg.name

        return severity, robot_id, name, message

    def listener_callback(self, msg):
        logging.debug("This is a debug message.")
        severity, robot_id,name, message = self.parse_log_message(msg)
        log_message = f"{severity}: {name}: {message}"
        # self.get_logger().info(robot_id)
        self.messageList.append([log_message, robot_id])
        # self.get_logger().info(log_message)
        

        
        
async def start_record_logs():
    rclpy.init()
    current_index:int = 0
    while(MissionData().get_mission_state() == MissionState.ONGOING):
        try:
            await asyncio.sleep(2)
            logSubscriber = LogSubscriber()
            rclpy.spin_once(logSubscriber, timeout_sec=2)
            # logging.debug("This is a debug message..............................")
            #TODO S'assurer qu'il y a vraiment une valeur de log enregistrée avant de send_log
            if current_index < len(logSubscriber.messageList) and logSubscriber.messageList[current_index] is not None:
                await send_log(logSubscriber.messageList[current_index][0],
                           logSubscriber.messageList[current_index][1])
                current_index += 1
            logSubscriber.destroy_node()
        except KeyboardInterrupt:
            pass

    logSubscriber.destroy_node()
    rclpy.shutdown()
