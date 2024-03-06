from ...logs import send_log
import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Log

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
        robot_id = log_msg.name.split("robot")[1]

        # Extract log message
        message = log_msg.msg

        #Extract log name
        name = log_msg.name

        return severity, robot_id, name, message

    def listener_callback(self, msg):
        severity, robot_id,name, message = self.parse_log_message(msg)
        log_message = f"{severity}: {name}: {message}"
        self.messageList.append([log_message, robot_id])
        # self.get_logger().info(log_message)
        

        
        
async def record_logs():
    rclpy.init()
    logSubscriber = LogSubscriber()
    current_index:int = 0
    while(rclpy.ok):
        try:
            rclpy.spin_once(logSubscriber, timeout_sec=2)
            #TODO S'assurer qu'il y a vraiment une valeur de log enregistrée avant de send_log
            await send_log(logSubscriber.messageList[current_index][0],
                           logSubscriber.messageList[current_index][1],)
            current_index += 1
        except KeyboardInterrupt:
            pass

    logSubscriber.destroy_node()
    rclpy.shutdown()
