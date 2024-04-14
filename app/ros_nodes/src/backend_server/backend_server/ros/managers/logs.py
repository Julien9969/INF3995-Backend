import logging
from backend_server.models.mission import Mission
from backend_server.classes.common import LogType
import rclpy
from fastapi.concurrency import run_in_threadpool
from backend_server.websocket.emitter import send_log
from backend_server.ros.subscribers.logs import LogSubscriber
from backend_server.classes.constants import RCL_TIMEOUT


class LogManager:
    is_recording = True  # TODO: shouldn't this be False?

    @staticmethod
    async def start_record_logs():
        logging.debug("starting recording logs")
        LogManager.is_recording = True
        log_subscriber = LogSubscriber()
        while LogManager.is_recording and rclpy.ok():
            try:
                await run_in_threadpool(lambda: rclpy.spin_once(log_subscriber, timeout_sec=RCL_TIMEOUT))
                if log_subscriber.is_new_log and log_subscriber.last_ros_log is not None:
                    log = log_subscriber.last_ros_log
                    if log.logType == LogType.BATTERY:
                        battery_level = log.message.split(":")[1].strip().replace("%", "")
                        Mission().check_battery(battery_level, log.source_id)
                    else:
                        await send_log(log.message, log.source_id, log.logType)
                    log_subscriber.is_new_log = False
            except Exception as err:
                if str(err) != "generator already executing":
                    logging.debug(f"Exception in Map manager: {err}")
        log_subscriber.destroy_node()

    @staticmethod
    def stop_record_logs():
        LogManager.is_recording = False
        logging.debug("stopping recording")
