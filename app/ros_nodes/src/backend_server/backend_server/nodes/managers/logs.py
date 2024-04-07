import logging
from backend_server.common import LogType
from backend_server.logic.robots import RobotsData
import rclpy
from fastapi.concurrency import run_in_threadpool
from backend_server.websocket.emitter import send_log
from backend_server.nodes.subscribers.logs import LogSubscriber
from backend_server.constants import RCL_TIMEOUT


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
                        RobotsData().update_battery(log.message, log.source_id)
                    else:
                        await send_log(log.message, log.source_id, log.logType)
                    log_subscriber.is_new_log = False
            except Exception as err:
                logging.debug(f"Exception in Log manager: {err}")
                pass
        log_subscriber.destroy_node()

    @staticmethod
    def stop_record_logs():
        LogManager.is_recording = False
        logging.debug("stopping recording")
