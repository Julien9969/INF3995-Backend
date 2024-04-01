import logging
from fastapi.concurrency import run_in_threadpool
from backend_server.websocket.emitter import send_log
from backend_server.nodes.subscribers.logs import LogSubscriber
from backend_server.constants import RCL_TIMEOUT


class LogManager:
    is_recording = True

    @staticmethod
    async def start_record_logs():
        LogManager.is_recording = True
        log_subscriber = LogSubscriber()
        while LogManager.is_recording:
            try:
                await run_in_threadpool(lambda: rclpy.spin_once(log_subscriber, timeout_sec=RCL_TIMEOUT))
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
