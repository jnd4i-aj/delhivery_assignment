import os
import logging
from os.path import join
from .file_sys_utils import FileSysUtility
import threading
from datetime import date


def get_date():

    curr_date = date.today().strftime("%d-%m-%Y")
    return curr_date


class ServiceLogger:

    _singleton = None

    def __init__(self):

        if ServiceLogger._singleton is not None:
            raise Exception("ServiceLogger is a singleton class")
        else:

            self.agv_loggers = {}
            self.agv_logger_lock = threading.Lock()

            ServiceLogger._singleton = self

    @staticmethod
    def get():
        if ServiceLogger._singleton is None:
            ServiceLogger()
        return ServiceLogger._singleton

    def __get_logging_level(self, level: str):

        log_level = {
            "debug": logging.DEBUG,
            "info": logging.INFO,
            "warn": logging.WARNING,
            "error": logging.ERROR,
            "fatal": logging.CRITICAL,
        }.get(level, logging.INFO)

        return log_level

    def __central_data_log(self, node_name: str, message: str, level: str):

        with self.agv_logger_lock:

            date_str = get_date()
            base_dir = join(FileSysUtility.log_path(), f"{date_str}_all_logs")
            os.makedirs(base_dir, exist_ok=True)

            if node_name not in self.agv_loggers:
                self.agv_loggers[node_name] = self.__create_agv_logger(node_name, base_dir)

            logger = self.agv_loggers[node_name]
            log_level = self.__get_logging_level(level)
            logger.log(log_level, message)

    def __create_agv_logger(self, node_name: str, base_dir: str):

        logger = logging.getLogger(f"{node_name}_logs")
        logger.setLevel(logging.DEBUG)

        log_file = join(base_dir, f"{node_name}.log")
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.DEBUG)

        formatter = logging.Formatter(
            "%(asctime)s [%(levelname)s]: %(message)s", "%Y-%m-%d %H:%M:%S"
        )

        file_handler.setFormatter(formatter)

        logger.addHandler(file_handler)
        logger.propagate = False

        return logger

    def central_data_debug(self, agv_id: str, message: str):
        self.__central_data_log(agv_id, message, "debug")

    def central_data_info(self, agv_id: str, message: str):
        self.__central_data_log(agv_id, message, "info")

    def central_data_warn(self, agv_id: str, message: str):
        self.__central_data_log(agv_id, message, "warn")

    def central_data_error(self, agv_id: str, message: str):
        self.__central_data_log(agv_id, message, "error")

    def central_data_fatal(self, agv_id: str, message: str):
        self.__central_data_log(agv_id, message, "fatal")
