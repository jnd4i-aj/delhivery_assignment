import os
from os.path import join, expanduser


class FileSysUtility:
    username = os.environ.get('USER', 'default_user')

    @staticmethod
    def home_dir_path():
        return expanduser("~")

    @staticmethod
    def trajectory_map_path():
        return join(FileSysUtility.home_dir_path(), "ros2_ws", "src", "delhivery_assignment", "config", "trajectory_map.yaml")

    @staticmethod
    def log_path():
        return join(FileSysUtility.home_dir_path(), "service_logs")

    @staticmethod
    def log_code_file_path():
        return join(FileSysUtility.home_dir_path(), "ros2_ws", "src", "delhivery_assignment", "config", "log_codes.yaml")
