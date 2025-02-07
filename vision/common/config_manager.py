
"""Module for configuration manager."""

from types import SimpleNamespace

from common.utils.json_utils import read_json


class ConfigManager:
    def __init__(self, config_path: str) -> None:
        """Constructor for ConfigManager.

        Args:
            config_path: The file path to extract config from
        """
        self.app_conf: SimpleNamespace = read_json(config_path, False)

    def get_config(self) -> SimpleNamespace:
        """Gets the app configuration

        Returns:
            The configuration as a SimpleNamespace
        """
        return self.app_conf
