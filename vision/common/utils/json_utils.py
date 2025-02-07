"""Module for JSON utility functions."""

import json
from types import SimpleNamespace
from typing import Union

from common.utils.log import get_logger

logger = get_logger(__name__)


def read_json(file_path: str, to_dict: bool = False) -> Union[SimpleNamespace, dict]:
    """Extracts JSON content from a file.

    Args:
        file_path: The file path to the JSON
        to_dict: Flag determining to extract as a dict object

    Returns: The JSON data in SimpleNamespace or dict object format depending on the to_dict flag
    """
    logger.info(f"Extracting JSON from {file_path}")
    with open(file_path, "r") as file:
        data = json.load(file, object_hook=lambda d: d if to_dict else SimpleNamespace(**d))
    return data
