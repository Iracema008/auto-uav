"""Defines common config for logging.

Typical usage:

    logger = get_logger(__name__)
    logger.info('Log info')
"""

import logging
from rich.logging import RichHandler

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(" "message)s",
    datefmt="[%Y-%m-%d %H:%M:%S]",
    handlers=[RichHandler()],
)


def get_logger(name: str) -> logging.Logger:
    """Get logger for a module.

    Args:
        name: Name of a module

    Returns: logging.Logger object for the module
    """

    logger = logging.getLogger(name)
    return logger
