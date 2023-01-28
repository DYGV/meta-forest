import enum
import logging
import sys


class Level(enum.Enum):
    """Logger level definition in the logging module"""

    debug = logging.DEBUG
    info = logging.INFO
    warning = logging.WARNING
    error = logging.ERROR


def _setup_logger(log_level):
    """Internal logger setting

    Parameters
    ----------
    log_level: str
        Logger levels such as "info" and "debug"

    Returns
    -------
    -1 or 0: int
        Success or Failure

    """
    logger = logging.getLogger("meta-FOrEST")
    logger_handler = logging.StreamHandler()
    logger_handler.setFormatter(
        logging.Formatter("\n[%(levelname)s] [%(name)s]: %(message)s\n")
    )
    logger.addHandler(logger_handler)

    if log_level.lower() not in ["debug", "info", "warning", "error"]:
        logger.setLevel(Level.error.value)
        logger.error("Incorrect value for logging level")
        return -1

    level_value = Level[log_level].value
    logger.setLevel(level_value)
    logger.info(f"logging level was set to {logging.getLevelName(level_value)}")
    return 0


def setup_logger(args):
    """logger setting

    Parameters
    ----------
    args: argparse.Namespace
    """
    if _setup_logger(args.log_level) == -1:
        sys.exit(1)
