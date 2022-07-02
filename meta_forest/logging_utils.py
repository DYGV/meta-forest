import enum
import logging


class Level(enum.Enum):
    debug = logging.DEBUG
    info = logging.INFO
    warning = logging.WARNING
    error = logging.ERROR


def setup_logger(log_level):
    level_value = Level[log_level].value
    logger_handler = logging.StreamHandler()
    logger_handler.setFormatter(
        logging.Formatter("\n[%(levelname)s] [%(name)s]: %(message)s\n")
    )
    logger = logging.getLogger("meta-FOrEST")
    logger.setLevel(level_value)
    logger.addHandler(logger_handler)
    logger.info(
        f"logging level was set to {logging.getLevelName(level_value)}"
    )
