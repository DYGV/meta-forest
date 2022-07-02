import enum
import logging


class Level(enum.Enum):
    debug = logging.DEBUG
    info = logging.INFO
    warning = logging.WARNING
    error = logging.ERROR


def setup_logger(log_level):
    level_value = Level[log_level].value
    _logger_handler = logging.StreamHandler()
    _logger_handler.setFormatter(
        logging.Formatter("\n[%(levelname)s] [%(name)s]: %(message)s\n")
    )
    _logger = logging.getLogger("meta-FOrEST")
    _logger.setLevel(level_value)
    _logger.addHandler(_logger_handler)
    _logger.info(
        f"logging level was set to {logging.getLevelName(level_value)}"
    )
