import logging

_logger_handler = logging.StreamHandler()
_logger_handler.setFormatter(
    logging.Formatter("\n[%(levelname)s] [%(name)s]: %(message)s\n")
)
_logger = logging.getLogger("meta-FOrEST")
_logger.setLevel(logging.DEBUG)
_logger.addHandler(_logger_handler)
