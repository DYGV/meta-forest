import tomli

from .logging_utils import _logger


class Params(object):
    def __init__(self):
        _logger.debug("Data for required parameters is being configured.")

    def __setattr__(self, name, value):
        object.__setattr__(self, name, value)
        _logger.debug(f"{name}: {value}")


def get_render_params(args):
    if not args.number_of_IPs > 0:
        _logger.error(
            "Specify the number of IP cores must be greater than or equal to 1"
        )
        return
    render_params = {
        "number_of_IPs": args.number_of_IPs,
    }
    return render_params


def parse(file_path):
    toml_file = open(file_path, "rb")
    toml_dict = None
    try:
        toml_dict = tomli.load(toml_file)
    except tomli.TOMLDecodeError as error:
        _logger.error(error)
    toml_file.close()
    return toml_dict
