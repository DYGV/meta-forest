import logging
import os

import tomli

from .helper import render_to_template

_logger = logging.getLogger("meta-FOrEST")


class Params(object):
    def __init__(self):
        _logger.debug("Data for required parameters is being configured.")

    def __setattr__(self, name, value):
        object.__setattr__(self, name, value)
        _logger.debug(f"{name}: {value}")


def _get_render_params(args):
    if not args.number_of_IPs > 0:
        _logger.error(
            "Specify the number of IP cores must be greater than or equal to 1"
        )
        return
    render_params = {
        "number_of_IPs": args.number_of_IPs,
    }
    return render_params


def _write_to_config_template(output_file_name, params, is_force):
    _logger.info("Config file generation mode")
    if os.path.isfile(output_file_name) and not is_force:
        _logger.error(
            "A config file already exists. "
            "Remove a file that already exists or use --force"
        )
        return
    if params is not None:
        render_to_template("config.toml.jinja2", output_file_name, params)


def load(file_path):
    toml_file = open(file_path, "rb")
    toml_dict = None
    try:
        toml_dict = tomli.load(toml_file)
    except tomli.TOMLDecodeError as error:
        _logger.error(error)
    toml_file.close()
    return toml_dict


def generate_config(args):
    render_params = _get_render_params(args)
    _write_to_config_template(args.config, render_params, args.force)
