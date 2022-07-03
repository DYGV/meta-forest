import logging
import os
import sys

import tomli

from .helpers import render_to_template

_logger = logging.getLogger("meta-FOrEST")


class Params(object):
    def __init__(self):
        _logger.debug("Data for required parameters is being configured.")

    def __setattr__(self, name, value):
        object.__setattr__(self, name, value)
        _logger.debug(f"{name}: {value}")


def _get_render_params(args):
    if not args.number_of_IPs > 0:
        return -1
    render_params = {
        "number_of_IPs": args.number_of_IPs,
    }
    return render_params


def _write_to_config_template(output_file_name, params, is_force):
    if os.path.isfile(output_file_name) and not is_force:
        return -1
    render_to_template("config.toml.jinja2", output_file_name, params)


def load(file_path):
    toml_file = open(file_path, "rb")
    toml_dict = None
    try:
        toml_dict = tomli.load(toml_file)
    except Exception as error:
        _logger.error(error)
    toml_file.close()
    return toml_dict


def generate_config(args):
    render_params = _get_render_params(args)
    if render_params == -1:
        _logger.error(
            "Specify the number of IP cores must be greater than or equal to 1"
        )
        sys.exit(1)
    _logger.info("Config file generation mode")
    if _write_to_config_template(args.config, render_params, args.force) == -1:
        _logger.error(
            "A config file already exists. "
            "Remove a file that already exists or use --force"
        )
        sys.exit(1)
