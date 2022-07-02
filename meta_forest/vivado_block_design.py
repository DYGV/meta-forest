import distutils.spawn
import logging
import os

from . import config
from .helper import PACKAGE_INSTALLED_DIR, run_sys_cmd

_logger = logging.getLogger("meta-FOrEST")


def configure_params(config_dict):
    params = config.Params()
    params.project = f"{config_dict['project']}_vivado"
    params.ip_directory = config_dict["Vivado"]["ip_directory"]
    params.board_part = config_dict["Vivado"]["board_part"]

    ip_name_list = []
    ip_count_list = []
    for IP in config_dict["IP"].values():
        ip_name_list.append(IP["name"])
        ip_count_list.append(IP["count"])
    params.ip_name_list = ip_name_list
    params.ip_count_list = ip_count_list

    return params


def _build_command(params):
    args = (
        f"-project_name {params.project} "
        f"-device_part {params.board_part} "
        f"-ips_directory {params.ip_directory} "
    )
    args += "-auto_connect -write_bitstream -start_gui"
    for name, count in zip(params.ip_name_list, params.ip_count_list):
        args += f" -ip {name} {count}"
    tcl_script = os.path.join(PACKAGE_INSTALLED_DIR, "create_bd.tcl")
    command = (
        f"vivado -nolog -nojournal -mode batch "
        f"-source {tcl_script} -tclargs {args}"
    )
    return command


def create(params):
    if not distutils.spawn.find_executable("vivado"):
        _logger.error("Vivado not found. Please setup Vivado")
        return
    command = _build_command(params)
    _logger.info(f"Built the command: {command}")
    _logger.info("Making vivado block design")
    run_sys_cmd(command)
