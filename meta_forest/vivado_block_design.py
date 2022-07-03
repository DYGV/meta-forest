import distutils.spawn
import logging
import os
import sys

from . import config
from .helpers import PACKAGE_INSTALLED_DIR, run_sys_cmd


def _configure_params(config_dict):
    params = config.Params()
    params.project = f"{config_dict['project']}_vivado"
    params.ip_directory = config_dict["Vivado"]["ip_directory"]
    params.board_part = config_dict["Vivado"]["board_part"]

    ip_name_list = []
    ip_count_list = []
    for IP in config_dict["IP"].values():
        name = IP["name"]
        count = IP["count"]
        if len(name) > 0 and count > 0:
            ip_name_list.append(name)
            ip_count_list.append(count)
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
    if len(params.ip_name_list) == 0 or len(params.ip_count_list) == 0:
        return -1
    if len(params.ip_name_list) != len(params.ip_count_list):
        return -1
    for name, count in zip(params.ip_name_list, params.ip_count_list):
        args += f" -ip {name} {count}"
    tcl_script = os.path.join(PACKAGE_INSTALLED_DIR, "vivado_block_design.tcl")
    command = (
        f"vivado -nolog -nojournal -mode batch "
        f"-source {tcl_script} -tclargs {args}"
    )
    return command


def generate_block_design(args):
    logger = logging.getLogger("meta-FOrEST")
    config_dict = config.load(args.config)
    params = _configure_params(config_dict)
    if not distutils.spawn.find_executable("vivado"):
        logger.error("Vivado not found. Please setup Vivado")
        sys.exit(1)
    command = _build_command(params)
    if command == -1:
        logger.error("Wrong way to specify IP")
        sys.exit(1)
    logger.info(f"Built the command: {command}")
    logger.info("Making vivado block design")
    run_sys_cmd(command)
