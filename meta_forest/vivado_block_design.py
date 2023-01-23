import distutils.spawn
import logging
import os
import sys

from . import project
from .helpers import PACKAGE_INSTALLED_DIR, run_sys_cmd


def _configure_params(args):
    params = project.Params()
    params.project = "vivado"
    params.ip_directory = args.ip_directory
    params.board_part = args.target_part

    params.auto_start_gui = 0
    params.auto_connect_block_design = 0
    params.to_step_write_bitstream = 0
    if args.step_to == "write_bitstream":
        params.auto_connect_block_design = 1
        params.to_step_write_bitstream = 1
    elif args.step_to == "connect":
        params.auto_connect_block_design = 1

    ip_name_list = []
    ip_count_list = []
    for i in range(len(args.IP)):
        name = args.IP[i]
        count = args.count[i]
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
        f"-start_gui {params.auto_start_gui} "
        f"-auto_connect {params.auto_connect_block_design} "
        f"-write_bitstream {params.to_step_write_bitstream} "
    )
    for ip_dir in params.ip_directory:
        args += f"-ip_directories {ip_dir} "
    if len(params.ip_name_list) == 0 or len(params.ip_count_list) == 0:
        return -1
    if len(params.ip_name_list) != len(params.ip_count_list):
        return -1
    for name, count in zip(params.ip_name_list, params.ip_count_list):
        args += f" -ip {name} {count}"
    tcl_script = os.path.join(PACKAGE_INSTALLED_DIR, "vivado_block_design.tcl")
    command = (
        f"vivado -nolog -nojournal -mode batch " f"-source {tcl_script} -tclargs {args}"
    )
    return command


def generate_block_design(args):
    logger = logging.getLogger("meta-FOrEST")
    params = _configure_params(args)
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
