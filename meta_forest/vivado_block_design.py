import distutils.spawn
import logging
import os
import sys

from . import project
from .helpers import PACKAGE_INSTALLED_DIR, run_sys_cmd


def _configure_params(project_settings_dict, config_dict):
    params = project.Params()
    params.project = "vivado"
    params.ip_directory = os.path.join(project_settings_dict["project_path"], "ip")
    params.board_part = project_settings_dict["target_part"]

    vivado_block_design = project_settings_dict["vivado_block_design"]
    params.auto_start_gui = vivado_block_design["auto_start_gui"]
    params.auto_connect_block_design = vivado_block_design["auto_connect_block_design"]
    params.to_step_write_bitstream = vivado_block_design["to_step_write_bitstream"]

    ip_name_list = []
    ip_count_list = []
    for k, v in config_dict.items():
        name = k
        count = v[0]["count"]
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
        f"-start_gui {params.auto_start_gui} "
        f"-auto_connect {params.auto_connect_block_design} "
        f"-write_bitstream {params.to_step_write_bitstream} "
    )
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


def package_rtl(solution_path):
    tcl_script = os.path.join(PACKAGE_INSTALLED_DIR, "package_rtl.tcl")
    return f"vitis_hls {tcl_script} {solution_path}"


def generate_block_design(args):
    logger = logging.getLogger("meta-FOrEST")
    project_settings_dict = dict(project.load_project_file("."))
    config_dict = dict(project.load_config_file("."))

    for solution_path in project_settings_dict["solution_path"]:
        command = package_rtl(list(solution_path.values())[0])
        logger.info(f"Built the command: {command}")
        if not distutils.spawn.find_executable("vitis_hls"):
            logger.error("Vitis HLS not found. Please setup Vitis HLS")
            sys.exit(1)
        run_sys_cmd(command)

    params = _configure_params(project_settings_dict, config_dict)
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
