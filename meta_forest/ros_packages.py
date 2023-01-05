import json
import logging
import os
import re
import shutil

from . import project
from .helpers import TEMPORARY_OUTPUT_DIR, render_to_template, run_sys_cmd


def _build_io_maps(config_dict):
    io_maps = {"map_num": {}, "maps": {}}
    ip_num = 0
    for ip_name, ip_details in config_dict.items():
        ip_num += 1
        ip_count = ip_details[0]["count"]
        ip_signals = ip_details[1]["signals"]
        io_maps["maps"].update({ip_num: {"input": {}, "output": {}}})

        for count in range(ip_count):
            io_maps["map_num"].update({f"{ip_name}_{count}": ip_num})
        for signal in ip_signals:
            if signal["direction"] == "in":
                direction = "input"
            elif signal["direction"] == "out":
                direction = "output"
            io_map = {}

            signal_name = signal["name"]
            io_map["addr"] = signal["address_offset"]
            io_map["protocol"] = signal["protocol"].replace("axi4", "").lower()
            io_map["type"] = signal["type"]
            io_map["n_bits"] = int(signal["data_width"])
            io_map["signed"] = not signal["is_unsigned"]
            io_map["n_elem"] = int(signal["array_size"])
            io_map["arr"] = signal["is_array"]
            io_maps["maps"][ip_num][direction][signal_name] = io_map
    return io_maps


def build_packages_with_colcon(dev_ws, packages_list):
    run_sys_cmd(
        ["colcon build --packages-select " + " ".join(packages_list)],
        cwd=dev_ws,
    )


class MessagePackage:
    def _configure_params(self, project_settings_dict, config_dict, dev_ws):
        params = project.Params()
        params.project = f"{project_settings_dict['project_name']}_interface"
        params.dev_ws = dev_ws
        params.io_maps = _build_io_maps(config_dict)
        return params

    def _get_msg_files(self, params):
        msg_files = []
        for map_num in params.io_maps["maps"].keys():
            msg_files.append(os.path.join("msg", f"FpgaIn{map_num}.msg"))
            msg_files.append(os.path.join("msg", f"FpgaOut{map_num}.msg"))
        return msg_files

    def _make_cmakelists_txt_params(self, params):
        return {
            "project": params.project,
            "msg_files": self._get_msg_files(params),
        }

    def _make_package_xml_params(self, params):
        return {
            "project": params.project,
        }

    def _process_msg_file(self, filename, io_map):
        ros2_type = ""
        for signal_name in io_map.keys():
            is_arr = io_map[signal_name]["arr"]
            signal_type = io_map[signal_name]["type"]
            num_bits = str(io_map[signal_name]["n_bits"])
            signed = io_map[signal_name]["signed"]
            if not signed:
                ros2_type += "u"
            ros2_type += signal_type
            ros2_type += num_bits
            if is_arr:
                n_elem = str(io_map[signal_name]["n_elem"])
                ros2_type += "[" + n_elem + "]"
            ros2_type += f" {signal_name}\n"
        f = open(
            os.path.join(TEMPORARY_OUTPUT_DIR, "%s-int.msg" % filename),
            "w",
        )
        f.write(ros2_type)
        f.close()

    def _create_msg_file(self, params):
        msg_dir = os.path.join(params.dev_ws, "src", params.project, "msg")
        if not os.path.exists(msg_dir):
            os.makedirs(msg_dir)
        for map_num, io_map in params.io_maps["maps"].items():
            fpga_in_msg = f"FpgaIn{map_num}"
            fpga_out_msg = f"FpgaOut{map_num}"
            self._process_msg_file(fpga_in_msg, io_map["input"])
            self._process_msg_file(fpga_out_msg, io_map["output"])

            shutil.copy(
                os.path.join(TEMPORARY_OUTPUT_DIR, f"{fpga_in_msg}-int.msg"),
                os.path.join(msg_dir, fpga_in_msg + ".msg"),
            )
            shutil.copy(
                os.path.join(TEMPORARY_OUTPUT_DIR, f"{fpga_out_msg}-int.msg"),
                os.path.join(msg_dir, fpga_out_msg + ".msg"),
            )

    def _render(self, params):
        if not os.path.exists(TEMPORARY_OUTPUT_DIR):
            os.makedirs(TEMPORARY_OUTPUT_DIR)
        render_params = self._make_package_xml_params(params)
        render_to_template(
            "package-int.xml.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "package-int.xml"),
            render_params,
        )

        render_params = self._make_cmakelists_txt_params(params)
        render_to_template(
            "CMakeLists-int.txt.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "CMakeLists-int.txt"),
            render_params,
        )

    def _create(self, params):
        src_dir = os.path.join(params.dev_ws, "src")

        if not os.path.exists(src_dir):
            os.makedirs(src_dir)

        run_sys_cmd(
            ["ros2 pkg create --build-type ament_cmake " + params.project],
            cwd=src_dir,
        )

        self._render(params)
        self._create_msg_file(params)

        shutil.copy(
            os.path.join(TEMPORARY_OUTPUT_DIR, "CMakeLists-int.txt"),
            os.path.join(params.dev_ws, "src", params.project, "CMakeLists.txt"),
        )
        shutil.copy(
            os.path.join(TEMPORARY_OUTPUT_DIR, "package-int.xml"),
            os.path.join(params.dev_ws, "src", params.project, "package.xml"),
        )


class NodePackage:
    def __init__(self, test_node_enabled):
        self.test_node_enabled = test_node_enabled

    def _configure_params(self, project_settings_dict, config_dict, dev_ws, bitstream):
        params = project.Params()
        params.project = f"{project_settings_dict['project_name']}_fpga_node"
        params.project_interface = f"{project_settings_dict['project_name']}_interface"
        params.dev_ws = dev_ws
        params.bit_file = bitstream
        params.io_maps = _build_io_maps(config_dict)
        params.qos = 10
        params.head_ip_name = next(iter(params.io_maps["map_num"]))
        return params

    def _render(self, params):
        if not os.path.exists(TEMPORARY_OUTPUT_DIR):
            os.makedirs(TEMPORARY_OUTPUT_DIR)

        render_to_template(
            "package-node.xml.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "package-node.xml"),
            {
                "project": params.project,
                "project_interface": params.project_interface,
            },
        )

        render_to_template(
            "setup.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "setup-node.py"),
            {
                "project": params.project,
                "test_enabled": self.test_node_enabled,
            },
        )

        render_to_template(
            "ros_fpga_lib.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "ros_fpga_lib-node.py"),
            {"project": params.project, "bit_file": params.bit_file},
        )

        render_to_template(
            "fpga_node.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "fpga_node-node.py"),
            {
                "project": params.project_interface,
                "qos": params.qos,
                "ip_name": params.head_ip_name,
            },
        )

        render_to_template(
            "fpga_node_launch.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "fpga_node_launch.py"),
            {
                "project": params.project,
                "ip_names": list(params.io_maps["map_num"].keys()),
            },
        )

        if not self.test_node_enabled:
            return

        render_to_template(
            "talker.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "talker-node.py"),
            {
                "project": params.project_interface,
                "qos": params.qos,
                "ip_name": params.head_ip_name,
            },
        )

        render_to_template(
            "talker_launch.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "talker_launch.py"),
            {
                "project": params.project,
                "ip_map_nums": params.io_maps["map_num"],
            },
        )

        render_to_template(
            "listener.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "listener-node.py"),
            {
                "project": params.project_interface,
                "qos": params.qos,
                "ip_name": params.head_ip_name,
            },
        )

        render_to_template(
            "listener_launch.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "listener_launch.py"),
            {
                "project": params.project,
                "ip_map_nums": params.io_maps["map_num"],
            },
        )

    def _create(self, params):
        src_dir = os.path.join(params.dev_ws, "src")

        if not os.path.exists(src_dir):
            os.makedirs(src_dir)

        run_sys_cmd(
            ["ros2 pkg create --build-type ament_python " + params.project],
            cwd=src_dir,
        )

        self._render(params)
        io_maps_json = open(
            os.path.join(
                params.dev_ws,
                "src",
                params.project,
                params.project,
                "io_maps.json",
            ),
            "w",
        )
        io_maps_json.write(json.dumps(params.io_maps))
        io_maps_json.close()

        # Copy modified node package.xml
        shutil.copy(
            os.path.join(TEMPORARY_OUTPUT_DIR, "package-node.xml"),
            os.path.join(params.dev_ws, "src", params.project, "package.xml"),
        )
        # Copy modified node setup.py
        shutil.copy(
            os.path.join(TEMPORARY_OUTPUT_DIR, "setup-node.py"),
            os.path.join(params.dev_ws, "src", params.project, "setup.py"),
        )
        # Copy modified node fpga_node.py
        shutil.copy(
            os.path.join(TEMPORARY_OUTPUT_DIR, "fpga_node-node.py"),
            os.path.join(
                params.dev_ws,
                "src",
                params.project,
                params.project,
                "fpga_node.py",
            ),
        )
        # Copy modified node ros_fpga_lib.py
        shutil.copy(
            os.path.join(TEMPORARY_OUTPUT_DIR, "ros_fpga_lib-node.py"),
            os.path.join(
                params.dev_ws,
                "src",
                params.project,
                params.project,
                "ros_fpga_lib.py",
            ),
        )
        # Copy modified node fpga_node_launch.py
        launch_dir = os.path.join(params.dev_ws, "src", params.project, "launch")
        if not os.path.exists(launch_dir):
            os.makedirs(launch_dir)
        launch_file_name = "fpga_node_launch.py"
        shutil.copy(
            os.path.join(TEMPORARY_OUTPUT_DIR, launch_file_name),
            os.path.join(launch_dir, launch_file_name),
        )

        # If running in test generation mode, copy the test nodes as well
        if self.test_node_enabled:
            shutil.copy(
                os.path.join(TEMPORARY_OUTPUT_DIR, "talker-node.py"),
                os.path.join(
                    params.dev_ws,
                    "src",
                    params.project,
                    params.project,
                    "talker.py",
                ),
            )
            shutil.copy(
                os.path.join(TEMPORARY_OUTPUT_DIR, "listener-node.py"),
                os.path.join(
                    params.dev_ws,
                    "src",
                    params.project,
                    params.project,
                    "listener.py",
                ),
            )
            shutil.copy(
                os.path.join(TEMPORARY_OUTPUT_DIR, "talker_launch.py"),
                os.path.join(launch_dir, "talker_launch.py"),
            )
            shutil.copy(
                os.path.join(TEMPORARY_OUTPUT_DIR, "listener_launch.py"),
                os.path.join(launch_dir, "listener_launch.py"),
            )


def generate_packages(args):
    logger = logging.getLogger("meta-FOrEST")
    config_dict = dict(project.load_config_file("."))
    project_settings_dict = dict(project.load_project_file("."))

    message_package = MessagePackage()
    node_package = NodePackage(args.test)

    message_package_params = message_package._configure_params(
        project_settings_dict, config_dict, args.ros2_ws
    )
    node_package_params = node_package._configure_params(
        project_settings_dict, config_dict, args.ros2_ws, args.bitstream
    )

    logger.info("Generating the ROS2 package for the FPGA node messages")
    message_package._create(message_package_params)

    logger.info("Generating the ROS2 package for the FPGA node")
    node_package._create(node_package_params)

    logger.info("Building the ROS2 packages")
    build_packages_with_colcon(
        args.ros2_ws,
        [message_package_params.project, node_package_params.project],
    )
