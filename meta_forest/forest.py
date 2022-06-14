import argparse
import json
import logging
import os
import re
import subprocess
import sys

from jinja2 import Environment, FileSystemLoader

PACKAGE_INSTALLED_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPORARY_DIR = os.path.join(PACKAGE_INSTALLED_DIR, "temp")

_logger_handler = logging.StreamHandler()
_logger_handler.setFormatter(
    logging.Formatter("\n[%(levelname)s] [%(name)s]: %(message)s\n")
)
_logger = logging.getLogger("meta-FOrEST")
_logger.setLevel(logging.DEBUG)
_logger.addHandler(_logger_handler)


def run_sys_cmd(cmd, cwd=None):
    # Helper function for running Linux commands

    subprocess.run(
        cmd, cwd=cwd, stderr=sys.stderr, stdout=sys.stdout, shell=True
    )


def process_msg_file(filename, io_map):
    # Generates the appropriate input and output ROS2 message files
    # given the user logic input and output signals

    f = open(
        os.path.join(TEMPORARY_DIR, "%s-int.msg" % filename),
        "w",
    )
    for signal_name in io_map.keys():
        is_arr = io_map[signal_name]["arr"]
        signal_type = io_map[signal_name]["type"]
        layout_name = io_map[signal_name]["layout_name"]
        num_bits = str(io_map[signal_name]["n_bits"])
        signed = io_map[signal_name]["signed"]
        ros2_type = ""
        if len(layout_name) > 0:
            ros2_type = "std_msgs/MultiArrayLayout %s\n" % layout_name
        if not signed:
            ros2_type = "u"
        ros2_type += signal_type
        ros2_type += num_bits
        if is_arr:
            n_elem = str(io_map[signal_name]["n_elem"])
            ros2_type += "[" + n_elem + "]"
        f.write(ros2_type + " " + signal_name + "\n")
    f.close()


def check_in_map_validity(in_map):
    # Check the input map generated after parsing the config file, and
    # verify its validity

    n_axis = 0
    for input_signal in in_map.keys():
        if not (
            set(in_map[input_signal].keys())
            == set(
                [
                    "protocol",
                    "type",
                    "layout_name",
                    "n_bits",
                    "signed",
                    "arr",
                    "n_elem",
                    "addr",
                ]
            )
        ):
            _logger.error(
                "Input map is not valid. Make sure the input"
                "definition is correct. Error at signal {}".format(
                    input_signal
                )
            )
            sys.exit(1)
        if (
            not in_map[input_signal]["arr"]
            and len(in_map[input_signal]["layout_name"]) > 0
        ):
            _logger.error(
                "std_msgs/MultiArrayLayout "
                "can be used when the type is an array."
            )
            sys.exit(1)
        if in_map[input_signal]["protocol"] == "stream":
            n_axis += 1
        if n_axis > 1:
            _logger.error(
                "Only 1 input signal can use " "the AXI-Stream protocol"
            )
            sys.exit(1)


def check_out_map_validity(out_map):
    # Check the output map generated after parsing the config file, and
    # verify its validity

    n_axis = 0
    for output_signal in out_map.keys():
        if not (
            set(out_map[output_signal].keys())
            == set(
                [
                    "protocol",
                    "type",
                    "layout_name",
                    "n_bits",
                    "signed",
                    "arr",
                    "n_elem",
                    "addr",
                ]
            )
        ):
            _logger.error(
                "Output map is not valid. "
                "Make sure the output definition is correct. "
                "Error at signal {}".format(output_signal)
            )
            sys.exit(1)

        if not out_map[output_signal]["arr"]:
            _logger.error(
                "All output signals must be an array types "
                "at signal {}".format(output_signal)
            )
            sys.exit(1)
        if out_map[output_signal]["protocol"] == "stream":
            n_axis += 1
        if n_axis > 1:
            _logger.error(
                "Only 1 output signal can use the AXI-Stream protocol"
            )
            sys.exit(1)


# Rendering functions


def render_config_file(env, n_config_ips, n_config_inputs, n_config_outputs):
    # Render config.forest template when running in genconfig mode

    config_file = env.get_template("config.forest.jinja2")
    f = open("config.forest", "w")
    f.write(
        config_file.render(
            n_ips=n_config_ips, n_in=n_config_inputs, n_out=n_config_outputs
        )
    )
    f.close()


def render_int_package_xml(env, prj, depend_std_msgs):
    # Render package.xml for the interface package

    package_xml = env.get_template("package-int.xml.jinja2")
    f = open(os.path.join(TEMPORARY_DIR, "package-int.xml"), "w")
    f.write(package_xml.render(prj_name=prj, depend_std_msgs=depend_std_msgs))
    f.close()


def render_int_cmakelists_txt(env, prj, msg_files, depend_std_msgs):
    # Render CMakeLists.txt for the interface package

    cmake_txt = env.get_template("CMakeLists-int.txt.jinja2")
    f = open(
        os.path.join(TEMPORARY_DIR, "CMakeLists-int.txt"),
        "w",
    )
    f.write(
        cmake_txt.render(
            prj_name=prj, msg_files=msg_files, depend_std_msgs=depend_std_msgs
        )
    )
    f.close()


def render_node_package_xml(env, prj):
    # Render package.xml for the node package

    package_xml = env.get_template("package-node.xml.jinja2")
    f = open(os.path.join(TEMPORARY_DIR, "package-node.xml"), "w")
    f.write(package_xml.render(prj_name=prj))
    f.close()


def render_node_setup_py(env, prj, test_enabled):
    # Render setup.py for the node package

    setup_py = env.get_template("setup.py.jinja2")
    f = open(os.path.join(TEMPORARY_DIR, "setup-node.py"), "w")
    f.write(setup_py.render(prj_name=prj, test_enabled=test_enabled))
    f.close()


def render_node_ros_fpga_lib_py(env, prj, bit_file):
    # Render ros_fpga_lib.py for the node package

    ros_fpga_lib_py = env.get_template("ros_fpga_lib.py.jinja2")
    f = open(
        os.path.join(TEMPORARY_DIR, "ros_fpga_lib-node.py"),
        "w",
    )
    f.write(ros_fpga_lib_py.render(prj_name=prj, bit_file=bit_file))
    f.close()


def render_node_fpga_node_py(env, prj, qos, head_ip_name):
    # Render fpga_node.py for the node package

    fpga_node_py = env.get_template("fpga_node.py.jinja2")
    f = open(os.path.join(TEMPORARY_DIR, "fpga_node-node.py"), "w")
    f.write(fpga_node_py.render(qos=qos, prj_name=prj, ip_name=head_ip_name))
    f.close()


def render_node_fpga_launch(env, prj, ip_names):
    # Render fpga_node_launch.py for the node package

    launch_file_name = "fpga_node_launch.py"
    fpga_node_launch_py = env.get_template(launch_file_name + ".jinja2")
    f = open(
        os.path.join(TEMPORARY_DIR, "fpga_node_launch.py"),
        "w",
    )
    f.write(fpga_node_launch_py.render(prj_name=prj, ip_names=ip_names))
    f.close()


def render_test_talker(env, prj, qos, ip_name):
    # Render the message generation file for the node package

    talker_py = env.get_template("talker.py.jinja2")
    f = open(os.path.join(TEMPORARY_DIR, "talker-node.py"), "w")
    f.write(talker_py.render(prj_name=prj, qos=qos, ip_name=ip_name))
    f.close()


def render_test_talker_launch(env, prj, ip_map_nums):
    # Render talker_launch.py for the node package

    launch_file_name = "talker_launch.py"
    talker_launch_py = env.get_template(launch_file_name + ".jinja2")
    f = open(os.path.join(TEMPORARY_DIR, "talker_launch.py"), "w")
    f.write(talker_launch_py.render(prj_name=prj, ip_map_nums=ip_map_nums))
    f.close()


def render_test_listener(env, prj, qos, ip_name):
    # Render the message reader file for the node package

    listener_py = env.get_template("listener.py.jinja2")
    f = open(os.path.join(TEMPORARY_DIR, "listener-node.py"), "w")
    f.write(listener_py.render(prj_name=prj, qos=qos, ip_name=ip_name))
    f.close()


def render_test_listener_launch(env, prj, ip_map_nums):
    # Render listener_launch.py for the node package

    launch_file_name = "listener_launch.py"
    listener_launch_py = env.get_template(launch_file_name + ".jinja2")
    f = open(
        os.path.join(TEMPORARY_DIR, "listener_launch.py"),
        "w",
    )
    f.write(listener_launch_py.render(prj_name=prj, ip_map_nums=ip_map_nums))
    f.close()


# Package generation functions


def create_msg_pkg(dev_ws, prj):
    # Creates interface package

    _logger.info("Generating the ROS2 package for the FPGA node messages")

    # Create ROS2 package for the ROS FPGA messages (interface)
    pkg_name = prj + "_interface"
    run_sys_cmd(
        ["ros2 pkg create --build-type ament_cmake " + pkg_name],
        cwd=os.path.join(dev_ws, "src"),
    )


def create_msg_file(dev_ws, prj, map_num, in_map, out_map):
    pkg_name = prj + "_interface"
    # Create the FPGA message files
    fpga_in_msg = "FpgaIn" + str(map_num)
    fpga_out_msg = "FpgaOut" + str(map_num)
    process_msg_file(fpga_in_msg, in_map)
    process_msg_file(fpga_out_msg, out_map)

    run_sys_cmd(["mkdir -p msg"], cwd=os.path.join(dev_ws, "src", pkg_name))
    run_sys_cmd(
        "cp {} {}".format(
            os.path.join(TEMPORARY_DIR, "%s-int.msg" % fpga_in_msg),
            os.path.join(dev_ws, "src", pkg_name, "msg", fpga_in_msg + ".msg"),
        )
    )
    run_sys_cmd(
        "cp {} {}".format(
            os.path.join(TEMPORARY_DIR, "%s-int.msg" % fpga_out_msg),
            os.path.join(
                dev_ws, "src", pkg_name, "msg", fpga_out_msg + ".msg"
            ),
        )
    )


def build_msg_pkg(dev_ws, prj):
    # Build message package

    pkg_name = prj + "_interface"
    # Copy modified interface CMakeLists.txt
    run_sys_cmd(
        "cp {} {}".format(
            os.path.join(TEMPORARY_DIR, "CMakeLists-int.txt"),
            os.path.join(dev_ws, "src", pkg_name, "CMakeLists.txt"),
        )
    )
    # Copy modified interface package.xml
    run_sys_cmd(
        "cp {} {}".format(
            os.path.join(TEMPORARY_DIR, "package-int.xml"),
            os.path.join(dev_ws, "src", pkg_name, "package.xml"),
        )
    )
    _logger.info("Building the FPGA ROS2 node messages package")
    run_sys_cmd(["colcon build --packages-select " + pkg_name], cwd=dev_ws)


def create_fpga_node_pkg(dev_ws, prj, test_enabled, io_maps):
    # Creates and builds the node package

    # Create ROS2 package for the ROS FPGA node
    _logger.info("Generating the ROS2 package for the FPGA node")
    pkg_name = prj + "_fpga_node"
    run_sys_cmd(
        ["ros2 pkg create --build-type ament_python " + pkg_name],
        cwd=os.path.join(dev_ws, "src"),
    )
    # Copy modified node package.xml
    run_sys_cmd(
        "cp {} {}".format(
            os.path.join(TEMPORARY_DIR, "package-node.xml"),
            os.path.join(dev_ws, "src", pkg_name, "package.xml"),
        )
    )
    # Copy modified node setup.py
    run_sys_cmd(
        "cp {} {}".format(
            os.path.join(TEMPORARY_DIR, "setup-node.py"),
            os.path.join(dev_ws, "src", pkg_name, "setup.py"),
        )
    )
    # Copy modified node fpga_node.py
    run_sys_cmd(
        "cp {} {}".format(
            os.path.join(TEMPORARY_DIR, "fpga_node-node.py"),
            os.path.join(dev_ws, "src", pkg_name, pkg_name, "fpga_node.py"),
        )
    )
    # Copy modified node ros_fpga_lib.py
    run_sys_cmd(
        "cp {} {}".format(
            os.path.join(TEMPORARY_DIR, "ros_fpga_lib-node.py"),
            os.path.join(dev_ws, "src", pkg_name, pkg_name, "ros_fpga_lib.py"),
        )
    )
    # Copy modified node fpga_node_launch .py
    run_sys_cmd(["mkdir -p launch"], cwd=os.path.join(dev_ws, "src", pkg_name))
    launch_file_name = "fpga_node_launch.py"
    run_sys_cmd(
        "cp {} {}".format(
            os.path.join(TEMPORARY_DIR, launch_file_name),
            os.path.join(dev_ws, "src", pkg_name, "launch", launch_file_name),
        )
    )

    # If running in test generation mode, copy the test nodes as well
    if test_enabled:
        run_sys_cmd(
            "cp {} {}".format(
                os.path.join(TEMPORARY_DIR, "talker-node.py"),
                os.path.join(dev_ws, "src", pkg_name, pkg_name, "talker.py"),
            )
        )
        run_sys_cmd(
            "cp {} {}".format(
                os.path.join(TEMPORARY_DIR, "listener-node.py"),
                os.path.join(dev_ws, "src", pkg_name, pkg_name, "listener.py"),
            )
        )
        run_sys_cmd(
            "cp {} {}".format(
                os.path.join(TEMPORARY_DIR, "talker_launch.py"),
                os.path.join(
                    dev_ws, "src", pkg_name, "launch", "talker_launch.py"
                ),
            )
        )
        run_sys_cmd(
            "cp {} {}".format(
                os.path.join(TEMPORARY_DIR, "listener_launch.py"),
                os.path.join(
                    dev_ws, "src", pkg_name, "launch", "listener_launch.py"
                ),
            )
        )

    io_maps_json = open(
        os.path.join(dev_ws, "src", pkg_name, pkg_name, "io_maps.json"), "w"
    )
    io_maps_json.write(json.dumps(io_maps))
    io_maps_json.close()

    # Build the FPGA node package
    _logger.info("Building the FPGA ROS2 node package")
    run_sys_cmd(["colcon build --packages-select " + pkg_name], cwd=dev_ws)


def create_vivado_bd(prj_name, device_part, ips_path, ips):
    args = "-project_name {} -device_part {} -ips_directory {}".format(
        prj_name, device_part, ips_path
    )
    args += " -auto_connect -write_bitstream -start_gui"
    for name, count in ips.items():
        args += " -ip {} {}".format(name, count)
    run_sys_cmd(
        [
            "vivado -nolog -nojournal -mode batch \
            -source {} -tclargs {}".format(
                os.path.join(PACKAGE_INSTALLED_DIR, "create_bd.tcl"), args
            )
        ]
    )


def generate_block_design(args):
    _logger.info("vivado block design generation mode")
    try:
        f = open("config.forest", "r")
    except Exception:
        _logger.error(
            "Config file could not be opened! "
            "meta-FOrEST requires config.forest "
            "to be present in the execution location"
        )
        sys.exit(1)
    prj = "meta_forest_vivado"
    device_part = ""
    ip_path = ""
    ips = {}
    config_data = f.read().splitlines()
    f.close()
    for line in config_data:
        # Parse input and assign values to variables
        if line.startswith("*") or line.startswith("/") or not line:
            continue
        index_delim = line.find(":")
        key, value = line[0:index_delim], line[index_delim + 1 :]
        key = key.strip()
        value = value.strip()

        # Setup Information
        if key == "meta-FOrEST project name":
            prj += "_" + value
            _logger.info(
                "vivado project will be generated in "
                "{}/{}".format(os.getcwd(), prj)
            )
        elif key == "FPGA board part":
            device_part = value
        elif key == "Absolute IP path":
            ip_path = value
        elif key == "User IP name":
            reading_ip_name = value
            if reading_ip_name not in ips:
                ips[reading_ip_name] = 0
        elif key == "User IP count":
            ips[reading_ip_name] += int(value)
    create_vivado_bd(prj, device_part, ip_path, ips)


def generate_config_forest(args):
    _logger.info("Config file generation mode")
    if not all(map(lambda io_n: io_n > 0, args.input + args.output)):
        _logger.error("Need the number of input and output signals")
        exit(1)
    if len(args.input) != len(args.output):
        _logger.error(
            "Need to specify the same number of input and output signals"
        )
        exit(1)
    # Check if config.forest file already exists

    if os.path.isfile("config.forest"):
        overwrite = input(
            "config.forest file already exists. " "Overwrite it? (y/n)"
        )
        if "y" not in overwrite:
            exit(1)
    # Render config file template
    env = Environment(
        loader=FileSystemLoader(
            os.path.join(PACKAGE_INSTALLED_DIR, "templates")
        )
    )
    render_config_file(env, len(args.input), args.input, args.output)


def generate_node(args):

    # Setup

    env = Environment(
        loader=FileSystemLoader(
            os.path.join(PACKAGE_INSTALLED_DIR, "templates")
        )
    )

    prj = "meta_forest_"
    bit_file = ""
    ip_name = ""
    ip_count = 1
    ip_names = []
    dev_ws = ""
    io_maps = {"map_num": {}, "maps": {}}
    map_num = 0

    reading_direction = None
    depend_std_msgs = False

    try:
        f = open("config.forest", "r")
    except Exception:
        _logger.error(
            "Config file could not be opened! "
            "meta-FOrEST requires config.forest "
            "to be present in the execution location"
        )
        sys.exit(1)
    config_data = f.read().splitlines()
    f.close()
    for line in config_data:
        # Parse input and assign values to variables
        if line.startswith("*") or line.startswith("/") or not line:
            continue
        index_delim = line.find(":")
        key, value = line[0:index_delim], line[index_delim + 1 :]
        key = key.strip()
        value = value.strip()

        direction_match = re.match(r"(Input|Output) name", key)
        # Setup Information
        if key == "meta-FOrEST project name":
            prj += value
        elif key == "Absolute ROS2 dev_ws path":
            dev_ws = value
        elif key == "Absolute FPGA .bit file path":
            bit_file = value
        elif key == "User IP name":
            map_num += 1
            io_maps["maps"].update({map_num: {"input": {}, "output": {}}})
            ip_name = value
        elif key == "User IP count":
            ip_count = int(value)
            for i in range(ip_count):
                ip_name_ = ip_name.strip() + "_%d" % i
                io_maps["map_num"].update({ip_name_: map_num})
                ip_names.append(ip_name_)
        elif direction_match:
            reading_direction = direction_match.groups()[0].lower()
            io_maps["maps"][map_num][reading_direction].update({value: {}})
            signal_name = value
        elif key == "Type":
            type_match = re.search(
                r"(?P<unsigned>[u|U]?)(?P<type>[a-zA-Z]+)" r"(?P<n_bits>\d+)",
                value,
            )
            # Do error handling
            if not type_match:
                _logger.error(
                    "Config file error. "
                    "{} type not recognized at signal {} in {}".format(
                        reading_direction, signal_name, ip_name
                    )
                )
                sys.exit(1)
            n_elem_match = re.search(r".*?\[(\d+)\]", value)
            type_parsed = type_match.groupdict()
            type_dict = {}
            type_dict["type"] = type_parsed["type"].lower()
            type_dict["n_bits"] = int(type_parsed["n_bits"])
            type_dict["signed"] = type_parsed["unsigned"] == ""
            type_dict["n_elem"] = (
                int(n_elem_match.groups()[0])
                if n_elem_match is not None
                else 1
            )
            type_dict["arr"] = n_elem_match is not None
            io_maps["maps"][map_num][reading_direction][signal_name].update(
                type_dict
            )
        elif key == "Use MultiArrayLayout":
            layout_name = ""
            if value.lower() == "true":
                depend_std_msgs = True
                layout_name = signal_name + "_layout"
            io_maps["maps"][map_num][reading_direction][signal_name][
                "layout_name"
            ] = layout_name
        elif key == "Protocol":
            io_maps["maps"][map_num][reading_direction][signal_name][
                "protocol"
            ] = value.lower()
        elif "Address" in key:
            if (
                io_maps["maps"][map_num][reading_direction][signal_name][
                    "protocol"
                ]
                == "lite"
            ):
                io_maps["maps"][map_num][reading_direction][signal_name][
                    "addr"
                ] = int(value)
            else:
                io_maps["maps"][map_num][reading_direction][signal_name][
                    "addr"
                ] = None

    for io_map in io_maps["maps"].values():
        check_in_map_validity(io_map["input"])
        check_out_map_validity(io_map["output"])

    if not os.path.exists(TEMPORARY_DIR):
        os.makedirs(TEMPORARY_DIR)

    head_ip_name = next(iter(io_maps["map_num"]))

    qos = 10

    # Rendering of template files
    _logger.info("Rendering templates according to user inputs...")
    # Render interface package.xml
    render_int_package_xml(env, prj, depend_std_msgs)

    # Render interface CMakeLists.txt
    fpga_in_msg = [
        os.path.join("msg", "FpgaIn" + str(map_num) + ".msg")
        for map_num in io_maps["maps"].keys()
    ]
    fpga_out_msg = [
        os.path.join("msg", "FpgaOut" + str(map_num) + ".msg")
        for map_num in io_maps["maps"].keys()
    ]
    render_int_cmakelists_txt(
        env, prj, fpga_in_msg + fpga_out_msg, depend_std_msgs
    )

    # Render node package.xml
    render_node_package_xml(env, prj)

    # Render node setup.py
    render_node_setup_py(env, prj, args.test)

    # Render node ros_fpga_lib.py
    render_node_ros_fpga_lib_py(env, prj, bit_file)

    # Render node fpga_node.py
    render_node_fpga_node_py(env, prj, qos, head_ip_name)

    # Render node fpga_node_launch.py
    render_node_fpga_launch(env, prj, list(io_maps["map_num"].keys()))

    # Render talker and listener if the test option is enabled
    if args.test:
        render_test_talker(env, prj, qos, head_ip_name)
        render_test_listener(env, prj, qos, head_ip_name)
        render_test_talker_launch(env, prj, io_maps["map_num"])
        render_test_listener_launch(env, prj, io_maps["map_num"])

    # Generate message package
    create_msg_pkg(dev_ws, prj)
    for map_num, io_map in io_maps["maps"].items():
        create_msg_file(
            dev_ws, prj, map_num, io_map["input"], io_map["output"]
        )
    build_msg_pkg(dev_ws, prj)

    # Generate node package
    create_fpga_node_pkg(dev_ws, prj, args.test, io_maps)


def main():
    parser = argparse.ArgumentParser()

    subparsers = parser.add_subparsers()
    parser_gen_config_forest = subparsers.add_parser(
        "gen_config_forest",
        help="Generate a template config file to be used by the script",
    )
    parser_gen_config_forest.set_defaults(func=generate_config_forest)
    parser_gen_config_forest.add_argument(
        "-i",
        "--input",
        metavar="N",
        type=int,
        action="append",
        required=True,
        help="Number of input signals for the template config file",
    )
    parser_gen_config_forest.add_argument(
        "-o",
        "--output",
        metavar="N",
        type=int,
        action="append",
        required=True,
        help="Number of output signals for the template config file",
    )

    parser_block_design = subparsers.add_parser(
        "gen_block_design",
        help="Generate a Vivado block design \
        according to the description in config file",
    )
    parser_block_design.set_defaults(func=generate_block_design)

    parser_gen_node = subparsers.add_parser(
        "gen_node",
        help="Generate ROS2FPGA Nodes \
        according to the description in config file",
    )
    parser_gen_node.set_defaults(func=generate_node)
    parser_gen_node.add_argument(
        "-t",
        "--test",
        action="store_true",
        help="Generate simple talker and listener nodes \
        along with the FPGA ROS node",
    )

    args = parser.parse_args()
    if len(sys.argv) < 2:
        parser.print_help()
        exit(1)
    args.func(args)


if __name__ == "__main__":
    main()
