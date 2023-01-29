import logging
import os
import shutil
from itertools import chain

from .helpers import (TEMPLATE_DIR, TEMPORARY_OUTPUT_DIR, Params,
                      render_to_template, run_sys_cmd)


def _build_packages_with_colcon(dev_ws, packages_list):
    """Generate a Vivado block design

    Parameters
    ----------
    dev_ws: str
        Workspace of ROS2 packages
    packages_list: List[str]
        Names of ROS2 package to build

    Returns
    -------
    None
    """

    run_sys_cmd(
        ["colcon build --packages-select " + " ".join(packages_list)],
        cwd=dev_ws,
    )


class MessagePackage:
    """Class to create a package of ROS2 custom message"""

    def _configure_params(self, args):
        """Configure parameters to create a ROS2 package

        Parameters
        ----------
        args: argparse.Namespace

        Returns
        -------
        params: Params
        """

        params = Params()
        params.project = f"{args.package_name_prefix}_interface"
        params.dev_ws = args.workspace
        params.io_maps = args.ip
        params.msg_files = self._get_msg_files(args.ip)
        return params

    def _get_msg_files(self, ip):
        """Get ROS2 message file names

        Parameters
        ----------
        ip: List

        Returns
        -------
        msg_files: List[str]
            List of ROS2 message file names
        """

        msg_files = []
        for map_num in range(1, len(ip) + 1):
            msg_files.append(os.path.join("msg", f"FpgaIn{map_num}.msg"))
            msg_files.append(os.path.join("msg", f"FpgaOut{map_num}.msg"))
        return msg_files

    def _process_msg_file(self, filename, io, io_type):
        """Create the contents of custom messages

        Parameters
        ----------
        filename: str
        io: List
        io_type: List

        Returns
        -------
        None
        """

        ros2_type = ""
        for i in range(len(io)):
            signal_name = io[i]
            signal_type = io_type[i]

            ros2_type += f"{signal_type} {signal_name}\n"
        f = open(
            os.path.join(TEMPORARY_OUTPUT_DIR, "%s-int.msg" % filename),
            "w",
        )
        f.write(ros2_type)
        f.close()

    def _create_msg_file(self, params):
        """Create custom messages

        Parameters
        ----------
        params: Params

        Returns
        -------
        None
        """

        msg_dir = os.path.join(params.dev_ws, "src", params.project, "msg")
        if not os.path.exists(msg_dir):
            os.makedirs(msg_dir)
        for map_num, io_map in enumerate(params.io_maps, 1):
            fpga_in_msg = f"FpgaIn{map_num}"
            fpga_out_msg = f"FpgaOut{map_num}"
            self._process_msg_file(fpga_in_msg, io_map.input, io_map.input_type)
            self._process_msg_file(fpga_out_msg, io_map.output, io_map.output_type)
            shutil.copy(
                os.path.join(TEMPORARY_OUTPUT_DIR, f"{fpga_in_msg}-int.msg"),
                os.path.join(msg_dir, fpga_in_msg + ".msg"),
            )
            shutil.copy(
                os.path.join(TEMPORARY_OUTPUT_DIR, f"{fpga_out_msg}-int.msg"),
                os.path.join(msg_dir, fpga_out_msg + ".msg"),
            )

    def _render(self, params):
        """Render custom message packages

        Parameters
        ----------
        params: Params

        Returns
        -------
        None
        """

        if not os.path.exists(TEMPORARY_OUTPUT_DIR):
            os.makedirs(TEMPORARY_OUTPUT_DIR)
        render_to_template(
            "package-int.xml.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "package-int.xml"),
            params,
        )
        render_to_template(
            "CMakeLists-int.txt.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "CMakeLists-int.txt"),
            params,
        )

    def _create(self, params):
        """Create a custom message package

        Parameters
        ----------
        params: Params

        Returns
        -------
        None
        """

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
    """Class to create a packages of ROS2-FPGA nodes"""

    def _configure_params(self, args):
        """Configure parameters to create a ROS2 package

        Parameters
        ----------
        args: argparse.Namespace

        Returns
        -------
        params: Params
        """

        params = Params()
        params.project = f"{args.package_name_prefix}_fpga_node"
        params.ros2_interface_pkg = f"{args.package_name_prefix}_interface"
        params.ros2_interface_in_base = "FpgaIn"
        params.ros2_interface_out_base = "FpgaOut"
        params.ros2_interface_in = f"{params.ros2_interface_in_base}1"
        params.ros2_interface_out = f"{params.ros2_interface_out_base}1"
        params.dev_ws = args.workspace
        params.ros_distro = os.environ["ROS_DISTRO"]
        params.test_enabled = args.test
        params.bitfile_path = args.bitstream
        params.io_maps = args.ip
        params.qos = 10
        params.ip_names_2d = self._get_all_ip_names(params.io_maps)
        params.ip_names = list(chain.from_iterable(params.ip_names_2d))
        params.ip_msg_table = self._get_ip_msg_table(params.ip_names_2d)
        params.head_ip_name = params.ip_names[0]
        return params

    def _get_all_ip_names(self, io_maps):
        """Get IP core names with suffix

        Parameters
        ----------
        io_maps: List

        Returns
        -------
        ip_names: List
            IP core names
        """

        ip_names = []
        for i in range(len(io_maps)):
            name = io_maps[i].ip[0]
            count = io_maps[i].count[0]
            ip = [f"{name}_{x}" for x in range(count)]
            ip_names.append(ip)
        return ip_names

    def _get_ip_msg_table(self, ip_names_2d):
        """Table containing IP core names and ROS2 message file suffix

        Parameters
        ----------
        ip_names_2d: List

        Returns
        -------
        table: Dict
        """

        table = {}
        for i in range(1, len(ip_names_2d) + 1):
            ip_names = ip_names_2d[i - 1]
            for ip_name in ip_names:
                table[ip_name] = i
        return table

    def _render(self, params):
        """Render ROS2-FPGA nodes packages

        Parameters
        ----------
        params: Params

        Returns
        -------
        None
        """

        if not os.path.exists(TEMPORARY_OUTPUT_DIR):
            os.makedirs(TEMPORARY_OUTPUT_DIR)
        render_to_template(
            "package-node.xml.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "package-node.xml"),
            params,
        )
        render_to_template(
            "setup.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "setup-node.py"),
            params,
        )
        render_to_template(
            "fpga_node.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "fpga_node-node.py"),
            params,
        )
        render_to_template(
            "fpga_node_launch.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "fpga_node_launch.py"),
            params,
        )
        if not params.test_enabled:
            return
        render_to_template(
            "talker.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "talker-node.py"),
            params,
        )
        render_to_template(
            "talker_launch.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "talker_launch.py"),
            params,
        )
        render_to_template(
            "listener.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "listener-node.py"),
            params,
        )
        render_to_template(
            "listener_launch.py.jinja2",
            os.path.join(TEMPORARY_OUTPUT_DIR, "listener_launch.py"),
            params,
        )

    def _create(self, params):
        """Create a ROS2-FPGA nodes package

        Parameters
        ----------
        params: Params

        Returns
        -------
        None
        """

        src_dir = os.path.join(params.dev_ws, "src")
        if not os.path.exists(src_dir):
            os.makedirs(src_dir)
        run_sys_cmd(
            ["ros2 pkg create --build-type ament_python " + params.project],
            cwd=src_dir,
        )
        self._render(params)
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
            os.path.join(TEMPLATE_DIR, "pynq_driver.py"),
            os.path.join(
                params.dev_ws,
                "src",
                params.project,
                params.project,
                "pynq_driver.py",
            ),
        )
        shutil.copy(
            os.path.join(TEMPLATE_DIR, "io_maps.py"),
            os.path.join(
                params.dev_ws,
                "src",
                params.project,
                params.project,
                "io_maps.py",
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
        if not params.test_enabled:
            return
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
    """Generate ROS2 package

    Parameters
    ----------
    args: argparse.Namespace

    Returns
    -------
    None
    """

    logger = logging.getLogger("meta-FOrEST")
    message_package = MessagePackage()
    node_package = NodePackage()
    message_package_params = message_package._configure_params(args)
    node_package_params = node_package._configure_params(args)

    logger.info("Generating the ROS2 package for the FPGA node messages")
    message_package._create(message_package_params)

    logger.info("Generating the ROS2 package for the FPGA node")
    node_package._create(node_package_params)

    logger.info("Building the ROS2 packages")
    if args.step_to == "build":
        _build_packages_with_colcon(
            args.workspace,
            [message_package_params.project, node_package_params.project],
        )
