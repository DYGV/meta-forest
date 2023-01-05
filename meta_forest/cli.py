import argparse
import sys

from .project import init_project
from .logging_utils import setup_logger
from .ros_packages import generate_packages
from .version import __version__
from .vivado_block_design import generate_block_design


def _build_arg_parser():
    parser = argparse.ArgumentParser(prog="meta-forest")
    parser.add_argument(
        "-v", "--version", action="version", version=f"%(prog)s {__version__}"
    )
    parser.add_argument(
        "-l",
        "--log_level",
        metavar="STR",
        default="info",
        type=str,
        help="set logging level (debug|info|warn|error) (Default: info)",
    )

    sub_parser = parser.add_subparsers()

    _build_init_project(sub_parser)
    _build_gen_block_design_parser(sub_parser)
    _build_gen_node_parser(sub_parser)

    return parser


def _build_init_project(parser):
    parser_init_project = parser.add_parser(
        "init",
        help="Generate meta-FOrEST project",
    )
    parser_init_project.add_argument(
        "-p",
        "--project",
        metavar="STR",
        type=str,
        default="meta_forest_project",
        help="meta-FOrEST project name",
    )
    parser_init_project.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="Force generation even if project directory already exists",
    )

    parser_init_project.add_argument(
        "-s",
        "--solution_dir",
        metavar="STR",
        type=str,
        help="Vitis HLS solution directory",
    )
    parser_init_project.set_defaults(func=init_project)

    return parser_init_project


def _build_gen_block_design_parser(parser):
    parser_block_design = parser.add_parser(
        "gen_block_design",
        help="Generate a Vivado block design \
        according to the description in config file",
    )
    parser_block_design.set_defaults(func=generate_block_design)

    return parser_block_design


def _build_gen_node_parser(parser):
    parser_gen_node = parser.add_parser(
        "gen_node",
        help="Generate ROS2-FPGA Nodes \
        according to the description in config file",
    )
    parser_gen_node.add_argument(
        "-r",
        "--ros2_ws",
        metavar="STR",
        type=str,
        help="ros2 workspace",
    )
    parser_gen_node.add_argument(
        "-b",
        "--bitstream",
        metavar="STR",
        type=str,
        help="bitstream path",
    )
    parser_gen_node.add_argument(
        "-t",
        "--test",
        action="store_true",
        help="Generate simple talker and listener nodes \
        along with the FPGA ROS node",
    )
    parser_gen_node.set_defaults(func=generate_packages)
    return parser_gen_node


def main():
    parser = _build_arg_parser()
    args = parser.parse_args()

    if "func" not in args:
        parser.print_help()
        sys.exit(1)

    setup_logger(args)
    args.func(args)
