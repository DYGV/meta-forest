import argparse
import sys

# from .project import init_project
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

    _build_gen_block_design_parser(sub_parser)
    _build_gen_node_parser(sub_parser)

    return parser


def _build_gen_block_design_parser(parser):
    parser_block_design = parser.add_parser(
        "gen_bd",
        help="Generate a Vivado block design \
        according to the description in config file",
    )
    parser_block_design.add_argument(
        "-d",
        "--ip_directory",
        required=True,
        type=str,
        action="append",
        help="exported IP path",
    )

    parser_block_design.add_argument(
        "--IP",
        type=str,
        required=True,
        action="append",
        help="ip",
    )

    parser_block_design.add_argument(
        "-t",
        "--target_part",
        type=str,
        required=True,
        help="target part",
    )

    parser_block_design.add_argument(
        "-c",
        "--count",
        type=int,
        required=True,
        action="append",
        help="ip count",
    )

    parser_block_design.add_argument(
        "-s",
        "--step_to",
        type=str,
        default="write_bitstream",
        choices=["connect", "write_bitstream"],
        help="step",
    )

    parser_block_design.set_defaults(func=generate_block_design)

    return parser_block_design


class GroupedAction(argparse.Action):
    groupspace = None

    def __call__(self, parser, namespace, values, nargs=None, option_string=None):
        group, dest = self.dest.split(".", 2)
        if dest == "ip":
            GroupedAction.groupspace = argparse.Namespace()
            if not hasattr(namespace, group):
                setattr(namespace, group, [])
            getattr(namespace, group).append(GroupedAction.groupspace)
        if not hasattr(GroupedAction.groupspace, dest):
            setattr(GroupedAction.groupspace, dest, [])
        getattr(GroupedAction.groupspace, dest).append(values)


def _build_gen_node_parser(parser):
    parser_gen_node = parser.add_parser(
        "gen_node",
        help="Generate ROS2-FPGA Nodes \
        according to the description in config file",
    )
    parser_gen_node.add_argument(
        "-p",
        "--package_name_prefix",
        required=True,
        metavar="STR",
        type=str,
        help="ROS2 package name prefix",
    )
    parser_gen_node.add_argument(
        "-w",
        "--workspace",
        metavar="STR",
        type=str,
        help="ros2 workspace",
    )

    parser_gen_node.add_argument(
        "-t",
        "--test",
        action="store_true",
        help="Generate simple talker and listener nodes \
        along with the FPGA ROS node",
    )

    parser_gen_node.add_argument(
        "-b",
        "--bitstream",
        required=True,
        metavar="STR",
        type=str,
        help="bitstream path",
    )

    parser_gen_node.add_argument(
        "-I",
        "--IP",
        action=GroupedAction,
        type=str,
        dest="ip.ip",
        default=argparse.SUPPRESS,
    )
    parser_gen_node.add_argument(
        "-c",
        "--count",
        type=int,
        action=GroupedAction,
        dest="ip.count",
        default=argparse.SUPPRESS,
    )
    parser_gen_node.add_argument(
        "-i",
        "--in",
        action=GroupedAction,
        type=str,
        dest="ip.input",
        default=argparse.SUPPRESS,
    )
    parser_gen_node.add_argument(
        "-o",
        "--out",
        action=GroupedAction,
        type=str,
        dest="ip.output",
        default=argparse.SUPPRESS,
    )
    parser_gen_node.add_argument(
        "--in_type",
        action=GroupedAction,
        type=str,
        dest="ip.input_type",
        default=argparse.SUPPRESS,
    )
    parser_gen_node.add_argument(
        "--out_type",
        type=str,
        action=GroupedAction,
        dest="ip.output_type",
        default=argparse.SUPPRESS,
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
