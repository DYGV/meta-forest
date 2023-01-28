import argparse
import sys

from .logging_utils import setup_logger
from .ros_packages import generate_packages
from .version import __version__
from .vivado_block_design import generate_block_design


def _build_arg_parser():
    """meta-FOrEST command line argument builder

    Create meta-FOrEST subcommands and parameters using argparse

    Parameters
    ----------
    None

    Returns
    -------
    parser: argparse.ArgumentParser
        Parser of meta-FOrEST command
    """

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
    """Setting up the Vivado Block Design command (gen_bd)

    Parameters
    -------
    parser: argparse.ArgumentParser

    Returns
    -------
    None
    """
    parser_block_design = parser.add_parser(
        "gen_bd",
        help="Generate a Vivado block design",
    )
    parser_block_design.add_argument(
        "-d",
        "--ip_directory",
        required=True,
        type=str,
        metavar="STR",
        action="append",
        help="Exported IP Path",
    )

    parser_block_design.add_argument(
        "-I",
        "--IP",
        type=str,
        metavar="STR",
        required=True,
        action="append",
        help="IP Core Name; It is usually the top function of HLS.",
    )

    parser_block_design.add_argument(
        "-t",
        "--target_part",
        type=str,
        metavar="STR",
        required=True,
        help="Target FPGA Part",
    )

    parser_block_design.add_argument(
        "-c",
        "--count",
        type=int,
        metavar="INT",
        required=True,
        action="append",
        help="Number of IP Cores to be Added to the Block Design",
    )

    parser_block_design.add_argument(
        "-s",
        "--step_to",
        type=str,
        metavar="STR",
        default="write_bitstream",
        choices=["connect", "write_bitstream"],
        help="Steps on Vivado",
    )

    parser_block_design.set_defaults(func=generate_block_design)


class GroupedAction(argparse.Action):
    """Custom Action in the argparse module"""

    groupspace = None

    def __call__(self, parser, namespace, values, nargs=None, option_string=None):
        """Create a new Namespace each time --ip is passed as a command line argument"""
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
    """Setting up the ROS2-FPGA nodes generateion command (gen_bd)

    Parameters
    -------
    parser: ArgumentParser

    Returns
    -------
    None
    """

    parser_gen_node = parser.add_parser(
        "gen_node",
        help="Generate ROS2-FPGA Nodes",
    )
    parser_gen_node.add_argument(
        "-p",
        "--package_name_prefix",
        required=True,
        metavar="STR",
        type=str,
        help="ROS2 Package Name Prefix",
    )
    parser_gen_node.add_argument(
        "-w",
        "--workspace",
        metavar="STR",
        type=str,
        help="ROS2 Workspace Directory",
    )

    parser_gen_node.add_argument(
        "-t",
        "--test",
        action="store_true",
        help="Generate Talker/Listener Nodes",
    )

    parser_gen_node.add_argument(
        "-b",
        "--bitstream",
        required=True,
        metavar="STR",
        type=str,
        help="Bitstream Path",
    )

    parser_gen_node.add_argument(
        "-I",
        "--IP",
        action=GroupedAction,
        help="IP Core Name; It is usually the top function of HLS.",
        type=str,
        metavar="STR",
        dest="ip.ip",
        default=argparse.SUPPRESS,
    )
    parser_gen_node.add_argument(
        "-c",
        "--count",
        help="Number of IP Cores Used in The Generated ROS2-FPGA Nodes",
        type=int,
        metavar="INT",
        action=GroupedAction,
        dest="ip.count",
        default=argparse.SUPPRESS,
    )
    parser_gen_node.add_argument(
        "-i",
        "--in",
        help="Input variable names to be used in ROS2 custom messages. Must match the variable name in the data input of the top function argument.",
        action=GroupedAction,
        type=str,
        metavar="STR",
        dest="ip.input",
        default=argparse.SUPPRESS,
    )
    parser_gen_node.add_argument(
        "-o",
        "--out",
        help="Output variable names to be used in ROS2 custom messages. Must match the variable name in the data output of the top function argument.",
        action=GroupedAction,
        type=str,
        metavar="STR",
        dest="ip.output",
        default=argparse.SUPPRESS,
    )
    parser_gen_node.add_argument(
        "--in_type",
        action=GroupedAction,
        help="Type as input to be used in ROS2 custom messages(e.g. int32[1024])",
        type=str,
        metavar="STR",
        dest="ip.input_type",
        default=argparse.SUPPRESS,
    )
    parser_gen_node.add_argument(
        "--out_type",
        type=str,
        help="Type as output to be used in ROS2 custom messages; It must always be an array(e.g. int32[1])",
        metavar="STR",
        action=GroupedAction,
        dest="ip.output_type",
        default=argparse.SUPPRESS,
    )
    parser_gen_node.set_defaults(func=generate_packages)


def main():
    """Entry point of meta-FOrEST"""
    parser = _build_arg_parser()
    args = parser.parse_args()

    if "func" not in args:
        parser.print_help()
        sys.exit(1)

    setup_logger(args)
    args.func(args)
