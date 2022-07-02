import argparse

from . import meta_forest
from ._version import __version__


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

    _build_gen_config_parser(sub_parser)
    _build_gen_block_design_parser(sub_parser)
    _build_gen_node_parser(sub_parser)

    return parser


def _build_gen_config_parser(parser):
    parser_gen_config = parser.add_parser(
        "gen_config",
        help="Generate a template config file to be used meta-FOrEST",
    )
    parser_gen_config.add_argument(
        "-c",
        "--config",
        metavar="STR",
        type=str,
        default="config.toml",
        help="Name of toml file to output (Default: config.toml)",
    )
    parser_gen_config.add_argument(
        "-n",
        "--number_of_IPs",
        metavar="INT",
        type=int,
        required=True,
        help="Number of IPs for the template config file",
    )
    parser_gen_config.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="Force generation even if config file already exists",
    )
    parser_gen_config.set_defaults(func=meta_forest.generate_config)

    return parser_gen_config


def _build_gen_block_design_parser(parser):
    parser_block_design = parser.add_parser(
        "gen_block_design",
        help="Generate a Vivado block design \
        according to the description in config file",
    )
    parser_block_design.add_argument(
        "-c",
        "--config",
        metavar="STR",
        type=str,
        default="config.toml",
        help="Name of toml file to input (Default: config.toml)",
    )
    parser_block_design.set_defaults(func=meta_forest.generate_block_design)

    return parser_block_design


def _build_gen_node_parser(parser):
    parser_gen_node = parser.add_parser(
        "gen_node",
        help="Generate ROS2-FPGA Nodes \
        according to the description in config file",
    )
    parser_gen_node.add_argument(
        "-c",
        "--config",
        metavar="STR",
        type=str,
        default="config.toml",
        help="Name of toml file to input (Default: config.toml)",
    )
    parser_gen_node.add_argument(
        "-t",
        "--test",
        action="store_true",
        help="Generate simple talker and listener nodes \
        along with the FPGA ROS node",
    )
    parser_gen_node.set_defaults(func=meta_forest.generate_node)
    return parser_gen_node


def main():
    parser = _build_arg_parser()
    args = parser.parse_args()

    if "func" not in args:
        parser.print_help()
        return

    meta_forest.setup(args)
    args.func(args)


if __name__ == "__main__":
    main()
