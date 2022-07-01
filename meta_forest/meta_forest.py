import argparse
import os
import sys

from . import config, ros_package, vivado_block_design
from .helper import render_to_template
from .logging_utils import _logger


def generate_config(args):
    if os.path.isfile("config.toml") and not args.force:
        _logger.error(
            "A config file already exists. "
            "To force generation, use the --force option."
        )
    render_params = config.get_render_params(args)
    if render_params is not None:
        render_to_template("config.toml.jinja2", "config.toml", render_params)


def generate_node(args):
    config_dict = config.parse("config.toml")

    message_package = ros_package.Message()
    node_package = ros_package.Node(args)

    message_package_params = node_package.configure_params(config_dict)
    node_package_params = message_package.configure_params(config_dict)

    message_package.create(node_package_params)
    node_package.create(message_package_params)

    message_package.build(node_package_params)
    node_package.build(message_package_params)


def generate_block_design(args):
    config_dict = config.parse("config.toml")
    params = vivado_block_design.configure_params(config_dict)
    vivado_block_design.create(params)


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers()

    parser_gen_config = subparsers.add_parser(
        "gen_config",
        help="Generate a template config file to be used by the script",
    )
    parser_gen_config.set_defaults(func=generate_config)
    parser_gen_config.add_argument(
        "-n",
        "--number_of_IPs",
        metavar="N",
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
