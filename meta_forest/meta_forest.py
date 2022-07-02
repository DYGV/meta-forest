from . import config, logging_utils, ros_package, vivado_block_design


def setup(args):
    logging_utils.setup_logger(args.log_level)


def generate_config(args):
    render_params = config.get_render_params(args)
    config.create_config_template(args.config, render_params, args.force)


def generate_node(args):
    config_dict = config.load(args.config)

    message_package = ros_package.Message()
    node_package = ros_package.Node(args)

    message_package_params = node_package.configure_params(config_dict)
    node_package_params = message_package.configure_params(config_dict)

    message_package.create(node_package_params)
    node_package.create(message_package_params)

    message_package.build(node_package_params)
    node_package.build(message_package_params)


def generate_block_design(args):
    config_dict = config.load(args.config)
    params = vivado_block_design.configure_params(config_dict)
    vivado_block_design.create(params)
