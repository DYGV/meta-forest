import pytest

from meta_forest import config, vivado_block_design


@pytest.fixture
def pre_load_config():
    config_dict = config.load("tests/data/config_empty.toml")
    return config_dict


def test__configure_params(pre_load_config):
    output = vivado_block_design._configure_params(pre_load_config)
    assert hasattr(output, "project")
    assert hasattr(output, "ip_directory")
    assert hasattr(output, "board_part")
    assert hasattr(output, "ip_name_list")
    assert hasattr(output, "ip_count_list")


def test__build_command(pre_load_config):
    configured_params = vivado_block_design._configure_params(pre_load_config)
    output = vivado_block_design._build_command(configured_params)
    len_name = len(configured_params.ip_name_list)
    len_count = len(configured_params.ip_count_list)
    if len_name == 0:
        assert output == -1
    if len_count == 0:
        assert output == -1
    if len_name != len_count:
        assert output == -1
