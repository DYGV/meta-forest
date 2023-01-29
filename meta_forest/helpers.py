import os
import subprocess
import sys

from jinja2 import Environment, FileSystemLoader

PACKAGE_INSTALLED_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPORARY_OUTPUT_DIR = os.path.join(PACKAGE_INSTALLED_DIR, "output")
TEMPLATE_DIR = os.path.join(PACKAGE_INSTALLED_DIR, "templates")


class Params:
    """Class for data container"""

    pass


def run_sys_cmd(cmd, cwd=None):
    """Shell command execution

    Parameters
    ----------
    cmd: str
        Command to execute in the shell
    cwd: str
        Shell runtime directory

    Returns
    -------
    None
    """

    subprocess.run(cmd, cwd=cwd, stderr=sys.stderr, stdout=sys.stdout, shell=True)


def render_to_template(template_file_name, output_file_path, params):
    """Jinja2 template renderings

    Parameters
    ----------
    template_file_name: str
        Path of the template file to render
    output_file_path: str
        Path of the rendered file
    params: dict
        Parameters to render to the template file

    Returns
    -------
    None
    """

    env = Environment(loader=FileSystemLoader(TEMPLATE_DIR))
    template_file = env.get_template(template_file_name)
    f = open(output_file_path, "w")
    f.write(template_file.render(params=params))
    f.close()
