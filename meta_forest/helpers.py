import os
import subprocess
import sys

from jinja2 import Environment, FileSystemLoader

PACKAGE_INSTALLED_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPORARY_OUTPUT_DIR = os.path.join(PACKAGE_INSTALLED_DIR, "output")
TEMPLATE_DIR = os.path.join(PACKAGE_INSTALLED_DIR, "templates")


def run_sys_cmd(cmd, cwd=None):
    # Helper function for running Linux commands
    subprocess.run(
        cmd, cwd=cwd, stderr=sys.stderr, stdout=sys.stdout, shell=True
    )


def render_to_template(template_file_name, output_file_path, params):
    env = Environment(loader=FileSystemLoader(TEMPLATE_DIR))
    template_file = env.get_template(template_file_name)
    f = open(output_file_path, "w")
    f.write(template_file.render(params=params))
    f.close()
