import subprocess
import sys
import os
import json
import getopt
from jinja2 import Environment, FileSystemLoader

# Helper functions

def run_sys_cmd(cmd, cwd=None):

    # Helper function for running Linux commands

    subprocess.run(cmd, cwd=cwd, stderr=sys.stderr, stdout=sys.stdout, shell=True)
    return

def process_msg_file(filename, io_map):

    # Generates the appropriate input and output ROS2 message files given the user logic input and output signals
    
    f = open("output/" + filename + "-int.msg", "w")
    for signal_name in io_map.keys():
        is_arr = io_map[signal_name]["arr"]
        signal_type = io_map[signal_name]["type"]
        num_bits = str(io_map[signal_name]["n_bits"])
        signed = io_map[signal_name]["signed"]
        ros2_type = ""
        if not signed:
            ros2_type = "u"
        ros2_type += signal_type
        ros2_type += num_bits
        if is_arr:
            n_elem = str(io_map[signal_name]["n_elem"])
            ros2_type += "[" + n_elem + "]"
        f.write(ros2_type + " " + signal_name + "\n")
    f.close()
    return

def usage():
    # Prints the optional commands for the script
    print("\n[Forest]: usage: python3 forest.py [-h] [-t] [-g [-i ninputs -o noutputs]]")
    print("\n-h or --help: Prints the usage statement for the script")
    print("\n-t or --test: Generates simple talker and listener nodes along with the FPGA ROS node")
    print("\n-g or --genconfig: Generates a template config file to be used by the script")
    print("\n-i or --ninputs: Number of input signals for the template config file")
    print("\n-o or --noutputs: Number of output signals for the template config file\n")

def check_in_map_validity(in_map):
    # Check the input map generated after parsing the config file, and
    # verify its validity

    n_axis = 0
    err_found = False
    wrong_in = ""
    for input_signal in in_map.keys():
        properties = in_map[input_signal].keys()
        if "protocol" not in properties:
            err_found = True
            wrong_in = input_signal
        elif "type" not in properties:
            err_found = True
            wrong_in = input_signal
        elif "n_bits" not in properties:
            err_found = True
            wrong_in = input_signal
        elif "signed" not in properties:
            err_found = True
            wrong_in = input_signal
        elif "arr" not in properties:
            err_found = True
            wrong_in = input_signal
        elif "n_elem" not in properties:
            err_found = True
            wrong_in = input_signal
        elif "addr" not in properties:
            err_found = True
            wrong_in = input_signal
        
        if in_map[input_signal]["protocol"] == "stream":
            n_axis+=1

        if n_axis > 1:
            print("\n[Forest]: Error! Only 1 input signal can use the AXI-Stream protocol\n")
            sys.exit(1)

    if err_found:
        print("\n[Forest]: Input map is not valid. Make sure the input definition is correct. Error at signal {}\n". format(wrong_in))
        sys.exit(1)

def check_out_map_validity(out_map):

    # Check the output map generated after parsing the config file, and
    # verify its validity

    n_axis = 0
    err_found = False
    wrong_out = ""
    for output_signal in out_map.keys():
        properties = out_map[output_signal].keys()
        if "protocol" not in properties:
            err_found = True
            wrong_out = output_signal
        elif "type" not in properties:
            err_found = True
            wrong_out = output_signal
        elif "n_bits" not in properties:
            err_found = True
            wrong_out = output_signal
        elif "signed" not in properties:
            err_found = True
            wrong_out = output_signal
        elif "arr" not in properties:
            err_found = True
            wrong_out = output_signal
        elif "n_elem" not in properties:
            err_found = True
            wrong_out = output_signal
        elif "addr" not in properties:
            err_found = True
            wrong_out = output_signal

        if not out_map[output_signal]["arr"]:
            print("\n[Forest]: Error! All output signals must be an array types at signal {}\n".format(output_signal))
            sys.exit(1)
        
        if out_map[output_signal]["protocol"] == "stream":
            n_axis+=1

        if n_axis > 1:
            print("\n[Forest]: Error! Only 1 output signal can use the AXI-Stream protocol\n")
            sys.exit(1)

    if err_found:
        print("\n[Forest]: Output map is not valid. Make sure the output definition is correct. Error at signal {}\n". format(wrong_out))
        sys.exit(1)


# Rendering functions

def render_config_file(env, n_config_ips, n_config_inputs, n_config_outputs):

    # Render config.forest template when running in genconfig mode
    
    config_file = env.get_template("config.forest.jinja2")
    f = open("config.forest", "w")
    f.write(config_file.render(n_ips=n_config_ips, n_in=n_config_inputs, n_out=n_config_outputs))
    f.close()

    return

def render_int_package_xml(env, prj):

    # Render package.xml for the interface package
    
    package_xml = env.get_template("package-int.xml.jinja2")
    f = open("output/package-int.xml", "w")
    f.write(package_xml.render(prj_name=prj))
    f.close()

    return

def render_int_cmakelists_txt(env, prj, msg_files):

    # Render CMakeLists.txt for the interface package

    cmake_txt = env.get_template("CMakeLists-int.txt.jinja2")
    f = open("output/CMakeLists-int.txt", "w")
    f.write(cmake_txt.render(prj_name=prj, msg_files=msg_files))
    f.close()

    return

def render_node_package_xml(env, prj):

    # Render package.xml for the node package

    package_xml = env.get_template("package-node.xml.jinja2")
    f = open("output/package-node.xml", "w")
    f.write(package_xml.render(prj_name=prj))
    f.close()

    return

def render_node_setup_py(env, prj, test_enabled):

    # Render setup.py for the node package

    setup_py = env.get_template("setup.py.jinja2")
    f = open("output/setup-node.py", "w")
    f.write(setup_py.render(prj_name=prj, test_enabled=test_enabled))
    f.close()

    return

def render_node_ros_fpga_lib_py(env, prj, bit_file):
    # Render ros_fpga_lib.py for the node package

    ros_fpga_lib_py = env.get_template("ros_fpga_lib.py.jinja2")
    f = open("output/ros_fpga_lib-node.py", "w")
    f.write(ros_fpga_lib_py.render(prj_name=prj, bit_file=bit_file))
    f.close()

    return

def render_node_fpga_node_py(env, prj, qos, head_ip_name):

    # Render fpga_node.py for the node package

    fpga_node_py = env.get_template("fpga_node.py.jinja2")
    f = open("output/fpga_node-node.py", "w")
    f.write(fpga_node_py.render(qos=qos, prj_name=prj, ip_name=head_ip_name))
    f.close()

    return

def render_node_fpga_launch(env, prj, ip_names):

    # Render fpga_node_launch.py for the node package

    launch_file_name = "fpga_node_launch.py"
    fpga_node_launch_py = env.get_template(launch_file_name + ".jinja2")
    f = open("output/" + launch_file_name, "w")
    f.write(fpga_node_launch_py.render(prj_name=prj, ip_names=ip_names))
    f.close()

    return

def render_test_talker(env, prj, qos, ip_name):

    # Render the message generation file for the node package

    talker_py = env.get_template("talker.py.jinja2")
    f = open("output/talker-node.py", "w")
    f.write(talker_py.render(prj_name=prj, qos=qos, ip_name=ip_name))
    f.close()

    return

def render_test_talker_launch(env, prj, ip_map_nums):

    # Render talker_launch.py for the node package

    launch_file_name = "talker_launch.py"
    talker_launch_py = env.get_template(launch_file_name + ".jinja2")
    f = open("output/" + launch_file_name, "w")
    f.write(talker_launch_py.render(prj_name=prj, ip_map_nums=ip_map_nums))
    f.close()

    return

def render_test_listener(env, prj, qos, ip_name):

    # Render the message reader file for the node package

    listener_py = env.get_template("listener.py.jinja2")
    f = open("output/listener-node.py", "w")
    f.write(listener_py.render(prj_name=prj, qos=qos, ip_name=ip_name))
    f.close()

    return

def render_test_listener_launch(env, prj, ip_map_nums):

    # Render listener_launch.py for the node package

    launch_file_name = "listener_launch.py"
    listener_launch_py = env.get_template(launch_file_name + ".jinja2")
    f = open("output/" + launch_file_name, "w")
    f.write(listener_launch_py.render(prj_name=prj, ip_map_nums=ip_map_nums))
    f.close()

    return

# Package generation functions

def create_msg_pkg(dev_ws, prj):

    # Creates interface package

    print("\n[Forest]: Generating the ROS2 package for the FPGA node messages\n")

    # Create ROS2 package for the ROS FPGA messages (interface)

    pkg_name = prj + '_interface'

    run_sys_cmd(['ros2 pkg create --build-type ament_cmake ' + pkg_name], cwd=dev_ws+'src/')

def create_msg_file(dev_ws, prj, map_num, in_map, out_map):
    pkg_name = prj + '_interface'
    # Create the FPGA message files
    fpga_in_msg = "FpgaIn" + str(map_num)
    fpga_out_msg = "FpgaOut" + str(map_num)
    process_msg_file(fpga_in_msg, in_map)
    process_msg_file(fpga_out_msg, out_map)


    run_sys_cmd(['mkdir -p msg'], cwd= dev_ws+ 'src/' + pkg_name)
    
    run_sys_cmd([('cp output/%s-int.msg ' % fpga_in_msg) + dev_ws + 'src/' + pkg_name + '/msg/%s.msg' % fpga_in_msg])
    run_sys_cmd([('cp output/%s-int.msg ' % fpga_out_msg) + dev_ws + 'src/' + pkg_name + '/msg/%s.msg'% fpga_out_msg])


def build_msg_pkg(dev_ws, prj):
    pkg_name = prj + '_interface'
    # Copy modified interface CMakeLists.txt

    run_sys_cmd(['cp output/CMakeLists-int.txt ' + dev_ws + 'src/' + pkg_name + '/CMakeLists.txt'])

    # Copy modified interface package.xml

    run_sys_cmd(['cp output/package-int.xml ' + dev_ws + 'src/' + pkg_name + '/package.xml'])

    # Build message package

    print("\n[Forest]: Building the FPGA ROS2 node messages package\n")

    run_sys_cmd(['colcon build --packages-select ' + pkg_name], cwd=dev_ws)

    return

def create_fpga_node_pkg(dev_ws, prj, test_enabled, io_maps):

    # Creates and builds the node package

    # Create ROS2 package for the ROS FPGA node

    print("\n[Forest]: Generating the ROS2 package for the FPGA node\n")

    pkg_name = prj + '_fpga_node'

    run_sys_cmd(['ros2 pkg create --build-type ament_python ' + pkg_name], cwd=dev_ws+'src/')

    # Copy modified node package.xml

    run_sys_cmd(['cp output/package-node.xml ' + dev_ws + 'src/' + pkg_name + '/package.xml'])

    # Copy modified node setup.py

    run_sys_cmd(['cp output/setup-node.py ' + dev_ws + 'src/' + pkg_name + '/setup.py'])

    # Copy modified node fpga_node.py

    run_sys_cmd(['cp output/fpga_node-node.py ' + dev_ws + 'src/' + pkg_name + '/' + pkg_name + '/fpga_node.py'])

    # Copy modified node ros_fpga_lib.py

    run_sys_cmd(['cp output/ros_fpga_lib-node.py ' + dev_ws + 'src/' + pkg_name + '/' + pkg_name + '/ros_fpga_lib.py'])

    # Copy modified node fpga_node_launch .py

    run_sys_cmd(['mkdir -p launch'], cwd=dev_ws + 'src/' + pkg_name)
    launch_file_name = "fpga_node_launch.py"
    run_sys_cmd([('cp output/%s ' % launch_file_name) + dev_ws + 'src/' + pkg_name + '/launch/'+ launch_file_name])

    # If running in test generation mode, copy the test nodes as well
    
    if test_enabled:
        run_sys_cmd(['cp output/talker-node.py ' + dev_ws + 'src/' + pkg_name + '/' + pkg_name + '/talker.py'])
        run_sys_cmd(['cp output/listener-node.py ' + dev_ws + 'src/' + pkg_name + '/' + pkg_name + '/listener.py'])
        run_sys_cmd(['cp output/talker_launch.py ' + dev_ws + 'src/' + pkg_name + '/launch' + '/talker_launch.py'])
        run_sys_cmd(['cp output/listener_launch.py ' + dev_ws + 'src/' + pkg_name + '/launch' + '/listener_launch.py'])

    io_maps_json = open(dev_ws + 'src/' + pkg_name + '/' + pkg_name + "/io_maps.json", "w")
    io_maps_json.write(json.dumps(io_maps))
    io_maps_json.close()

    # Build the FPGA node package
    
    print("\n[Forest]: Building the FPGA ROS2 node package\n")
    run_sys_cmd(['colcon build --packages-select ' + pkg_name], cwd=dev_ws)

    return


def create_vivado_bd(prj_name, device_part, ips_path, ips):
    args = "-project_name {} -device_part {} -ips_directory {}".format(prj_name, device_part, ips_path)
    args += " -auto_connect -write_bitstream -start_gui"
    for name, count in ips.items():
        args += " -ip {} {}".format(name, count)
    run_sys_cmd(["vivado -nolog -nojournal -mode batch -source create_bd.tcl -tclargs {}".format(args)])


# Main

def main():

    # Setup

    env = Environment(
        loader=FileSystemLoader('./templates/')
    )
 
    test_enabled = False

    gen_config_file = False

    gen_block_design = False

    n_config_inputs = []

    n_config_outputs = []

    # Parse command line arguments

    arg_list = sys.argv[1:] 
    # Options 
    short_options = "htgbi:o:"
    long_options = ["help", "test", "genconfig", "genbd", "ninputs=", "noutputs="]
    try: 
        args, _ = getopt.getopt(arg_list, short_options, long_options)
        # checking each argument 
        for curr_arg, curr_val in args:
            if curr_arg in ("-h", "--help"): 
                usage()
                sys.exit()
            if curr_arg in ("-t", "--test"): 
                print ("\n[Forest]: Test nodes will be generated\n")
                test_enabled = True
                break
            if curr_arg in ("-g", "--genconfig"): 
                print("\n[Forest]: Config file generation mode\n")
                gen_config_file = True
            if curr_arg in ("-b", "--genbd"):
                print("\n[Forest]: vivado block design generation mode\n")
                gen_block_design = True
            if curr_arg in ("-i", "--ninputs"):
                if gen_config_file:
                    for ninputs in curr_val.split(","):
                        n_config_inputs.append(int(ninputs))
                else:
                    usage()
                    sys.exit(1)
            if curr_arg in ("-o", "--noutputs"):
                if gen_config_file:
                    for noutputs in curr_val.split(","):
                        n_config_outputs.append(int(noutputs))
                else:
                    usage()
                    sys.exit(1)
                        
    except getopt.error as err: 
        print(str(err)) 
        usage()
        sys.exit(1)

    if gen_config_file:
        if not n_config_inputs or not n_config_outputs \
                or any([n_config_input == 0 for n_config_input in n_config_inputs]) \
                or any(n_config_output == 0 for n_config_output in n_config_outputs):
            print("\n[Forest]: Error! Need the number of input and output signals\n")
            usage()
        elif len(n_config_inputs) != len(n_config_outputs):
            print("\n[Forest]: Error! Need to specify the same number of input and output signals\n")
        else:
            n_config_ips = len(n_config_inputs)
            gen_new = True
            # Check if config.forest file already exists
            if os.path.isfile("config.forest"):
                overwrite = input("\n[Forest]: config.forest file already exists. Overwrite it? (y/n)")
                if "y" in overwrite:
                    gen_new = True
                else:
                    gen_new = False
            if gen_new:
                # Render config file template
                render_config_file(env, n_config_ips, n_config_inputs, n_config_outputs)
        sys.exit()

    if gen_block_design:
        try:
            f = open("config.forest", "r")
        except:
            print("\n[Forest]: Config file could not be opened! I need a config.forest file in the same directory as forest.py\n")
            sys.exit(1)
        prj = ""
        device_part = ""
        ip_path = ""
        ips = {}
        config_data = f.read().splitlines()
        for line in config_data:
            # Parse input and assign values to variables
            if not (line.startswith("*") or line.startswith("/") or not line):
                index_delim = line.find(":")
                key, value = line[0:index_delim], line[index_delim+1:]
                key = key.strip()
                value = value.strip()

                # Setup Information
                if key == "Forest project name":
                    prj = "forest_vivado_" + value
                    print("\n[Forest]: vivado project will be generated in {}/{} \n".format(os.getcwd(), prj))
                elif key == "FPGA board name":
                    device_part = value
                elif key == "Absolute IP path":
                    ip_path = value
                elif key == "User IP name":
                    reading_ip_name = value
                    if not  reading_ip_name in ips:
                        ips[reading_ip_name] = 0
                elif key == "User IP count":
                    ips[reading_ip_name] += int(value)

        create_vivado_bd(prj, device_part, ip_path, ips)
        sys.exit()

    # Parse config file

    prj = ""
    bit_file = ""
    ip_name = ""
    ip_count = 1
    ip_names = []
    dev_ws = ""
    io_maps = {"map_num":{}, "maps":{}}
    map_num = 0


    reading_input = False
    reading_output = False
    
    try:
        f = open("config.forest", "r")
    except:
        print("\n[Forest]: Config file could not be opened! I need a config.forest file in the same directory as forest.py\n")
        sys.exit(1)
    config_data = f.read().splitlines()
    for line in config_data:
        # Parse input and assign values to variables
        if not (line.startswith("*") or line.startswith("/") or not line):
            index_delim = line.find(":")
            key, value = line[0:index_delim], line[index_delim+1:]
            key = key.strip()
            value = value.strip()

            # Setup Information
            if key == "Forest project name":
                prj = value
            elif key == "Absolute ROS2 dev_ws path":
                dev_ws = value
            elif key == "Absolute FPGA .bit file path":
                bit_file = value
            elif key == "User IP name":
                map_num += 1
                io_maps["maps"].update({map_num:{"input":{}, "output":{}}})
                ip_name = value

            elif key == "User IP count":
                ip_count = int(value)
                for i in range(ip_count):
                    ip_name_ = ip_name.strip() + "_%d" % i
                    io_maps["map_num"].update({ip_name_:map_num})
                    ip_names.append(ip_name_)

            # Inputs
            elif key == "Input name":
                io_maps["maps"][map_num]["input"].update({value: {}})
                reading_input = True
                in_name = value
            elif key == "Protocol" and reading_input:
                io_maps["maps"][map_num]["input"][in_name]["protocol"] = value.lower()
            elif key == "Type" and reading_input:
                if value == "uint8":
                    io_maps["maps"][map_num]["input"][in_name]["type"] = "int"
                    io_maps["maps"][map_num]["input"][in_name]["n_bits"] = 8
                    io_maps["maps"][map_num]["input"][in_name]["signed"] = False
                    io_maps["maps"][map_num]["input"][in_name]["arr"] = False
                    io_maps["maps"][map_num]["input"][in_name]["n_elem"] = 1
                elif value == "int32":
                    io_maps["maps"][map_num]["input"][in_name]["type"] = "int"
                    io_maps["maps"][map_num]["input"][in_name]["n_bits"] = 32
                    io_maps["maps"][map_num]["input"][in_name]["signed"] = True
                    io_maps["maps"][map_num]["input"][in_name]["arr"] = False
                    io_maps["maps"][map_num]["input"][in_name]["n_elem"] = 1
                elif value == "float32":
                    io_maps["maps"][map_num]["input"][in_name]["type"] = "float"
                    io_maps["maps"][map_num]["input"][in_name]["n_bits"] = 32
                    io_maps["maps"][map_num]["input"][in_name]["signed"] = True
                    io_maps["maps"][map_num]["input"][in_name]["arr"] = False
                    io_maps["maps"][map_num]["input"][in_name]["n_elem"] = 1
                elif value == "float64":
                    io_maps["maps"][map_num]["input"][in_name]["type"] = "float"
                    io_maps["maps"][map_num]["input"][in_name]["n_bits"] = 64
                    io_maps["maps"][map_num]["input"][in_name]["signed"] = True
                    io_maps["maps"][map_num]["input"][in_name]["arr"] = False
                    io_maps["maps"][map_num]["input"][in_name]["n_elem"] = 1
                elif "uint8" in value and "[" in value:
                    io_maps["maps"][map_num]["input"][in_name]["type"] = "int"
                    io_maps["maps"][map_num]["input"][in_name]["n_bits"] = 8
                    io_maps["maps"][map_num]["input"][in_name]["signed"] = False
                    io_maps["maps"][map_num]["input"][in_name]["arr"] = True
                    n_elem = int(value.replace('uint8', '').replace('[', '').replace(']', ''))
                    io_maps["maps"][map_num]["input"][in_name]["n_elem"] = n_elem
                elif "uint64" in value and "[" in value:
                    io_maps["maps"][map_num]["input"][in_name]["type"] = "int"
                    io_maps["maps"][map_num]["input"][in_name]["n_bits"] = 64
                    io_maps["maps"][map_num]["input"][in_name]["signed"] = False
                    io_maps["maps"][map_num]["input"][in_name]["arr"] = True
                    n_elem = int(value.replace('uint64', '').replace('[', '').replace(']', ''))
                    io_maps["maps"][map_num]["input"][in_name]["n_elem"] = n_elem
                elif "int32" in value and "[" in value:
                    io_maps["maps"][map_num]["input"][in_name]["type"] = "int"
                    io_maps["maps"][map_num]["input"][in_name]["n_bits"] = 32
                    io_maps["maps"][map_num]["input"][in_name]["signed"] = True
                    io_maps["maps"][map_num]["input"][in_name]["arr"] = True
                    n_elem = int(value.replace('int32', '').replace('[', '').replace(']', ''))
                    io_maps["maps"][map_num]["input"][in_name]["n_elem"] = n_elem
                elif "float32" in value and "[" in value:
                    io_maps["maps"][map_num]["input"][in_name]["type"] = "float"
                    io_maps["maps"][map_num]["input"][in_name]["n_bits"] = 32
                    io_maps["maps"][map_num]["input"][in_name]["signed"] = True
                    io_maps["maps"][map_num]["input"][in_name]["arr"] = True
                    n_elem = int(value.replace('float32', '').replace('[', '').replace(']', ''))
                    io_maps["maps"][map_num]["input"][in_name]["n_elem"] = n_elem
                elif "float64" in value and "[" in value:
                    io_maps["maps"][map_num]["input"][in_name]["type"] = "float"
                    io_maps["maps"][map_num]["input"][in_name]["n_bits"] = 64
                    io_maps["maps"][map_num]["input"][in_name]["signed"] = True
                    io_maps["maps"][map_num]["input"][in_name]["arr"] = True
                    n_elem = int(value.replace('float64', '').replace('[', '').replace(']', ''))
                    io_maps["maps"][map_num]["input"][in_name]["n_elem"] = n_elem
                else:
                    print("\n[Forest]: Error! Config file error. Input type not recognized at signal {} in {}\n".format(in_name, ip_name))
                    sys.exit(1)
            elif "Address" in key and reading_input:
                if io_maps["maps"][map_num]["input"][in_name]["protocol"] == "lite":
                    if value.isnumeric():
                        io_maps["maps"][map_num]["input"][in_name]["addr"] = int(value)
                    else:
                        print("\n[Forest]: Error! Config file error. Address not recognized at signal {} in {}\n".format(in_name, ip_name))
                        sys.exit(1)
                else:
                    io_maps["maps"][map_num]["input"][in_name]["addr"] = None

            # Outputs
            elif key == "Output name":
                io_maps["maps"][map_num]["output"].update({value: {}})
                reading_input = False
                reading_output = True
                out_name = value
            elif key == "Protocol" and reading_output:
                io_maps["maps"][map_num]["output"][out_name]["protocol"] = value.lower()
            elif key == "Type" and reading_output:
                if "[" not in value or "]" not in value:
                    print("\n[Forest]: Error! Config file error. All output signals must be fixed size arrays at signal {} in {}\n".format(out_name, ip_name))
                    sys.exit(1)
                if "uint8" in value and "[" in value:
                    io_maps["maps"][map_num]["output"][out_name]["type"] = "int"
                    io_maps["maps"][map_num]["output"][out_name]["n_bits"] = 8
                    io_maps["maps"][map_num]["output"][out_name]["signed"] = False
                    io_maps["maps"][map_num]["output"][out_name]["arr"] = True
                    n_elem = int(value.replace('uint8', '').replace('[', '').replace(']', ''))
                    io_maps["maps"][map_num]["output"][out_name]["n_elem"] = n_elem
                elif "uint64" in value and "[" in value:
                    io_maps["maps"][map_num]["output"][out_name]["type"] = "int"
                    io_maps["maps"][map_num]["output"][out_name]["n_bits"] = 64
                    io_maps["maps"][map_num]["output"][out_name]["signed"] = False
                    io_maps["maps"][map_num]["output"][out_name]["arr"] = True
                    n_elem = int(value.replace('uint64', '').replace('[', '').replace(']', ''))
                    io_maps["maps"][map_num]["output"][out_name]["n_elem"] = n_elem
                elif "int32" in value and "[" in value:
                    io_maps["maps"][map_num]["output"][out_name]["type"] = "int"
                    io_maps["maps"][map_num]["output"][out_name]["n_bits"] = 32
                    io_maps["maps"][map_num]["output"][out_name]["signed"] = True
                    io_maps["maps"][map_num]["output"][out_name]["arr"] = True
                    n_elem = int(value.replace('int32', '').replace('[', '').replace(']', ''))
                    io_maps["maps"][map_num]["output"][out_name]["n_elem"] = n_elem
                elif "float32" in value and "[" in value:
                    io_maps["maps"][map_num]["output"][out_name]["type"] = "float"
                    io_maps["maps"][map_num]["output"][out_name]["n_bits"] = 32
                    io_maps["maps"][map_num]["output"][out_name]["signed"] = True
                    io_maps["maps"][map_num]["output"][out_name]["arr"] = True
                    n_elem = int(value.replace('float32', '').replace('[', '').replace(']', ''))
                    io_maps["maps"][map_num]["output"][out_name]["n_elem"] = n_elem
                elif "float64" in value and "[" in value:
                    io_maps["maps"][map_num]["output"][out_name]["type"] = "float"
                    io_maps["maps"][map_num]["output"][out_name]["n_bits"] = 64
                    io_maps["maps"][map_num]["output"][out_name]["signed"] = True
                    io_maps["maps"][map_num]["output"][out_name]["arr"] = True
                    n_elem = int(value.replace('float64', '').replace('[', '').replace(']', ''))
                    io_maps["maps"][map_num]["output"][out_name]["n_elem"] = n_elem
                else:
                    print("\n[Forest]: Error! Config file error. Output type not recognized at signal {} in {}\n".format(out_name, ip_name))
                    sys.exit(1)
            elif "Address" in key and reading_output:
                if io_maps["maps"][map_num]["output"][out_name]["protocol"] == "lite":
                    if value.isnumeric():
                        io_maps["maps"][map_num]["output"][out_name]["addr"] = int(value)
                    else:
                        print("\n[Forest]: Error! Config file error. Address not recognized at signal {} in {}\n".format(in_name, ip_name))
                        sys.exit(1)
                else:
                    io_maps["maps"][map_num]["output"][out_name]["addr"] = None
    f.close()

    for io_map in io_maps["maps"].values():
        check_in_map_validity(io_map["input"])
        check_out_map_validity(io_map["output"])

    print("\n[Forest]: Starting the tool...\n")
 
    if not os.path.exists('output'):
        os.makedirs('output')

    prj = "forest_" + prj
    head_ip_name = next(iter(io_maps["map_num"]))

    qos = 10

    if not dev_ws.endswith('/'):
        dev_ws += '/'

    # Rendering of template files

    print("\n[Forest]: Rendering templates according to user inputs...\n")

    # Render interface package.xml

    render_int_package_xml(env, prj)

    # Render interface CMakeLists.txt

    fpga_in_msg = ["msg/"+"FpgaIn"+str(map_num)+".msg" for map_num in io_maps["maps"].keys()]
    fpga_out_msg = ["msg/"+"FpgaOut"+str(map_num)+".msg" for map_num in io_maps["maps"].keys()]
    render_int_cmakelists_txt(env, prj, fpga_in_msg+fpga_out_msg)

    # Render node package.xml

    render_node_package_xml(env, prj)

    # Render node setup.py

    render_node_setup_py(env, prj, test_enabled)

    # Render node ros_fpga_lib.py

    render_node_ros_fpga_lib_py(env, prj, bit_file)

    # Render node fpga_node.py

    render_node_fpga_node_py(env, prj, qos, head_ip_name)

    # Render node fpga_node_launch.py
    render_node_fpga_launch(env, prj, list(io_maps["map_num"].keys()))

    # Render talker and listener if the test option is enabled

    if test_enabled:
        render_test_talker(env, prj, qos, head_ip_name)
        render_test_listener(env, prj, qos, head_ip_name)
        render_test_talker_launch(env, prj, io_maps["map_num"])
        render_test_listener_launch(env, prj, io_maps["map_num"])

    # Generate message package
    create_msg_pkg(dev_ws, prj)
    for map_num, io_map in io_maps["maps"].items():
        create_msg_file(dev_ws, prj, map_num, io_map["input"], io_map["output"])
    build_msg_pkg(dev_ws, prj)

    # Generate node package

    create_fpga_node_pkg(dev_ws, prj, test_enabled, io_maps)

    return 0

if __name__ == '__main__':
    main()


