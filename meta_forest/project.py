import logging

_logger = logging.getLogger("meta-FOrEST")

import json
import os
import sys
import re
import xml.etree.ElementTree as ET


class Params:
    def __init__(self):
        self.top = ""
        self.target_part = ""
        self.signals = []


class Signal:
    def __init__(self):
        self.name = ""
        self.direction = ""
        self.type = ""
        #self.is_array = False
        #self.array_size = 0
        #self.is_unsigned = False


def find_cpp_primitive_type_from_ap_int(type_name):
    parsed_type_name = ""
    ap_int_match = re.search("ap_int", type_name)
    ap_uint_match = re.search("ap_uint", type_name)
    primitive_type_table = {"char": 8, "int": 32, "long": 64}

    if ap_uint_match:
        parsed_type_name = "unsigned "

    if ap_int_match or ap_uint_match:
        bits = re.search(".*<(.*?)>.*", type_name).group(1)
        for primitive_type, type_bits in primitive_type_table.items():
            if type_bits == int(bits):
                return parsed_type_name + primitive_type


def find_cpp_primitive_type(type_name):
    ap_int2primitive = find_cpp_primitive_type_from_ap_int(type_name)
    if ap_int2primitive:
        type_name = ap_int2primitive
    types = ["char", "int", "long", "float", "double"]
    is_unsigned = False
    is_array = False
    for primitive_type in types:
        primitive_type_match = re.search(primitive_type + "+[*]?", type_name)
        if primitive_type_match:
            unsigned_type_match = re.search("unsigned", type_name)
            if "*" in primitive_type_match.group():
                is_array = True
            if unsigned_type_match:
                is_unsigned = True
            primitive_type = primitive_type
            return (is_unsigned, primitive_type, is_array)

def find_array_size(solution_base_dir, top_name):
    adb_file_path = os.path.join(
        solution_base_dir, ".autopilot", "db", f"{top_name}.adb"
    )
    tree = ET.parse(adb_file_path)
    root = tree.getroot()
    array_size_dict = {}
    for item in root.iter("item"):
        value, obj, name = (None, None, None)
        array_size = None
        value = item.find("Value")
        if value:
            obj = value.find("Obj")
        if obj:
            name = obj.find("name")
        array_size = item.find("array_size")
        if array_size is None or obj is name:
            continue
        array_size_dict[name.text] = int(array_size.text)
    return array_size_dict


def load_project_file(path):
    file = open(os.path.join(path, ".project.json"))
    return json.load(file)


def load_config_file(path):
    file = open(os.path.join(path, "config.json"))
    return json.load(file)


def parse_solution_file(path):
    json_file_name = f"{os.path.basename(path)}_data.json"
    json_file_path = os.path.join(path, json_file_name)

    solution_file = open(json_file_path)
    solution_data = json.load(solution_file)
    params = Params()
    params.top = solution_data["Top"]
    array_size_dict = find_array_size(path, params.top)
    args = solution_data["Args"]
    interfaces = solution_data["Interfaces"]
    target = solution_data["Target"]
    params.target_part = target["Device"] + target["Package"] + target["Speed"]
    for arg_name, arg_detail in solution_data["Args"].items():
        signal = Signal()
        params.signals.append(signal)
        signal.name = arg_name
        signal.direction = arg_detail["direction"]
        interface_name = arg_detail["hwRefs"][0]["interface"]
        interface = interfaces[interface_name]
        protocol = interface["type"]
        type_dict = {"type": "", "is_unsigned": False, "data_width": 0, "array_size": 0}
        type_dict["data_width"] = int(interface["dataWidth"])
        # signal.data_width = int(interface["dataWidth"])
        if protocol == "axi4lite":
            constraints = interface["constraints"]
            for array_signal_name, array_size in array_size_dict.items():
                if array_signal_name == signal.name:
                    if signal.direction == "out" and array_size == 0:
                        #signal.array_size = 1
                        type_dict["array_size"] = 1
                    else:
                        #signal.array_size = array_size
                        type_dict["array_size"] = array_size
        primitive_type_match = find_cpp_primitive_type(arg_detail["srcType"])
        if primitive_type_match:
            #signal.is_unsigned = primitive_type_match[0]
            #signal.type = primitive_type_match[1]
            #signal.is_array = primitive_type_match[2]
            type_dict["is_unsigned"] = primitive_type_match[0]
            type_dict["type"] = primitive_type_match[1]
        if protocol == "axi4stream":
            #signal.is_array = True
            #signal.array_size = 1
            type_dict["array_size"] = 1
        signal.type = _build_ros2_type(type_dict)


    signals = [dict(vars(i)) for i in params.signals]
    packed_signals = {
        params.top: [
            {"count": 1},
            {"signals": signals},
        ]
    }
    project_settings = {
        "target_part": params.target_part,
        "vitis_hls_solution": {params.top: path},
    }
    return (packed_signals, project_settings)

def _build_ros2_type(type_dict):
    ros2_type = ""
    if type_dict["is_unsigned"]:
        ros2_type += "u"
    ros2_type += type_dict["type"]
    ros2_type += str(type_dict["data_width"])
    if type_dict["array_size"] > 0:
        ros2_type += f"[{type_dict['array_size']}]"
    return ros2_type


class PathParams:
    pass

def make_project_setting(project_settings, project_name):
    params = PathParams()
    params.project_name = project_name
    params.project_path = os.path.abspath(project_name)
    params.target_part = project_settings["target_part"]
    params.solution_path = []
    params.solution_path.append(project_settings["vitis_hls_solution"])

    params.vivado_block_design = {
        "auto_start_gui": 0,
        "auto_connect_block_design": 1,
        "to_step_write_bitstream": 1,
    }
    return dict(vars(params))


def init_project(args):
    if os.path.exists(args.project) and not args.force:
        _logger.error(
            "A config file already exists. "
            "Remove a file that already exists or use --force"
        )
        sys.exit(1)
    os.makedirs(args.project, exist_ok=args.force)
    _logger.info("Generating configuration file from Vitis HLS solution")
    parsed_dict, project_settings = parse_solution_file(
        os.path.abspath(args.solution_dir)
    )
    project_dict = make_project_setting(project_settings, args.project)
    with open(os.path.join(args.project, "config.json"), "w") as outfile:
        json.dump(parsed_dict, outfile, indent=2)

    with open(os.path.join(args.project, ".project.json"), "w") as outfile:
        json.dump(project_dict, outfile, indent=2)
