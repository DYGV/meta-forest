import fnmatch
import re

from pynq.pl_server import hwh_parser
from rosidl_adapter.parser import parse_message_file
from rosidl_runtime_py import get_interface_path


class IOMap:
    @classmethod
    def _merge(cls, ros2_maps, hwh_maps):
        merged_dict = {}
        for ros2_interface in ros2_maps:
            for hwh_interface in hwh_maps:
                if vars(ros2_interface)["name"] == vars(hwh_interface)["name"]:
                    merged_interface = dict(vars(ros2_interface), **vars(hwh_interface))
                    name = merged_interface.pop("name", None)
                    merged_dict[name] = merged_interface
        return merged_dict


class HWHMap:
    def __init__(self, hwh, target_ip, signal_names):
        self.target_ip = target_ip
        self.signal_names = signal_names
        self.hwh = hwh
        self.transfer_data_maps = []
        self._configure()

    @classmethod
    def get_hwh(self, bitfile):
        hwh_file = hwh_parser.get_hwh_name(bitfile)
        return hwh_parser.HWH(hwh_file)

    def _configure(self):
        target_ip_dict = self.hwh.ip_dict.get(self.target_ip)

        if not target_ip_dict:
            return
        registers = target_ip_dict.get("registers")
        for x in self.signal_names:
            signal_register = registers.get(x)
            if signal_register:
                register = Register(signal_register)
                register._configure()
                address_offset = register.address_offset
                data_width = register.data_width
                axi_dma = ""
                protocol = "lite"
            elif registers.get(f"{x}_1"):
                register = Register(registers.get(f"{x}_1"))
                register._configure()
                address_offset = register.address_offset
                data_width = register.data_width
                axi_dma = ""
                protocol = "m_axi"
            else:
                dma = DMA_INTF(self.hwh, x, self.target_ip)
                dma._configure()
                address_offset = -1
                data_width = dma.data_width
                axi_dma = dma.axi_dma_name
                protocol = "stream"
            transfer_data_map = IOMap()
            transfer_data_map.name = x
            transfer_data_map.data_width = data_width
            transfer_data_map.address_offset = address_offset
            transfer_data_map.axi_dma = axi_dma
            transfer_data_map.protocol = protocol
            self.transfer_data_maps.append(transfer_data_map)


class Register:
    def __init__(self, register):
        self.register = register

    def _configure(self):
        self.address_offset = self.register.get("address_offset")
        self.data_width = self.register.get("size")


class DMA_INTF:
    def __init__(self, hwh, signal_name, target_ip):
        self.target_ip = target_ip
        self.axi_dma_name = ""
        self.data_width = 0
        self.hwh = hwh
        self.signal_name = signal_name
        self.tdata_types = {"input": "m_axis_mm2s_tdata", "output": "s_axis_s2mm_tdata"}
        self.tdata_width_types = {
            "input": f"c_{self.tdata_types['input']}_width",
            "output": f"c_{self.tdata_types['output']}_width",
        }

    def _find_corresponding_axi_dma_pins(self):
        return fnmatch.filter(self.hwh.pins.keys(), f"{self.target_ip}/*TDATA")

    def _configure(self):
        self._find_corresponding_axi_dma()
        pins = self._find_corresponding_axi_dma_pins()
        for pin in pins:
            pin_signal_name, _ = pin.split("/")[1].split("_TDATA")
            if pin_signal_name != self.signal_name:
                continue
            self.direction = self._find_direction(pin)
        tdata_width_type = self.tdata_width_types[self.direction]
        self.data_width = self.hwh.ip_dict[self.axi_dma_name]["parameters"][
            tdata_width_type
        ]

    def _find_direction(self, pin):
        for direction, tdata_type in self.tdata_types.items():
            if tdata_type in self.hwh.pins[pin]:
                return direction

    def _find_corresponding_axi_dma(self):
        pins = self._find_corresponding_axi_dma_pins()
        for pin in pins:
            dma_match = re.search(r"axi_dma_\d+", self.hwh.pins[pin])
            if dma_match:
                self.axi_dma_name = dma_match.group()


class ROS2Map:
    def __init__(self, parsed_message_file):
        self.maps = []
        self.message_spec = parsed_message_file

    @classmethod
    def message_file(cls, message_package, message_interface):
        return parse_message_file(
            message_package,
            get_interface_path(f"{message_package}/msg/{message_interface}"),
        )

    def _configure(self):
        for field in self.message_spec.fields:
            interface_map = IOMap()
            self.maps.append(interface_map)
            interface_map.name = field.name
            interface_map.is_unsigned = False
            if "int" in field.type.type:
                interface_map.type = "int"
                interface_map.is_unsigned = field.type.type.startswith("u")
            elif "float" in field.type.type:
                interface_map.type = "float"
            interface_map.is_array = field.type.is_array
            interface_map.array_size = (
                field.type.array_size if field.type.is_array else 0
            )


def generate_map(hwh, target_ip, parsed_message_file_in, parsed_message_file_out):
    ros2_maps_in = ROS2Map(parsed_message_file_in)
    ros2_maps_out = ROS2Map(parsed_message_file_out)
    ros2_maps_in._configure()
    ros2_maps_out._configure()
    signal_names_in = [vars(signal_name)["name"] for signal_name in ros2_maps_in.maps]
    signal_names_out = [vars(signal_name)["name"] for signal_name in ros2_maps_out.maps]

    hwh_maps_input = HWHMap(hwh, target_ip, signal_names_in).transfer_data_maps
    hwh_maps_output = HWHMap(hwh, target_ip, signal_names_out).transfer_data_maps

    io_map = IOMap()
    io_map.input = IOMap._merge(ros2_maps_in.maps, hwh_maps_input)
    io_map.output = IOMap._merge(ros2_maps_out.maps, hwh_maps_output)
    return io_map
