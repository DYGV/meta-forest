import logging

import numpy as np
from pynq import MMIO, Overlay, PL, allocate

from .io_maps import *


class FpgaDriver:
    def __init__(
        self,
        bitfile,
        program_fpga,
        user_ip_name,
        ros2_msg_pkg,
        ros2_msg_intf_in,
        ros2_msg_intf_out,
    ):
        self.overlay = Overlay(bitfile, download=program_fpga)
        self.overlay.is_loaded = self.is_loaded
        self.user_ip = getattr(self.overlay, user_ip_name)
        self.in_map, self.out_map = self._init_io_map(
            user_ip_name, ros2_msg_pkg, ros2_msg_intf_in, ros2_msg_intf_out
        )
        self._setup_ip_core()

    def is_loaded(self):
        return True

    def _init_io_map(
        self, user_ip_name, ros2_msg_pkg, ros2_msg_intf_in, ros2_msg_intf_out
    ):
        ros2_in = ROS2Map.message_file(ros2_msg_pkg, ros2_msg_intf_in)
        ros2_out = ROS2Map.message_file(ros2_msg_pkg, ros2_msg_intf_out)
        if hasattr(self.overlay.parser, "pins"):
            io_map = generate_map(self.overlay.parser, user_ip_name, ros2_in, ros2_out)
        else:
            bit = self.overlay.bitfile_name
            io_map = generate_map(HWHMap.get_hwh(bit), user_ip_name, ros2_in, ros2_out)
        return (io_map.input, io_map.output)

    @classmethod
    def program_fpga(cls, bitfile):
        PL.reset()
        return Overlay(bitfile)

    @property
    def physical_address(self):
        return self.user_ip.mmio.base_addr

    @property
    def address_range(self):
        return self.user_ip.mmio.length

    def _allocate_pynq_buffer(self, signal):
        primitive_type = signal["type"]
        is_unsigned = signal["is_unsigned"]
        data_width = signal["data_width"]
        array_size = signal["array_size"]

        dtype = ""
        dtype += "u" if is_unsigned else ""
        dtype += primitive_type
        dtype += str(data_width)
        cls_dtype = getattr(np, dtype)
        return allocate(shape=(array_size,), dtype=cls_dtype)

    def _setup_ip_core(self):
        self.stream_signal_name_in = []
        self.stream_signal_name_out = []
        self.burst_signal_name_in = []
        self.burst_signal_name_out = []
        for signal, value in self.in_map.items():
            self.in_map[signal]["alloc_buffer"] = None
            protocol = self.in_map[signal]["protocol"]
            if protocol == "stream":
                self.stream_signal_name_in.append(signal)
                self.in_map[signal]["alloc_buffer"] = self._allocate_pynq_buffer(value)
                self.in_map[signal]["axi_dma_ip"] = getattr(
                    self.overlay, value["axi_dma"]
                )
            if protocol == "m_axi":
                self.burst_signal_name_in.append(signal)
                self.in_map[signal]["alloc_buffer"] = self._allocate_pynq_buffer(value)

        for signal, value in self.out_map.items():
            self.out_map[signal]["alloc_buffer"] = None
            protocol = self.out_map[signal]["protocol"]
            if protocol == "stream":
                self.stream_signal_name_out.append(signal)
                self.out_map[signal]["alloc_buffer"] = self._allocate_pynq_buffer(value)
                self.out_map[signal]["axi_dma_ip"] = getattr(
                    self.overlay, value["axi_dma"]
                )
            if protocol == "m_axi":
                self.burst_signal_name_out.append(signal)
                self.out_map[signal]["alloc_buffer"] = self._allocate_pynq_buffer(value)

    def cmd_stream_transfer(self):
        for signal_name in self.stream_signal_name_in:
            axi_dma_ip = self.in_map[signal_name]["axi_dma_ip"]
            alloc_buffer = self.in_map[signal_name]["alloc_buffer"]
            axi_dma_ip.sendchannel.transfer(alloc_buffer)
        for signal_name in self.stream_signal_name_out:
            axi_dma_ip = self.out_map[signal_name]["axi_dma_ip"]
            alloc_buffer = self.out_map[signal_name]["alloc_buffer"]
            axi_dma_ip.recvchannel.transfer(alloc_buffer)

    def cmd_burst_transfer(self):
        for signal_name in self.burst_signal_name_in:
            alloc_buffer_address = self.in_map[signal_name]["alloc_buffer"].device_address
            signal_address_offset = self.in_map[signal_name]["address_offset"]
            self.user_ip.write(signal_address_offset, alloc_buffer_address)
        for signal_name in self.burst_signal_name_out:
            alloc_buffer_address = self.out_map[signal_name]["alloc_buffer"].device_address
            signal_address_offset = self.out_map[signal_name]["address_offset"]
            self.user_ip.write(signal_address_offset, alloc_buffer_address)


    def do_calc(self):
        self.cmd_stream_transfer()
        self.cmd_burst_transfer()
        self.user_ip.write(0x00, 1)
        self.wait_dma_transfer()
        self.wait_result()

    def wait_dma_transfer(self):
        for signal_name in self.stream_signal_name_in:
            axi_dma_ip = self.in_map[signal_name]["axi_dma_ip"]
            if axi_dma_ip.sendchannel.running:
                axi_dma_ip.sendchannel.wait()
        for signal_name in self.stream_signal_name_out:
            axi_dma_ip = self.out_map[signal_name]["axi_dma_ip"]
            if axi_dma_ip.recvchannel.running:
                axi_dma_ip.recvchannel.wait()

    def wait_result(self):
        while not (self.user_ip.read(0x00) >> 1) & 0x0001:
            continue

    def process_input(self, signal_name, input_data):
        signal = self.in_map[signal_name]
        signal_type = signal["type"]
        is_array = signal["is_array"]
        data_width = signal["data_width"]
        alloc_buffer = signal["alloc_buffer"]
        protocol = signal["protocol"]
        if protocol == "stream":
            self.in_map[signal_name]["alloc_buffer"][:] = input_data
        elif protocol == "m_axi":
            self.in_map[signal_name]["alloc_buffer"][:] = input_data
        elif protocol == "lite":
            if is_array:
                if signal_type == "int":
                    if data_width == 8:
                        self.write_uint8_array_lite(signal, input_data)
                    elif data_width == 32:
                        self.write_uint32_array_lite(signal, input_data)
                elif signal_type == "float":
                    if data_width == 32:
                        self.write_float32_array_lite(signal, input_data)
                    elif data_width == 64:
                        self.write_float64_array_lite(signal, input_data)
            else:
                if signal_type == "int":
                    address_offset = signal["address_offset"]
                    self.user_ip.write(address_offset, int(input_data))
                elif signal_type == "float":
                    if data_width == 32:
                        ieee_rep = self.dec_to_ieee(input_data, "float")
                        self.user_ip.write(address_offset, ieee_rep)
                    elif data_width == 64:
                        ieee_rep_hi, ieee_rep_lo = self.dec_to_ieee(
                            input_data, "double"
                        )
                        self.user_ip.write(address_offset, ieee_rep_lo)
                        self.user_ip.write(address_offset + 4, ieee_rep_hi)

    def process_output(self, signal_name):
        signal = self.out_map[signal_name]
        signal_type = signal["type"]
        is_array = signal["is_array"]
        data_width = signal["data_width"]
        protocol = signal["protocol"]
        alloc_buffer = signal["alloc_buffer"]
        if protocol == "stream":
            val = np.zeros(shape=alloc_buffer.shape, dtype=alloc_buffer.dtype)
            val[:] = alloc_buffer[:]
            return val
        elif protocol == "m_axi":
            val = np.zeros(shape=alloc_buffer.shape, dtype=alloc_buffer.dtype)
            val[:] = alloc_buffer[:]
            return val
        elif protocol == "lite":
            if is_array:
                if signal_type == "int":
                    if data_width == 8:
                        return self.read_uint8_array_lite(signal)
                    elif data_width == 32:
                        val = self.read_int32_array_lite(signal)
                        return val
                elif signal_type == "float":
                    if data_width == 32:
                        return self.read_float32_array_lite(signal)
                    elif data_width == 64:
                        return self.read_float64_array_lite(signal)

    def read_uint8_array_lite(self, signal):
        array_size = signal.get("array_size")
        val = [0] * array_size
        n_exact = array_size // 4
        n_last = array_size % 4
        padding = 4 - n_last
        addr = signal.get("address_offset")
        for i in range(n_exact):
            result = self.user_ip.read(addr).to_bytes(4, "little")
            for j in range(4):
                val[j + i] = int(result[j])
            addr += 4
        if n_last != 0:
            result = self.user_ip.read(addr).to_bytes(n_last, "little")
            for j in range(n_last):
                val[array_size - n_last + j] = int(result[j])
        return val

    def read_int32_array_lite(self, signal):
        array_size = signal.get("array_size")
        val = [0] * array_size
        head = signal.get("address_offset") 
        for i in range(array_size):
            val[i] = self.user_ip.read(head + (4 * i))
        return val

    def read_float32_array_lite(self, signal):
        array_size = signal.get("array_size")
        val = [0] * array_size
        for i in range(array_size):
            val_ieee = self.user_ip.read(signal.get("address_offset") + 4 * i)
            val_ieee = hex(val)[2:]
            val[i] = self.ieee_to_dec(val_ieee, "float")
        return val

    def read_float64_array_lite(self):
        array_size = signal.get("array_size")
        val = [0] * array_size
        array_head = signal.get("address_offset")
        for i in range(array_size):
            val_ieee_lo = self.user_ip.read(array_head + 8 * i)
            val_ieee_hi = self.user_ip.read(array_head + 8 * i + 4)
            val_ieee = hex((val_ieee_hi << 32) | val_ieee_lo)[2:]
            val[i] = self.ieee_to_dec(val_ieee, "double")
        return val

    def write_uint8_array_lite(self, signal, msg):
        array_size = signal.get("array_size")
        array_head = signal.get("address_offset")

        byte_arr = bytes(msg)
        lower_lim = 0
        upper_lim = 4
        n_exact = array_size // 4
        n_last = array_size % 4
        padding = 4 - n_last
        addr = array_head
        for i in range(n_exact):
            partition = byte_arr[lower_lim:upper_lim]
            self.user_ip.write(addr, partition)
            lower_lim += 4
            upper_lim += 4
            addr += 4
        if n_last != 0:
            partition = byte_arr[array_size - n_last : array_size]
            pad = bytes(padding)
            partition += pad
            self.user_ip.write(addr, partition)

    def write_uint32_array_lite(self, signal, msg):
        array_size = signal.get("array_size")
        array_head = signal.get("address_offset")
        for i in range(array_size):
            self.user_ip.write(array_head + (4 * i), int(msg[i]))

    def write_float32_array_lite(self, signal, msg):
        array_size = signal.get("array_size")
        array_head = signal.get("address_offset")
        # 1 array element per address
        for i in range(array_size):
            ieee_rep = self.dec_to_ieee(msg[i], "float")
            self.user_ip.write(array_head + 4 * i, ieee_rep)

    def write_float64_array_lite(self, signal, msg):
        array_size = signal.get("array_size")
        array_head = signal.get("address_offset")
        # 1 array element per 2 addresses
        for i in range(array_size):
            ieee_rep_hi, ieee_rep_lo = self.dec_to_ieee(msg[i], "double")
            self.user_ip.write(array_head + 8 * i, ieee_rep_lo)
            self.user_ip.write(array_head + 8 * i + 4, ieee_rep_hi)

    def dec_to_ieee(self, num, data_type):
        # Converts a decimal number to IEEE-754 representation
        if data_type == "float":
            num_ieee_hex = struct.pack(">f", num).hex()
            num_ieee = int(num_ieee_hex, 16)
            return num_ieee
        elif data_type == "double":
            num_ieee = struct.pack(">d", num).hex()
            num_hi = int(num_ieee[:8], 16)
            num_lo = int(num_ieee[8:], 16)
            return num_hi, num_lo

    def ieee_to_dec(self, num_ieee, data_type):
        # Converts a number in IEEE-754 representation to its decimal form
        if num_ieee != "0":
            num_in_bytes = bytes.fromhex(num_ieee)
            if data_type == "float":
                num_dec = struct.unpack(">f", num_in_bytes)[0]
            elif data_type == "double":
                num_dec = struct.unpack(">d", num_in_bytes)[0]
        else:
            num_dec = 0
        return num_dec

