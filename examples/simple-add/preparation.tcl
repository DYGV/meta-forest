open_project -reset vitis_hls

add_files ./hls/add.c

set_top add

open_solution -reset solution1
# Set PYNQ-Z1 board part
set_part {xc7z020clg400-1}
# M-KUBOS
#set_part {xczu19eg-ffvc1760-2-i}
create_clock -period 10 -name default
# Run C Simulation
# csim_design
csynth_design
export_design -rtl verilog -format ip_catalog -vendor xilinx.com -library hls -ipname [get_top] -version 1.0 -output ./vitis_hls/solution1
exit
