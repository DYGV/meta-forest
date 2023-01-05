open_project -reset vitis_hls

add_files ./hls/add.c

set_top add

open_solution -reset solution1
# Set PYNQ-Z1 board part
set_part {xc7z020clg400-1}
create_clock -period 10 -name default
# Run C Simulation
# csim_design
csynth_design
exit
