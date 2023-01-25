# FFT-ROS2-FPGA Nodes Example
## Overview
This is a meta-FOrEST sample using C++ code that processes FFT using FFT Library.  
The following environments were tested:
- [x] Vivado 2022.1
- [x] Vitis HLS 2022.1
- [x] PYNQ-Z1(xc7z020clg400-1)
  - OS: PYNQ v2.5
  - ROS2: eloquent
- [x] M-KUBOS(xczu19eg-ffvc1760-2-i)
  - OS: PYNQ v2.5
  - ROS2: eloquent 
## Goal of This Sample
The goal of this sample is to run **two ROS2-FPGA nodes** of 1024-point FFT processing on `PYNQ-Z1`.  

## Preparation
Let's prepare before using meta-FOrEST.  
To use meta-FOrEST, the code to be run as FPGA logic must have done `C Synthesis` and `export RTL`.  
**`In preparation.tcl`, the board parts of `PYNQ-Z1` are set. To target a different board part, change the argument of the `set_part` function in `preparation.tcl`.**  
HLS of C++ code with the following code:
```
vitis_hls preparation.tcl
```
When the process is finished, you will see a directory named `vitis_hls`.

## Make a Vivado Block Design with meta-FOrEST!
```
meta-forest gen_bd \
 --ip_directory ./vitis_hls/solution1  \
 --target_part xc7z020clg400-1 \
 --IP hls_fft \
 --count 2  \
 --step_to write_bitstream
 ```
 When the process is finished, you will see a directory named `vivado`.  
 <img src="./resources/vivado_bd.png" width="50%" height="50%">  
Let's transfer the created bitstream to the Zynq SoC with the following command.  
```
scp vivado/vivado.runs/impl_1/design_1_wrapper.bit xilinx@xxx.xxx.xxx.xxx:/home/xilinx/dev_ws/fft_2.bit
scp vivado/vivado.gen/sources_1/bd/design_1/hw_handoff/design_1.hwh  xilinx@xxx.xxx.xxx.xxx:/home/xilinx/dev_ws/fft_2.hwh
```  

## Generate ROS2 Packages for ROS2-FPGA Nodes
Finally, let's generate the package for the ROS2-FPGA nodes on your FPGA board.  

By executing the code below, the code will be generated in the ROS2 workspace as shown in the image.  

```
meta-forest gen_node \
--workspace /home/xilinx/dev_ws \
--package_name fft \
--bitstream /home/xilinx/dev_ws/fft_2.bit \
--test \
--IP hls_fft \
--count 2 \
--in stream_input --in_type float32[1024] \
--out stream_output --out_type float32[2048]
```  
<img src="./resources/dev_ws_tree.png" width="40%" height="40%">  

  
```
ros2 launch fft_fpga_node fpga_node_launch.py # as root
ros2 launch fft_fpga_node listener_launch.py
ros2 launch fft_fpga_node talker_launch.py
```  
It works!

- Left: Talker node feeding data into ROS2-FPGA node
- Center: ROS2-FPGA node performing FFT processing
- Right: Listener node receiving output from ROS2-FPGA node

<img src="./resources/fft_process.png" width="50%" height="50%">

