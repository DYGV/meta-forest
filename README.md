# meta-FOrEST
<img src="https://user-images.githubusercontent.com/8480644/210734598-63b3e675-0ca1-4d08-b015-bb0e0ec21a56.png" width="60%" height="60%">  

**meta-FOrEST** is an automatic generation tool for ROS 2 nodes integrating multiple FPGA logic into a ROS 2 system (ROS2-FPGA nodes). FPGA and [ROS 2](https://docs.ros.org/en/humble/index.html) can be used to accelerate calculation processing and improve design productivity. meta-FOrEST (also [FOrEST](https://github.com/ros2-forest/forest)) supports the [PYNQ](http://www.pynq.io/) platform installed on a Zynq SoC. With ROS 2 nodes running on a Zynq SoC, network communication with other machines is possible, facilitating load balancing and parallel processing with FPGA. meta-FOrEST users need only High-Level Synthesis. meta-FOrEST automatically integrates the IP cores into the ROS 2 system. meta-FOrEST users can run ROS2-FPGA nodes without having to write any programs.  
Currently, meta-FOrEST has the following features:  
- [x] Multiple ROS2-FPGA Nodes Generation  
- [x] Automatic Generation of a Vivado Block Design with multiple IP cores

## Supported Environment
<details>
<summary>FPGA SoC Boards</summary>

These are tested platforms and may work on other Zynq SoC boards.  
- [x] [PYNQ-Z1](https://digilent.com/reference/programmable-logic/pynq-z1/start)
- [x] [M-KUBOS](https://www.paltek.co.jp/en/design/original/m-kubos/index.html)
- [x] [Kria KR260, KV260 (K26 SOM)](https://www.xilinx.com/products/som/kria/k26c-commercial.html)
</details>  

<details>
<summary>Operating System on FPGA SoC Board</summary>

- [x] [PYNQ v2.5](https://github.com/Xilinx/PYNQ/releases/tag/v2.5)
- [x] [PYNQ v3.0.0](https://github.com/Xilinx/PYNQ/releases/tag/v3.0.0)
</details>

<details>
<summary>ROS 2</summary>

- [x] [Eloquent](https://docs.ros.org/en/eloquent/index.html)
- [x] [Humble](https://docs.ros.org/en/humble/index.html)
</details> 

<details>
<summary>Vivado/Vitis HLS</summary>

- [x] 2022.1
</details>  
 
## Installation

```
git clone https://github.com/DYGV/meta-forest
cd meta-forest
pip3 install --upgrade pip
pip3 install .
```

## Usage
### Tutorials
You can learn how to use meta-FOrEST through examples.
- [simple-add](./examples/simple-add)
- [vector-add](./examples/vector-add)
- [Fast Fourier Transform](./examples/FFT)
 
### Caution
- The types available for meta-FOrEST are the same as for [FOrEST's type support](https://github.com/ros2-forest/forest/tree/master/docs/tutorials#type-support)  
- HLS data I/O must be implemented with AXI4 (M_AXI), AXI4-Lite or AXI4-Stream  
- Ensure that the status register is allocated with s_axilite as follows  
  ```c
  #pragma HLS INTERFACE s_axilite port = return
  ```
# Related Publications

- [1] - D. Pinheiro Leal, M. Sugaya, H. Amano, T. Ohkawa "Automated Integration of High-Level Synthesis FPGA Modules with ROS2 Systems", International Conference on Field Programmable Technology (FPT), 2020.  

- [2] - D. Pinheiro Leal, M. Sugaya, H. Amano, T. Ohkawa "FPGA Acceleration of ROS2-Based Reinforcement Learning Agents", CANDAR'20 - 8th International Workshop on Computer Systems and Architectures (CSA'20), 2020.  

