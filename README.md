# meta-FOrEST
<img src="https://user-images.githubusercontent.com/8480644/210734598-63b3e675-0ca1-4d08-b015-bb0e0ec21a56.png" width="60%" height="60%">  

**meta-FOrEST** is an automatic generation tool for multiple ROS2-FPGA nodes. meta-FOrEST is developed based on [FOrEST](https://github.com/ros2-forest/forest). FPGA and [ROS2](https://docs.ros.org/en/humble/index.html) can be used to accelerate calculation processing and improve design productivity. meta-FOrEST (also FOrEST) supports the [PYNQ](http://www.pynq.io/) platform installed on a Zynq SoC.  
Currently, meta-FOrEST has the following features:  
- [x] Multiple ROS2-FPGA Nodes Generation  
- [x] Automatic Generation of a Vivado Block Design with multiple IP cores

## Supported Environment
<details>
<summary>FPGA SoC Boards</summary>

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
- [Fast Fourier Transform](./examples/FFT)
 
### Caution
The types available for meta-FOrEST are the same as for [FOrEST's type support](https://github.com/ros2-forest/forest/tree/master/docs/tutorials#type-support).  
# Related Publications

- [1] - D. Pinheiro Leal, M. Sugaya, H. Amano, T. Ohkawa "Automated Integration of High-Level Synthesis FPGA Modules with ROS2 Systems", International Conference on Field Programmable Technology (FPT), 2020. 

- [2] - D. Pinheiro Leal, M. Sugaya, H. Amano, T. Ohkawa "FPGA Acceleration of ROS2-Based Reinforcement Learning Agents", CANDAR'20 - 8th International Workshop on Computer Systems and Architectures (CSA'20), 2020.


