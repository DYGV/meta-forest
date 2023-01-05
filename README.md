# meta-FOrEST
<img src="https://user-images.githubusercontent.com/8480644/210734598-63b3e675-0ca1-4d08-b015-bb0e0ec21a56.png" width="60%" height="60%">  

**meta-FOrEST** is an automatic generation tool for multiple ROS2-FPGA nodes. meta-FOrEST is developed based on [FOrEST](https://github.com/ros2-forest/forest).  
Currently, meta-FOrEST has the following features:  
- [x] Multiple ROS2-FPGA Nodes Generation  
- [x] Automatic Generation of a Vivado Block Design with multiple IP cores from a Vitis HLS project  
- [x] Semi-Automatic Configuration file description (Advanced version of a file like FOrEST's config.forset)


## Installation

```
pip3 install --upgrade pip
pip3 install git+https://github.com/DYGV/meta-forest
```

## Usage
### Tutorials
You can learn how to use meta-FOrEST through examples.
- Simple Tutorial:
  - [simple-add](./examples/simple-add) (WIP)
  - [vector-add-lite](./examples/vector-add-lite) (WIP)
- Advanced Tutorial: [Fast Fourier Transform](./examples/FFT)
 
### Caution
The types available for meta-FOrEST are the same as for [FOrEST's type support](https://github.com/ros2-forest/forest/tree/master/docs/tutorials#type-support).  
# Related Publications

- [1] - D. Pinheiro Leal, M. Sugaya, H. Amano, T. Ohkawa "Automated Integration of High-Level Synthesis FPGA Modules with ROS2 Systems", International Conference on Field Programmable Technology (FPT), 2020. 

- [2] - D. Pinheiro Leal, M. Sugaya, H. Amano, T. Ohkawa "FPGA Acceleration of ROS2-Based Reinforcement Learning Agents", CANDAR'20 - 8th International Workshop on Computer Systems and Architectures (CSA'20), 2020.


