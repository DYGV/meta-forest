# meta-FOrEST
**meta-FOrEST** is a tool that allows automatic generation of multiple *ROS2-FPGA nodes* that are developed based on [FOrEST](https://github.com/ros2-forest/forest).

## Installation

```
pip3 install --upgrade pip
pip3 install git+https://github.com/DYGV/meta-forest
```

## Usage
meta-FOrEST has several subcommands. Run `meta-forest --help` to see available subcommands and their options.   

```
$ meta-forest --help
usage: meta-forest [-h] [-v] [-l STR] {gen_config,gen_block_design,gen_node} ...

positional arguments:
  {gen_config,gen_block_design,gen_node}
    gen_config          Generate a template config file to be used meta-FOrEST
    gen_block_design    Generate a Vivado block design according to the description
                        in config file
    gen_node            Generate ROS2-FPGA Nodes according to the description in
                        config file

options:
  -h, --help            show this help message and exit
  -v, --version         show program's version number and exit
  -l STR, --log_level STR
                        set logging level (debug|info|warn|error) (Default: info)
```

## Recommended Environment
WIP  

## Tutorial
WIP  

# Related Publications

- [1] - D. Pinheiro Leal, M. Sugaya, H. Amano, T. Ohkawa "Automated Integration of High-Level Synthesis FPGA Modules with ROS2 Systems", International Conference on Field Programmable Technology (FPT), 2020. 

- [2] - D. Pinheiro Leal, M. Sugaya, H. Amano, T. Ohkawa "FPGA Acceleration of ROS2-Based Reinforcement Learning Agents", CANDAR'20 - 8th International Workshop on Computer Systems and Architectures (CSA'20), 2020.

