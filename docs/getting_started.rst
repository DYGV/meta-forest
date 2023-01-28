***************
Getting Started
***************

Environment
=====================================
These are tested platforms and may work on other versions.

* FPGA SoC Boards

  *  PYNQ-Z1
  *  M-KUBOS
  *  Kria KR260, KV260 (K26 SOM)

* Operating System on FPGA SoC Board

  * PYNQ v2.5
  * PYNQ v3.0.0

* ROS 2

  * eloquent
  * humble

* Vivado/Vitis HLS on PC

  * 2022.1


Installation
=====================================
.. code-block:: bash

    git clone https://github.com/DYGV/meta-forest
    cd meta-forest
    pip3 install --upgrade pip
    pip3 install .

Usage
=====================================
Help
------------------------------------
.. code-block:: text 

    meta-forest -h
    usage: meta-forest [-h] [-v] [-l STR] {gen_bd,gen_node} ...

    positional arguments:
      {gen_bd,gen_node}
        gen_bd              Generate a Vivado block design
        gen_node            Generate ROS2-FPGA Nodes

    options:
      -h, --help            show this help message and exit
      -v, --version         show program's version number and exit
      -l STR, --log_level STR
                            set logging level (debug|info|warn|error) (Default: info)

Vivado Block Design
------------------------------------
.. code-block:: text 

    meta-forest gen_bd -h 
    usage: meta-forest gen_bd [-h] -d STR -I STR -t STR -c INT [-s STR]

    options:
      -h, --help            show this help message and exit
      -d STR, --ip_directory STR
                            Exported IP Path
      -I STR, --IP STR      IP Core Name; It is usually the top function of HLS.
      -t STR, --target_part STR
                            Target FPGA Part
      -c INT, --count INT   Number of IP Cores to be Added to the Block Design
      -s STR, --step_to STR
                            Steps on Vivado

ROS2-FPGA Nodes
------------------------------------
.. code-block:: text 

    usage: meta-forest gen_node [-h] -p STR [-w STR] [-t] -b STR [-I STR] [-c INT] [-i STR] [-o STR] [--in_type STR]
                                [--out_type STR]

    options:
      -h, --help            show this help message and exit
      -p STR, --package_name_prefix STR
                            ROS2 Package Name Prefix
      -w STR, --workspace STR
                            ROS2 Workspace Directory
      -t, --test            Generate Talker/Listener Nodes
      -b STR, --bitstream STR
                            Bitstream Path
      -I STR, --IP STR      IP Core Name; It is usually the top function of HLS.
      -c INT, --count INT   Number of IP Cores Used in The Generated ROS2-FPGA Nodes
      -i STR, --in STR      Input variable names to be used in ROS2 custom messages. Must match the variable name in the data
                            input of the top function argument.
      -o STR, --out STR     Output variable names to be used in ROS2 custom messages. Must match the variable name in the data
                            output of the top function argument.
      --in_type STR         Type as input to be used in ROS2 custom messages(e.g. int32[1024])
      --out_type STR        Type as output to be used in ROS2 custom messages; It must always be an array(e.g. int32[1])
