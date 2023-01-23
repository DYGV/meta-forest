proc create_vivado_project {project_name project_directory} {
    cd $project_directory
    create_project -force $project_name [append $project_directory $project_name]
}

proc setup_vivado_project {device_part ip_directories bd_name} {
    # Setup FPGA to be set up in a project and creating a block design file

    # Configure board parts for the project
    set board_part [get_board_parts -quiet -latest_file_version "*$device_part*"]
    if { [string length $board_part] != 0 } {
        set_property board_part $board_part [current_project]
    } else {
        set_property part $device_part [current_project]
    }
    # Setting related to the Vivado project
    set_property target_language "Verilog" [current_project]
    set_property ip_repo_paths $ip_directories [current_fileset]
    update_ip_catalog
    foreach ip_directory $ip_directories {
        set ip_zip [glob [file join $ip_directory/*.zip]]
        foreach ip $ip_zip {
            update_ip_catalog -add_ip $ip -repo_path $ip_directory
        }
    }
    if { [string length [get_filesets -quiet sources_1]] == 0 } {
        create_fileset -srcset sources_1
    }
    create_bd_design -verbose $bd_name
    update_compile_order -fileset sources_1
}

proc get_intf {ip_name protocol {master_slave "*"}} {
    # Return an interface object from a given IP name and protocol

    set filter "MODE =~ $master_slave && VLNV =~ xilinx.com:interface:$protocol*"
    lappend pins [get_bd_intf_pins -quiet \
                    -filter $filter \
                    -of [get_bd_cells $ip_name]]
    return $pins
}

proc configure_axi_dma_ip { dma_ip user_ip_axis_intr_pin_list } {
    # Setup DMA circuit
    # Turn off mm2s and s2mm in the DMA circuit according to user IP

    set user_ip_slave [filter $user_ip_axis_intr_pin_list {MODE == "Slave"}]
    set user_ip_master [filter $user_ip_axis_intr_pin_list {MODE == "Master"}]
    set include_mm2s [expr {[llength $user_ip_slave] > 0}]
    set include_s2mm [expr {[llength $user_ip_master] > 0}]

    set config [ list \
                CONFIG.c_include_mm2s $include_mm2s \
                CONFIG.c_include_s2mm $include_s2mm \
                CONFIG.c_include_sg 0 \
                CONFIG.c_sg_include_stscntrl_strm 0 \
                CONFIG.c_sg_length_width 26 \
    ]

    set_property -dict $config $dma_ip

    # Match the data width of s2mm and mm2s to the data width of the user IP to be connected
    if { $include_s2mm } {
        set user_ip_master_data_size [expr {[get_property CONFIG.TDATA_NUM_BYTES $user_ip_master] * 8}]
        set_property CONFIG.c_m_axi_s2mm_data_width $user_ip_master_data_size $dma_ip
        set_property CONFIG.c_s_axis_s2mm_tdata_width $user_ip_master_data_size $dma_ip
    }
    if { $include_mm2s } {
        set user_ip_slave_data_size [expr {[get_property CONFIG.TDATA_NUM_BYTES $user_ip_slave] * 8}]
        set_property CONFIG.c_m_axi_mm2s_data_width $user_ip_slave_data_size $dma_ip
        set_property CONFIG.c_m_axis_mm2s_tdata_width $user_ip_slave_data_size $dma_ip
    }

}

proc connect_master_to_slave_axis { dma_ip user_ip_axis_intr_pin_list } {
    # Connect the AXIS pins of the user logic to the AXIS pins of the DMA circuit

    set dma_ip_intr_pin_list [get_intf [get_property NAME $dma_ip] "axis"]

    foreach user_pin $user_ip_axis_intr_pin_list {
        set user_pin_mode [get_property MODE $user_pin]
        foreach dma_pin $dma_ip_intr_pin_list {
            set dma_pin_mode [get_property MODE $dma_pin]
            if { ![string equal $user_pin_mode $dma_pin_mode] } {
                connect_bd_intf_net $dma_pin $user_pin
            }
        }
    }
}

proc add_vivado_bd_ip_axi_interconnect { name_suffix } {
    set interconnect_vlnv [lindex [get_ipdefs -all xilinx.com:ip:axi_interconnect:* -filter {UPGRADE_VERSIONS == ""} ] 0]
    set interconnect_ip [create_bd_cell -type ip -vlnv $interconnect_vlnv "axi_interconnect_$name_suffix"]
    set config [ list CONFIG.NUM_SI 1 CONFIG.NUM_MI 1 ]
    set_property -dict $config $interconnect_ip
    return $interconnect_ip
}

proc add_vivado_bd_ip_axi_dma { axis_intf_pin_list } {
    set ip_list_len [llength [get_bd_cells -quiet -patterns "*axi_dma_*" ]]
    set dma_vlnv [lindex [get_ipdefs -all xilinx.com:ip:axi_dma:* -filter {UPGRADE_VERSIONS == ""} ] 0]
    set dma [create_bd_cell -type ip -vlnv $dma_vlnv "axi_dma_$ip_list_len"]
    configure_axi_dma_ip $dma $axis_intf_pin_list
    connect_master_to_slave_axis $dma $axis_intf_pin_list
    return $dma
}

proc add_vivado_bd_ip { ip_name number {library "hls"} {version 1.0} } {
    # Add the IP specified by ip_name to the block design
    # Add DMA circuit as needed

    set port [get_ps_port]
    set gp [dict get $port M_AXI_GP]
    for { set i 0 } { $i < $number } { incr i } {
        set ip_list_len [llength [get_bd_cells -quiet -patterns "*${ip_name}_*" ]]
        set added_ip [create_bd_cell -type ip -vlnv xilinx.com:$library:$ip_name:$version ${ip_name}_${ip_list_len}]
        set axis_intf_pin_list [get_intf ${ip_name}_${ip_list_len} "axis"]
        # Add DMA circuit if AXIS is included in the I/O of the user circuit
        if { [llength $axis_intf_pin_list] > 0 } {
            open_ps_hp
            add_vivado_bd_ip_axi_dma $axis_intf_pin_list
        }
    }
    if { [info exists added_ip ] } {
        return $added_ip
    }
}

proc open_ps_hp { } {
    # Open the High Performance (HP) port of the Processing System

    global PS

    set board_architecture [get_property architecture [get_property part [current_project]]]
    # Determine the name of the HP property from the board architecture
    if { [string equal "zynq" $board_architecture] } {
        set HP "CONFIG.PCW_USE_S_AXI_HP0"
    } elseif { [string equal "zynquplus" $board_architecture] } {
        set HP "CONFIG.PSU__USE__S_AXI_GP2"
        set_property "CONFIG.PSU__MAXIGP2__DATA_WIDTH" 128 $PS
    } else {
        return
    }

    set_property $HP 1 $PS
}

proc get_ps_port {} {
    # Get the pins of the HP/GP port of the Processing System

    global PS
    lappend pins [get_bd_intf_pins -filter {VLNV =~ "xilinx.com:interface:aximm*"} -of $PS]
    set S_AXI_HP [filter $pins {MODE =~ "Slave"}]
    set M_AXI_GP [filter $pins {MODE =~ "Master"}]
    set PS_PORT [dict create S_AXI_HP $S_AXI_HP M_AXI_GP $M_AXI_GP ]
    return $PS_PORT
}

proc add_vivado_bd_ip_ps { } {
    # Add Processing System IP core to the block design

    set board_ps_defs {"zynq" {"processing_system7"}
                       "zynquplus" {"zynq_ultra_ps_e"}
    }
    set board_architecture [get_property architecture [get_property part [current_project]]]
    set board [dict get $board_ps_defs $board_architecture]
    set ps_name [lindex $board 0]

    set ps_vlnv [lindex [get_ipdefs -all xilinx.com:ip:${ps_name}:* -filter {UPGRADE_VERSIONS == ""} ] 0]
    set ps [create_bd_cell -type ip -vlnv ${ps_vlnv} ${ps_name}_0]

    if { [string equal "zynq" $board_architecture]} {
        set config [list \
                    make_external "FIXED_IO, DDR" \
                    apply_board_preset "1" \
                    Master "Disable" \
                    Slave "Disable" \
                    ]
    } elseif { [string match "zynquplus" $board_architecture] } {
        set config [list apply_board_preset "1"]
    }
    apply_bd_automation -rule xilinx.com:bd_rule:$ps_name -config $config $ps


    if { [string match "zynquplus" $board_architecture] } {
        set_property CONFIG.PSU__USE__M_AXI_GP1 0 $ps
    }
    return $ps
}

proc incr_NUM_SI { interconnect_ip } {
    set NUM_SI [get_property CONFIG.NUM_SI $interconnect_ip]

    if { $NUM_SI < 16 } {
        set_property -dict [list \
            CONFIG.NUM_MI 1 \
            CONFIG.NUM_SI [expr {$NUM_SI + 1}]\
        ] $interconnect_ip
        return 1
    }
    return 0
}

proc incr_NUM_MI { interconnect_ip } {
    set NUM_MI [get_property CONFIG.NUM_MI $interconnect_ip]

    if { $NUM_MI < 16 } {
        set_property -dict [list \
            CONFIG.NUM_MI [expr {$NUM_MI + 1}] \
            CONFIG.NUM_SI 1 \
        ] $interconnect_ip
        return 1
    }
    return 0
}

proc connect_axi_interconnect_gp_to_slave { } {
    # Connect GP ports and IP core slaves with AXI-Interconnect

    set s_aximm_pins [get_bd_intf_pins \
            -of_objects [lsort -dictionary \
                [get_bd_cells -quiet -filter { VLNV =~ "*:hls:*:*" || VLNV =~ "*:*:axi_dma:*"}]] \
            -filter {MODE == Slave && CONFIG.Protocol == AXI4LITE}]

    set s_aximm_pins_len [llength $s_aximm_pins]
    if { $s_aximm_pins_len == 0} {
        return
    }

    lappend interconnect_ips [add_vivado_bd_ip_axi_interconnect "gp_0"]
    connect_bd_intf_net \
        [dict get [get_ps_port] M_AXI_GP] \
        [get_bd_intf_pins -of_objects [lindex $interconnect_ips 0] -filter {MODE == Slave}]

    # Find the number of additional AXI-Interconnects needed
    if { $s_aximm_pins_len > 16 } {
        set num [expr {[llength $s_aximm_pins] / 16}]
    } else {
        set num 0
    }

    # Interconnect to Interconnect if necessary
    for {set i 0} {$i < $num} {incr i} {
        lappend interconnect_ips [add_vivado_bd_ip_axi_interconnect "gp_[llength $interconnect_ips]"]
        connect_bd_intf_net \
            [lindex [get_intf [lindex $interconnect_ips 0] "aximm" "Master"] end] \
            [lindex [get_intf [lindex $interconnect_ips end] "aximm" "Slave"] end]
        incr_NUM_MI [lindex $interconnect_ips 0]
    }

    # Connect a free Interconnect port and user IP port
    set s_aximm_pin_index 0
    set target_interconnect_ip_index 0
    while { $s_aximm_pin_index < $s_aximm_pins_len} {
        set target_slave_pin [lindex $s_aximm_pins $s_aximm_pin_index]
        set target_interconnect_ip [lindex $interconnect_ips $target_interconnect_ip_index]
        set target_interconnect_master_pin [lindex [get_intf $target_interconnect_ip "aximm" "Master"] end]
        # If $target_interconnect_master_pin is already connected to another IP core, change the interconnect to which it is connected
        if { [llength [get_bd_intf_nets -quiet -of_objects $target_interconnect_master_pin]] > 0 } {
            set target_interconnect_ip_index [expr { $target_interconnect_ip_index + 1 }]
            continue
        }

        puts "Processing ${target_slave_pin} connection"
        connect_bd_intf_net $target_interconnect_master_pin $target_slave_pin
        set s_aximm_pin_index [expr { $s_aximm_pin_index + 1 }]
        if { $s_aximm_pin_index != $s_aximm_pins_len && ![incr_NUM_MI $target_interconnect_ip ]} {
            set target_interconnect_ip_index [expr { $target_interconnect_ip_index + 1 }]
        }
    }
}

proc connect_axi_interconnect_hp_to_master { } {
    # Connect HP port and IP core master with AXI-Interconnect

    set m_axi_pins [get_bd_intf_pins -quiet \
                    -of_objects [lsort -dictionary [get_bd_cells -quiet -filter { VLNV =~ "*:*:axi_dma:*"}]] \
                    -filter {MODE == Master && CONFIG.Protocol == AXI4}]
    set m_axi_pins_len [llength $m_axi_pins]
    if { $m_axi_pins_len == 0} {
        return
    }

    lappend interconnect_ips [add_vivado_bd_ip_axi_interconnect "hp_0"]
    connect_bd_intf_net \
        [get_bd_intf_pins -of_objects [lindex $interconnect_ips 0] -filter {MODE == Master}] \
        [dict get [get_ps_port] S_AXI_HP]

    # Find the number of additional AXI-Interconnects needed
    if { $m_axi_pins_len > 16 } {
        set num [expr {[llength $m_axi_pins] / 16}]
    } else {
        set num 0
    }
    # Interconnect to Interconnect if necessary
    for {set i 0} {$i < $num} {incr i} {
        lappend interconnect_ips [add_vivado_bd_ip_axi_interconnect "hp_[llength $interconnect_ips]"]
        connect_bd_intf_net \
            [lindex [get_intf [lindex $interconnect_ips end] "aximm" "Master"] end] \
            [lindex [get_intf [lindex $interconnect_ips 0] "aximm" "Slave"] end]
        incr_NUM_SI [lindex $interconnect_ips 0]
    }
    # Connect a free Interconnect port and user IP port
    set m_axi_pin_index 0
    set target_interconnect_ip_index 0
    while { $m_axi_pin_index < $m_axi_pins_len} {
        set target_master_pin [lindex $m_axi_pins $m_axi_pin_index]
        set target_interconnect_ip [lindex $interconnect_ips $target_interconnect_ip_index]
        set target_interconnect_ip_slave_pin [lindex [get_intf $target_interconnect_ip "aximm" "Slave"] end]

        # Change the interconnect to which it is connected if $target_interconnect_slave_pin is already connected to another IP core
        if { [llength [get_bd_intf_nets -quiet -of_objects $target_interconnect_ip_slave_pin]] > 0 } {
            set target_interconnect_ip_index [expr { $target_interconnect_ip_index + 1 }]
            continue
        }

        puts "Processing ${target_master_pin} connection"
        connect_bd_intf_net $target_interconnect_ip_slave_pin $target_master_pin
        set m_axi_pin_index [expr { $m_axi_pin_index + 1 }]
        if { $m_axi_pin_index != $m_axi_pins_len && ![incr_NUM_SI $target_interconnect_ip ]} {
            set target_interconnect_ip_index [expr { $target_interconnect_ip_index + 1 }]
        }
    }
    # Avoid ID_WIDTH mismatch with HP port
    set_property CONFIG.STRATEGY 1 [lindex $interconnect_ips 0]
}

proc connect_clk { } {
    set clk_in_list [get_bd_pins -filter {DIR == I && TYPE == clk } -of_objects [get_bd_cells]]
    set clk_out [get_bd_pins -filter {DIR == O && TYPE == clk} -of_objects [get_bd_cells]]
    foreach clk_in $clk_in_list {
        connect_bd_net $clk_out $clk_in
    }
}

proc connect_rst { } {
    set rst_ex_out [get_bd_pins -quiet -filter {DIR == O && TYPE == rst} -of_objects [get_bd_cells -quiet -filter {NAME !~ "*axi_dma*" && NAME !~ "*sys_rst*"}]]
    set rst_ex_in [get_bd_pins /sys_rst_0/ext_reset_in]
    connect_bd_net $rst_ex_out $rst_ex_in

    set rst_in_list [get_bd_pins -quiet -filter {DIR == I && TYPE == rst} -of_objects [get_bd_cells -quiet -filter {NAME !~ "*sys_rst*"}]]
    set rst_out [get_bd_pins /sys_rst_0/peripheral_aresetn]
    foreach rst_in $rst_in_list {
        connect_bd_net $rst_out $rst_in
    }
}

proc connect { } {
    # Add "Processor System Reset"
    create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 "sys_rst_0"
    # Connect between the master side of AXI-Interconnect leading to the GP port and the slave side of the user IP or DMA circuit
    connect_axi_interconnect_gp_to_slave
    # Connect between the slave side of the AXI-Interconnect leading to the HP port and the master side of the DMA circuit
    connect_axi_interconnect_hp_to_master
    # Connect clock signals of added IP cores
    connect_clk
    # Connect reset signals of added IP cores
    connect_rst
}

# Vivado project name (required arg)
set project_name ""
# Board parts to be applied to the project (required arg)
set device_part ""
# Absolute path where the user IP is located (required arg)
set ip_directories ""
# Name of block design file (default: "design_1")
set bd_file_name "design_1"
# Whether to automatically connect IP cores to each other (default: false)
set auto_connect 0
# Whether to create a bitstream (default: false)
set write_bitstream 0
# Whether to launch the Vivado GUI (default: false)
set start_gui 0

set seeking_index 0

while { $seeking_index < $argc } {
    set option_flag [lindex $argv $seeking_index]
    incr seeking_index

    switch $option_flag {
        -project_name {
            set project_name [lindex $argv $seeking_index]
            incr seeking_index
        }
        -device_part {
            set device_part [lindex $argv $seeking_index]
            incr seeking_index
        }
        -bd_file_name {
            set bd_file_name [lindex $argv $seeking_index]
            incr seeking_index
        }
        -ip_directories {
            set ip_directories [lindex $argv $seeking_index]
            incr seeking_index
        }
        -auto_connect {
            set auto_connect [lindex $argv $seeking_index]
            incr seeking_index
        }
        -write_bitstream {
            set write_bitstream [lindex $argv $seeking_index]
            incr seeking_index
        }
        -start_gui {
            set start_gui [lindex $argv $seeking_index]
            incr seeking_index
        }
        -ip {
            # Name and number of IP cores to be added to the block design
            set ip_name [lindex $argv $seeking_index]
            incr seeking_index
            set ip_count [lindex $argv $seeking_index]
            incr seeking_index
            dict set ips $ip_name $ip_count
        }
        default {
            puts "ERROR option '$option_flag' is not a valid option."
            exit
        }
    }
}

if { $project_name eq "" } {
    puts "No project name was specified for vivado project."
    exit
}

if { $device_part eq "" } {
    puts "No device part name was specified."
    exit
}

if { $ip_directories eq "" } {
    puts "No directory where the user IP is located was specified."
    exit
}

if { ![info exists ips] } {
    exit
}

create_vivado_project $project_name [pwd]
setup_vivado_project $device_part $ip_directories $bd_file_name

set PS [add_vivado_bd_ip_ps]
dict for {ip_name ip_count} $ips {
    add_vivado_bd_ip $ip_name $ip_count
}

if { $auto_connect } {
    connect
    if { $write_bitstream } {
        assign_bd_address -force
        make_wrapper -force -files [get_files "$bd_file_name.bd"] -top -import
        launch_runs synth_1 -jobs [expr { [get_param general.MaxThreads] - 1 }]
        wait_on_run synth_1
        launch_runs impl_1 -jobs [expr { [get_param general.MaxThreads] - 1 }] -to_step write_bitstream
        wait_on_run impl_1
    }
}

save_bd_design

if { $start_gui } {
    start_gui
}
