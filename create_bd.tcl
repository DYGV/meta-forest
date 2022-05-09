proc create_vivado_project {project_name project_directory} {
    # プロジェクトの作成

    cd $project_directory
    create_project -force $project_name [append $project_directory $project_name]
}

proc setup_vivado_project {board_name ip_directory bd_name} {
    # プロジェクトに設定するFPGAの設定とブロックデザインファイルの作成

    # Vivado上に取り込んであるボードファイルの情報を取得
    set board_part [get_board_parts -quiet -latest_file_version "*$board_name*"]
    if { [string equal board_part ""] } {
        puts [concat Error! No matching board file found for $board_name]
        exit
    }
    # プロジェクトにボード情報を設定する
    set_property board_part $board_part [current_project]
    set_property target_language "Verilog" [current_project]

    # IPをプロジェクトのIP Catalogに追加する
    lappend ip_list [file join $ip_directory]
    if {[info exists ip_list] && [llength $ip_list] > 0} {
        set_property ip_repo_paths $ip_list [current_fileset]
    }

    update_ip_catalog
    create_bd_design -verbose $bd_name
    update_compile_order -fileset sources_1
}

proc get_intf {ip_name protocol {master_slave "*"}} {
    # 指定されたIP名とプロトコルからインターフェースオブジェクトを返す

    lappend pins [get_bd_intf_pins -of [get_bd_cells $ip_name]]
    foreach pin $pins {
        set intf_vlnv [get_property VLNV $pin]
        set mode [get_property MODE $pin]
        # IPが$protocolを使った入出力を含んでいるかを調べる
        if { [string match "xilinx.com:interface:$protocol*" $intf_vlnv] } {
            if { [string equal "*" $master_slave] } {
                lappend intf_pin_list $pin
            } elseif { [string equal "Master" $master_slave] } {
                if { [string equal "Master" $mode]} {
                    lappend intf_pin_list $pin
                }
            } elseif { [string equal "Slave" $master_slave] } {
                if { [string equal "Slave" $mode]} {
                    lappend intf_pin_list $pin
                }
            }
        }
    }
    if { [info exists intf_pin_list] } {
        return $intf_pin_list
    }
}

proc configure_axi_dma_ip { dma_ip user_ip_axis_intr_pin_list } {
    # DMA IPの設定
    # ユーザIPに応じてmm2s、s2mmをオフにする

    set mm2s 0
    set s2mm 0
    foreach pin $user_ip_axis_intr_pin_list {
        # MasterかSlaveか
        set mode [get_property MODE $pin]
        set mm2s [expr {$mm2s || [string equal $mode "Slave" ]}]
        set s2mm [expr {$s2mm || [string equal $mode "Master" ]}]
    }
    if { ![expr {$mm2s || $s2mm}] } {
        return
    }
    set config [ list \
                CONFIG.c_include_mm2s $mm2s \
                CONFIG.c_include_s2mm $s2mm \
                CONFIG.c_include_sg 0 \
                CONFIG.c_sg_include_stscntrl_strm 0 \
                CONFIG.c_sg_length_width 26 \
    ]

    set_property -dict $config $dma_ip
}

proc connect_master_to_slave_axis { dma_ip user_ip_axis_intr_pin_list } {
    # ユーザロジックのAXISとDMA回路をつなぐ

    # 1. DMAのaxisのインターフェースピンをもらう
    # 2. DMAのaxisのmasterとuserのslaveを結ぶ(v.v.)
    
    set dma_ip_intr_pin_list [get_intf [get_property NAME $dma_ip] "axis"]

    foreach user_pin $user_ip_axis_intr_pin_list {
        set user_pin_mode [get_property MODE $user_pin]
        foreach dma_pin $dma_ip_intr_pin_list {
            set dma_pin_mode [get_property MODE $dma_pin]
            if { ![string equal $user_pin_mode $dma_pin_mode] } {
                connect_bd_intf_net [get_bd_intf_pins $dma_pin] [get_bd_intf_pins $user_pin]
            }
        }
    }
}


proc add_vivado_bd_ip_axi_intc { name_suffix } {
    set intc_ip [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 "axi_interconnect_$name_suffix"]
    set config [ list CONFIG.NUM_SI 1 CONFIG.NUM_MI 1 ]
    set_property -dict $config $intc_ip
    return $intc_ip
}

proc add_vivado_bd_ip_axi_dma { axis_intf_pin_list } {
    set ip_list_len [llength [get_bd_cells -quiet -patterns "*axi_dma_*" ]]
    set dma [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma:7.1 "axi_dma_$ip_list_len"]
    configure_axi_dma_ip $dma $axis_intf_pin_list
    connect_master_to_slave_axis $dma $axis_intf_pin_list
    return $dma
}

proc add_vivado_bd_ip { ip_name number {library "hls"} {version 1.0} } {
    # ip_nameに指定されてたIPをブロックデザインに追加する
    # 必要に応じてDMA回路も追加する

    set port [get_ps_port]
    set gp [dict get $port M_AXI_GP]
    for { set i 0 } { $i < $number } { incr i } {
        set ip_list_len [llength [get_bd_cells -quiet -patterns "*${ip_name}_*" ]]
        # startgroup
        set added_ip [create_bd_cell -type ip -vlnv xilinx.com:$library:$ip_name:$version ${ip_name}_${ip_list_len}]
        set axis_intf_pin_list [get_intf ${ip_name}_${ip_list_len} "axis"]
        # 入出力にAXISを含むならDMA回路を追加する
        if { [llength $axis_intf_pin_list] > 0 } {
            open_ps_hp
            add_vivado_bd_ip_axi_dma $axis_intf_pin_list
        }
        # endgroup
    }
    if { [info exists added_ip ] } {
        return $added_ip
    }
}

proc open_ps_hp { {port_num 0} } {
    # HPポートをオンにする

    global PS
    # すでに開いているなら処理を終える
    if { [get_property CONFIG.PCW_USE_S_AXI_HP$port_num $PS] } {
        return
    }
    # プロセッサのHPポートをオンにする
    set_property CONFIG.PCW_USE_S_AXI_HP$port_num 1 $PS
}

proc get_ps_port {} {
    global PS
    lappend pins [get_bd_intf_pins -of $PS]
    set PS_PORT [dict create]
    foreach pin $pins {
        set intf_vlnv [get_property VLNV $pin]
        set mode [get_property MODE $pin]
        if { [string match "xilinx.com:interface:aximm*" $intf_vlnv] } {
            if { [string equal $mode "Slave"] } {
                dict set PS_PORT S_AXI_HP $pin
            } elseif { [string equal $mode "Master"] } {
                dict set PS_PORT M_AXI_GP $pin
            }
        }
    }
    return $PS_PORT
}

proc add_vivado_bd_ip_ps { board_name } {
    # ブロックデザインにプロセッサIPを追加してHPポートを開く

    set board_ps_defs {"zynq" {"processing_system7" "5.5"}
                       "zynqplus" {"zynq_ultra_ps_e" "3.3"}
    }
    set board [dict get $board_ps_defs [get_property architecture [get_parts [get_property PART_NAME [current_board_part]]]]]
    set ps_name [lindex $board 0]
    set ps_version [lindex $board 1]

    set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:$ps_name:$ps_version ${ps_name}_0]
    # 周辺回路、DDRメモリをつなぐ
    apply_bd_automation -rule \
        xilinx.com:bd_rule:[lindex $board 0] \
        -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable" } $ps
    return $ps
}

proc incr_NUM_SI { intc_ip } {
    set NUM_SI [get_property CONFIG.NUM_SI $intc_ip]

    if { $NUM_SI < 16 } {
        # Slaveを16まで増やすにはMasterが1である必要がある
        set_property CONFIG.NUM_MI 1 $intc_ip
        set_property CONFIG.NUM_SI [expr {$NUM_SI + 1}] $intc_ip
        return 1
    }
    return 0
}

proc incr_NUM_MI { intc_ip } {
    set NUM_MI [get_property CONFIG.NUM_MI $intc_ip]

    if { $NUM_MI < 64 } {
        set_property CONFIG.NUM_MI [expr {$NUM_MI + 1}] $intc_ip
        # Masterを64まで増やすにはSlaveが1である必要がある
        set_property CONFIG.NUM_SI 1 $intc_ip
        return 1
    }
    return 0
}

proc connect_axi_intc_gp_to_slave { } {
    # GPポートとIPコアのスレーブをAXI-Interconnectで繋ぐ

    set user_ips [lsort -dictionary [get_bd_cells -filter { VLNV =~ "*:hls:*:*" || VLNV =~ "*:*:axi_dma:*"}]]
    if { [llength $user_ips] == 0} {
        return
    }
    set s_aximm_pins [get_bd_intf_pins -of_objects $user_ips -filter {MODE == Slave && CONFIG.Protocol == AXI4LITE}]
    set s_aximm_pins_len [llength $s_aximm_pins]

    lappend intc_ips [add_vivado_bd_ip_axi_intc "gp_0"]

    # GPポート(マスタ)とインターコネクト(スレーブ)を繋ぐ
    set intc_ip [lindex $intc_ips 0]
    set intc_ip_slave_pin [get_bd_intf_pins -of_objects $intc_ip -filter {MODE == Slave}]
    set ps_port [get_ps_port]
    set master [dict get $ps_port M_AXI_GP]
    connect_bd_intf_net $master $intc_ip_slave_pin

    # 追加で必要なAXI-Interconnectの個数
    set num [expr {[llength $s_aximm_pins] / 64}]
    # 必要であればInterconnectにInterconnectを接続する
    for {set i 0} {$i < $num} {incr i} {
        set len_axi_interconnect [llength $intc_ips]
        lappend intc_ips [add_vivado_bd_ip_axi_intc "gp_${len_axi_interconnect}"]
        set intc_ip_master [lindex $intc_ips 0]
        set intc_ip_slave [lindex $intc_ips end]

        set intc_ip_master_pin [lindex [get_intf $intc_ip_master "aximm" "Master"] end]
        set intc_ip_slave_pin [lindex [get_intf $intc_ip_slave "aximm" "Slave"] end]

        connect_bd_intf_net $intc_ip_master_pin $intc_ip_slave_pin

        incr_NUM_MI [lindex $intc_ips 0]
    }
    # 空いているInterconnectのポートとユーザIPを接続する
    set s_aximm_pin_index 0
    set target_intc_ip_index 0
    while { $s_aximm_pin_index < $s_aximm_pins_len} {
        set target_pin [lindex $s_aximm_pins $s_aximm_pin_index]
        set target_intc_ip [lindex $intc_ips $target_intc_ip_index]
        set intc_ip_master_pin [lindex [get_intf $target_intc_ip "aximm" "Master"] end]

        puts "Processing ${target_pin} connection"
        connect_bd_intf_net $intc_ip_master_pin $target_pin
        set s_aximm_pin_index [expr { $s_aximm_pin_index + 1 }]
        if { $s_aximm_pin_index != $s_aximm_pins_len && ![incr_NUM_MI $target_intc_ip ]} {
            set target_intc_ip_index [expr { $target_intc_ip_index + 1 }]
        }
    }
}

proc connect_master_to_axi_intc_hp { } {
    # HPポートとIPコアのマスタをAXI-Interconnectで繋ぐ

    set dma_ips [lsort -dictionary [get_bd_cells -filter { VLNV =~ "*:*:axi_dma:*"}]]
    if { [llength $dma_ips] == 0} {
        return
    }
    set m_axi_pins [get_bd_intf_pins -of_objects $dma_ips -filter {MODE == Master && CONFIG.Protocol == AXI4}]
    set m_axi_pins_len [llength $m_axi_pins]

    lappend intc_ips [add_vivado_bd_ip_axi_intc "hp_0"]
    set intc_ip [lindex $intc_ips 0]
    set intc_ip_master_pin [get_bd_intf_pins -of_objects $intc_ip -filter {MODE == Master}]

    set ps_port [get_ps_port]
    set slave [dict get $ps_port S_AXI_HP]
    connect_bd_intf_net $intc_ip_master_pin $slave

    # 追加で必要なAXI-Interconnectの個数
    set num [expr {[llength $m_axi_pins] / 16}]
    # 必要であればInterconnectにInterconnectを接続する
    for {set i 0} {$i < $num} {incr i} {
        set len_axi_interconnect [llength $intc_ips]
        lappend intc_ips [add_vivado_bd_ip_axi_intc "hp_${len_axi_interconnect}"]
        set intc_ip_slave [lindex $intc_ips 0]
        set intc_ip_master [lindex $intc_ips end]

        set intc_ip_master_pin [lindex [get_intf $intc_ip_master "aximm" "Master"] end]
        set intc_ip_slave_pin [lindex [get_intf $intc_ip_slave "aximm" "Slave"] end]

        connect_bd_intf_net $intc_ip_master_pin $intc_ip_slave_pin
        incr_NUM_SI [lindex $intc_ips 0]
    }
    # 空いているInterconnectのポートとユーザIPを接続する
    set m_axi_pin_index 0
    set target_intc_ip_index 0
    while { $m_axi_pin_index < $m_axi_pins_len} {
        set target_pin [lindex $m_axi_pins $m_axi_pin_index]
        set target_intc_ip [lindex $intc_ips $target_intc_ip_index]
        set intc_ip_slave_pin [lindex [get_intf $target_intc_ip "aximm" "Slave"] end]

        puts "Processing ${target_pin} connection"
        connect_bd_intf_net $intc_ip_slave_pin $target_pin
        set m_axi_pin_index [expr { $m_axi_pin_index + 1 }]
        if { $m_axi_pin_index != $m_axi_pins_len &&  ![incr_NUM_SI $target_intc_ip ]} {
            set target_intc_ip_index [expr { $target_intc_ip_index + 1 }]
        }
    }
}

proc connect { } {
    set ips [lsort -dictionary [get_bd_cells -quiet -filter { VLNV =~ "*:hls:*:*" || VLNV =~ "*:*:axi_dma:*"}]]
    # Processor System Resetを追加する
    set sys_reset [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 "sys_rst_0"]

    connect_axi_intc_gp_to_slave
    connect_master_to_axi_intc_hp
    set clk_in_list [get_bd_pins -filter {DIR == I && TYPE == clk } -of_objects [get_bd_cells]]
    set clk_out [get_bd_pins -filter {DIR == O && TYPE == clk} -of_objects [get_bd_cells]]
    foreach clk_in $clk_in_list {
        connect_bd_net $clk_out  $clk_in
    }

    set rst_ex_out [get_bd_pins -quiet -filter {DIR == O && TYPE == rst} -of_objects [get_bd_cells -filter {NAME !~ "*axi_dma*" && NAME !~ "*sys_rst*"}]]
    set rst_ex_in [get_bd_pins /sys_rst_0/ext_reset_in]
    connect_bd_net $rst_ex_out $rst_ex_in

    set rst_in_list [get_bd_pins -quiet -filter {DIR == I && TYPE == rst} -of_objects [get_bd_cells -filter {NAME !~ "*sys_rst*"}]]
    set rst_out [get_bd_pins /sys_rst_0/peripheral_aresetn]
    foreach rst_in $rst_in_list {
        connect_bd_net $rst_out  $rst_in
    }
}

set index 0
while { $index < $argc } {
    set option_flag [lindex $argv $index]
    incr index
    set value [lindex $argv $index]
    incr index

    switch $option_flag {
        -project_name {
            # プロジェクト名(必須)
            set project_name $value
        }
        -board_name {
            # ボードの名前(必須)
            set board_name $value
        }
        -bd_file_name {
            # ブロックデザインファイルの名前(デフォルト値: design_1)
            set bd_file_name $value
        }
        -ips_directory {
            # ユーザIPがある絶対パス(必須)
            set ips_directory $value
        }
        -auto_connect {
            # 自動的にIPコア同士を接続するか(デフォルト値: 1)
            set auto_connect $value
        }
        -write_bitstream {
            # ビットストリームを作成するか(デフォルト値: 1)
            set write_bitstream $value
        }
        -start_gui {
            # VivadoのGUIを立ち上げるか(デフォルト値: 1)
            set start_gui $value
        }
        -ip {
            # ブロックデザインに追加したいIPコア名と個数
            set ip_count [lindex $argv $index]
            incr index
            dict set ips $value $ip_count
        }
        default {
            puts $option_flag
            exit
        }
    }
}

if { [info exists project_name] } {
    create_vivado_project $project_name [file dirname [info script]]
}

if { ![info exists board_name] } {
    exit
}

if { ![info exists ips_directory] } {
    exit
}

if { ![info exists bd_file_name] } {
    set bd_file_name "design_1"
}

if { ![info exists auto_connect] } {
    set auto_connect 1
}

if { ![info exists write_bitstream] } {
    set write_bitstream 1
}

if { ![info exists start_gui] } {
    set start_gui 1
}

setup_vivado_project $board_name $ips_directory $bd_file_name
set PS [add_vivado_bd_ip_ps $board_name]
if  { [info exists ips] } {
    dict for {ip_name ip_count} $ips {
        add_vivado_bd_ip $ip_name $ip_count
    }
}

if { $auto_connect } {
    connect
}

save_bd_design

if { $write_bitstream } {
    # IPにアドレス空間を割り振る
    assign_bd_address
    make_wrapper -files [get_files "$bd_file_name.bd"] -top -import
    update_compile_order -fileset sources_1
    launch_runs impl_1 -to_step write_bitstream
}

if { $start_gui } {
    start_gui
}
