set solution_path [lindex $argv 0]
set project_path [file join $solution_path ".." ".."]
set solution_name [file tail $solution_path]
set project_dir_name [file tail [file dirname $solution_path]]

file mkdir [file join "ip"]
set prev_wd [pwd]

puts $prev_wd
cd $project_path
puts $project_path

puts $solution_path
puts $solution_name
puts $project_dir_name


set project_solution [file join $project_dir_name $solution_name]
open_solution $project_solution
set ip_path [file join $prev_wd "ip" [get_top].zip]
export_design -rtl verilog -format ip_catalog -vendor xilinx.com -library hls -ipname [get_top] -version 1.0 -output $ip_path

exit
