# ip

source ../scripts/adi_env.tcl
source $ad_hdl_dir/library/scripts/adi_ip.tcl

adi_ip_create util_dacfifo
adi_ip_files util_dacfifo [list \
  "$ad_hdl_dir/library/common/ad_mem.v" \
  "$ad_hdl_dir/library/common/ad_b2g.v" \
  "$ad_hdl_dir/library/common/ad_g2b.v" \
  "util_dacfifo.v" \
  "util_dacfifo_constr.xdc"]

adi_ip_properties_lite util_dacfifo

ipx::infer_bus_interface dma_clk xilinx.com:signal:clock_rtl:1.0 [ipx::current_core]
ipx::infer_bus_interface dma_rst xilinx.com:signal:reset_rtl:1.0 [ipx::current_core]
ipx::infer_bus_interface dac_clk xilinx.com:signal:clock_rtl:1.0 [ipx::current_core]
ipx::infer_bus_interface dac_rst xilinx.com:signal:reset_rtl:1.0 [ipx::current_core]

ipx::save_core [ipx::current_core]

