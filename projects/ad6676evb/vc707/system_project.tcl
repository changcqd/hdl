


source ../../scripts/adi_env.tcl
source $ad_hdl_dir/projects/scripts/adi_project.tcl
source $ad_hdl_dir/projects/scripts/adi_board.tcl

adi_project_create ad6676evb_vc707
adi_project_files ad6676evb_vc707 [list \
  "system_top.v" \
  "system_constr.xdc"\
  "$ad_hdl_dir/library/common/ad_iobuf.v" \
  "$ad_hdl_dir/projects/common/vc707/vc707_system_constr.xdc" ]

set_property is_enabled false [get_files  *axi_jesd_gt_tx_constr.xdc]

adi_project_run ad6676evb_vc707


