// ***************************************************************************
// ***************************************************************************
// Copyright 2014 - 2017 (c) Analog Devices, Inc. All rights reserved.
//
// In this HDL repository, there are many different and unique modules, consisting
// of various HDL (Verilog or VHDL) components. The individual modules are
// developed independently, and may be accompanied by separate and unique license
// terms.
//
// The user should read each of these license terms, and understand the
// freedoms and responsibilities that he or she has by using this source/core.
//
// This core is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE.
//
// Redistribution and use of source or resulting binaries, with or without modification
// of this file, are permitted under one of the following two license terms:
//
//   1. The GNU General Public License version 2 as published by the
//      Free Software Foundation, which can be found in the top level directory
//      of this repository (LICENSE_GPL2), and also online at:
//      <https://www.gnu.org/licenses/old-licenses/gpl-2.0.html>
//
// OR
//
//   2. An ADI specific BSD license, which can be found in the top level directory
//      of this repository (LICENSE_ADIBSD), and also on-line at:
//      https://github.com/analogdevicesinc/hdl/blob/master/LICENSE_ADIBSD
//      This will allow to generate bit files and not release the source code,
//      as long as it attaches to an ADI device.
//
// ***************************************************************************
// ***************************************************************************

module burst_fifo #(
  parameter DATA_WIDTH = 64,
  parameter ID_WIDTH = 3,
  parameter ASYNC_CLK = 1,
  parameter ADDRESS_WIDTH = 6
) (
  input src_clk,
  input src_reset,

  input src_data_valid,
  output reg src_data_ready = 1'b0,
  input [DATA_WIDTH-1:0] src_data,
  input src_data_last,

  input dest_clk,
  input dest_reset,

  output reg dest_data_valid = 1'b0,
  input dest_data_ready,
  output reg [DATA_WIDTH-1:0] dest_data = 'h00,
  output reg dest_data_last = 1'b0,

  output [ID_WIDTH-1:0] dest_request_id,
  input [ID_WIDTH-1:0] dest_data_request_id,
  output [ID_WIDTH-1:0] dest_data_response_id
);

localparam BURST_LEN_WIDTH = ADDRESS_WIDTH - ID_WIDTH + 1;

`include "inc_id.h"

wire [ID_WIDTH-1:0] dest_src_id;
wire [ID_WIDTH-1:0] src_dest_id;

wire src_beat;
wire src_last_beat;
wire dest_beat;
wire dest_last_beat;
wire dest_last;

wire [ADDRESS_WIDTH-1:0] dest_raddr;
wire [ADDRESS_WIDTH-1:0] src_waddr;

wire [BURST_LEN_WIDTH-1:0] dest_burst_len;

reg [ID_WIDTH-1:0] src_id = 'h0;
reg [ID_WIDTH-1:0] src_id_next;
reg [BURST_LEN_WIDTH-1:0] src_beat_counter = 'h00;
wire [ID_WIDTH-2:0] dest_reduced_id;

reg [ID_WIDTH-1:0] dest_id = 'h0;
reg [ID_WIDTH-1:0] dest_id_next;
reg [BURST_LEN_WIDTH-1:0] dest_beat_counter = 'h00;
wire [ID_WIDTH-2:0] src_reduced_id;

reg dest_valid = 1'b0;

reg [DATA_WIDTH-1:0] ram[0:2**ADDRESS_WIDTH-1];

reg [BURST_LEN_WIDTH-1:0] burst_len_mem[0:2**(ID_WIDTH-1)-1];

assign src_reduced_id = {src_id[ID_WIDTH-1]^src_id[ID_WIDTH-2],src_id[ID_WIDTH-3:0]};

assign src_beat = src_data_valid & src_data_ready;
assign src_last_beat = src_beat & src_data_last;
assign src_waddr = {src_reduced_id,src_beat_counter};

always @(*) begin
  if (src_last_beat == 1'b1) begin
    src_id_next <= inc_id(src_id);
  end else begin
    src_id_next <= src_id;
  end
end

always @(posedge src_clk) begin
  /* Ready if there is room for at least one full burst. */
  src_data_ready <= (src_id_next[ID_WIDTH-1] == src_dest_id[ID_WIDTH-1] ||
                src_id_next[ID_WIDTH-2] == src_dest_id[ID_WIDTH-2] ||
                src_id_next[ID_WIDTH-3:0] != src_dest_id[ID_WIDTH-3:0]);
end

always @(posedge src_clk) begin
  if (src_reset == 1'b1) begin
    src_id <= 'h00;
  end else begin
    src_id <= src_id_next;
  end
end

always @(posedge src_clk) begin
  if (src_reset == 1'b1) begin
    src_beat_counter <= 'h00;
  end else if (src_beat == 1'b1) begin
    src_beat_counter <= src_beat_counter + 1'b1;
  end
end

always @(posedge src_clk) begin
  if (src_beat == 1'b1) begin
    ram[src_waddr] <= src_data;
  end
end

always @(posedge src_clk) begin
  if (src_last_beat == 1'b1) begin
    burst_len_mem[src_reduced_id] <= src_beat_counter;
  end
end

assign dest_reduced_id = {dest_id[ID_WIDTH-1]^dest_id[ID_WIDTH-2],dest_id[ID_WIDTH-3:0]};

assign dest_burst_len = burst_len_mem[dest_reduced_id];

assign dest_ready = ~dest_data_valid | dest_data_ready;
assign dest_last = dest_beat_counter == dest_burst_len;

assign dest_beat = dest_valid & dest_ready;
assign dest_last_beat = dest_last & dest_beat;
assign dest_raddr = {dest_reduced_id,dest_beat_counter};

always @(posedge dest_clk) begin
  /* Valid if there is at least one full burst in the fifo */
  dest_valid <= dest_data_request_id != dest_id_next;
end

always @(posedge dest_clk) begin
  if (dest_reset == 1'b1) begin
    dest_data_valid <= 1'b0;
  end else if (dest_valid == 1'b1) begin
    dest_data_valid <= 1'b1;
  end else if (dest_data_ready == 1'b1) begin
    dest_data_valid <= 1'b0;
  end
end

always @(*) begin
  if (dest_last_beat == 1'b1) begin
    dest_id_next <= inc_id(dest_id);
  end else begin
    dest_id_next <= dest_id;
  end
end

always @(posedge dest_clk) begin
  if (dest_reset == 1'b1) begin
    dest_id <= 'h00;
  end else begin
    dest_id <= dest_id_next;
  end
end

always @(posedge dest_clk) begin
  if (dest_reset == 1'b1) begin
    dest_beat_counter <= 'h00;
  end else if (dest_beat == 1'b1) begin
    dest_beat_counter <= dest_beat_counter + 1'b1;
  end
end

always @(posedge dest_clk) begin
  if (dest_beat == 1'b1) begin
    dest_data <= ram[dest_raddr];
  end
end

always @(posedge dest_clk) begin
  if (dest_valid == 1'b1) begin
    dest_data_last <= dest_last;
  end else if (dest_data_ready == 1'b1) begin
    /*
     * This clears dest_data_last after the last beat. Strictly speaking this is
     * not necessary since dest_data_last is qualified by dest_data_valid and it
     * is OK to retain the previous value of dest_data_last when dest_data_valid
     * is not asserted. But it might be confusing to somebody looking at the raw
     * signal traces and clearing the signal here doesn't cost much.
     */
    dest_data_last <= 1'b0;
  end
end

sync_bits #(
  .NUM_OF_BITS(ID_WIDTH),
  .ASYNC_CLK(ASYNC_CLK)
) i_dest_sync_id (
  .in(src_id),
  .out_clk(dest_clk),
  .out_resetn(1'b1),
  .out(dest_src_id)
);

sync_bits #(
  .NUM_OF_BITS(ID_WIDTH),
  .ASYNC_CLK(ASYNC_CLK)
) i_src_sync_id (
  .in(dest_id),
  .out_clk(src_clk),
  .out_resetn(1'b1),
  .out(src_dest_id)
);

assign dest_request_id = dest_src_id;
assign dest_data_response_id = dest_id;

endmodule
