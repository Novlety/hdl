// ***************************************************************************
// ***************************************************************************
// Copyright 2018 (c) Analog Devices, Inc. All rights reserved.
//
// In this HDL repository, there are many different and unique modules, consisting
// of various HDL (Verilog or VHDL) components. The individual modules are
// developed independently, and may be accompanied by separate and unique license
// terms.
//
// The user should read each of these license terms, and understand the
// freedoms and responsabilities that he or she has by using this source/core.
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

module util_cpack2_impl #(
  parameter NUM_OF_CHANNELS = 4,
  parameter SAMPLES_PER_CHANNEL = 1,
  parameter SAMPLE_DATA_WIDTH = 16
) (
  input clk,
  input reset,

  input [NUM_OF_CHANNELS-1:0] enable,

  input [NUM_OF_CHANNELS-1:0] fifo_wr_en,
  output fifo_wr_overflow,
  input [NUM_OF_CHANNELS*SAMPLE_DATA_WIDTH*SAMPLES_PER_CHANNEL-1:0] fifo_wr_data,

  output reg packed_fifo_wr_en = 1'b0,
  input packed_fifo_wr_overflow,
  output packed_fifo_wr_sync,
  output reg [NUM_OF_CHANNELS*SAMPLE_DATA_WIDTH*SAMPLES_PER_CHANNEL-1:0] packed_fifo_wr_data = 'h00
);

  /* If the number of active channels can be a non-power of two */
  localparam NON_POWER_OF_TWO = NUM_OF_CHANNELS > 2 && 0;
  localparam MUL = NON_POWER_OF_TWO ? 2 : 1;

  localparam TOTAL_DATA_WIDTH_OUT = NUM_OF_CHANNELS * SAMPLE_DATA_WIDTH * SAMPLES_PER_CHANNEL;
  localparam TOTAL_DATA_WIDTH_IN = TOTAL_DATA_WIDTH_OUT * MUL;
  localparam NUM_OF_PORTS = NUM_OF_CHANNELS * SAMPLES_PER_CHANNEL * MUL;

  localparam PACK = 1;

  localparam PORT_ADDRESS_WIDTH =
    NUM_OF_PORTS > 512 ? 10 :
    NUM_OF_PORTS > 256 ? 9 :
    NUM_OF_PORTS > 128 ? 8 :
    NUM_OF_PORTS > 64 ? 7 :
    NUM_OF_PORTS > 32 ? 6 :
    NUM_OF_PORTS > 16 ? 5 :
    NUM_OF_PORTS > 8 ? 4 :
    NUM_OF_PORTS > 4 ? 3 :
    NUM_OF_PORTS > 2 ? 2 : 1;

  reg reset_data = 1'b1;
  reg reset_ctrl = 1'b1;
  reg startup_ctrl = 1'b0;
  reg ready = 1'b0;
  reg [PORT_ADDRESS_WIDTH-1:0] rotate = 'h00;
  reg [NUM_OF_CHANNELS-1:0] enable_int = 'h00;

  wire [NUM_OF_PORTS-1:0] ports_enable;
  wire [TOTAL_DATA_WIDTH_IN-1:0] data[0:2];
  wire ce_ctrl;

  wire [PORT_ADDRESS_WIDTH-1:0] prefix_count_s[0:NUM_OF_PORTS];
  reg [PORT_ADDRESS_WIDTH*NUM_OF_PORTS-1:0] prefix_count;
  reg [PORT_ADDRESS_WIDTH-1:0] enable_count;

  wire [TOTAL_DATA_WIDTH_IN-1:0] interleaved_data;
  reg [NUM_OF_PORTS-1:0] packed_mask = 'h00;

  wire data_wr_en = fifo_wr_en[0];

  generate
  genvar i;

  assign prefix_count_s[0] = 0;
  assign ports_enable = {SAMPLES_PER_CHANNEL{enable_int}};

  for (i = 0; i < NUM_OF_PORTS; i = i + 1) begin: gen_prefix_count
    assign prefix_count_s[i+1] = prefix_count_s[i] + (ports_enable[i] ? 1'b0 : 1'b1);

    if (i < 2 || NUM_OF_CHANNELS <= 4) begin
      /* This will only be one bit, no need to register it */
      always @(*) begin
        prefix_count[i*PORT_ADDRESS_WIDTH+:PORT_ADDRESS_WIDTH] <= prefix_count_s[i];
      end
    end else begin
      always @(posedge clk) begin
        prefix_count[i*PORT_ADDRESS_WIDTH+:PORT_ADDRESS_WIDTH] <= prefix_count_s[i];
      end
    end
  end
  endgenerate

  always @(posedge clk) begin
    enable_count <= ~prefix_count_s[NUM_OF_PORTS];
  end

  assign ce_ctrl = data_wr_en | startup_ctrl;
  assign fifo_wr_overflow = packed_fifo_wr_overflow;
  assign packed_fifo_wr_sync = 1'b1;

  /*
   * The internal state is reset whenever the selected channels change.
   * The control path is pipelined and computed one clock cycle in advance. This
   * means the control path needs to be taken out of reset one clock cycle before
   * the data path and a special startup cycle is required to compute the first
   * set of control signals.
   */
  always @(posedge clk) begin
    if (reset == 1'b1) begin
      reset_data <= 1'b1;
      reset_ctrl <= 1'b1;
      startup_ctrl <= 1'b0;
    end else if (enable != enable_int ||
                 enable == {NUM_OF_PORTS{1'b0}}) begin
      reset_data <= 1'b1;
      reset_ctrl <= 1'b1;
      startup_ctrl <= 1'b1;
    end else begin
      reset_data <= reset_ctrl;
      reset_ctrl <= 1'b0;
      startup_ctrl <= reset_ctrl;
    end
  end

  always @(posedge clk) begin
    if (reset == 1'b1) begin
      enable_int <= {NUM_OF_PORTS{1'b0}};
    end else begin
      enable_int <= enable;
    end
  end

  always @(posedge clk) begin
    if (reset_ctrl == 1'b1) begin
      rotate <= 'h00;
      ready <= 1'b0;
    end else if (ce_ctrl == 1'b1) begin
      {ready,rotate} <= rotate + enable_count + 1'b1;
    end
  end

  generate
  if (NON_POWER_OF_TWO == 1 && 0) begin: input_stage_gen
    reg [TOTAL_DATA_WIDTH_IN-1:0] data_buffer = 'h00;
    reg [1:0] wr_cnt;

    always @(posedge clk) begin
      if (data_wr_en == 1'b1) begin
        data_buffer[TOTAL_DATA_WIDTH_OUT*wr_cnt[0]+:TOTAL_DATA_WIDTH_OUT] <= fifo_wr_data;
      end
    end

    always @(posedge clk) begin
      if (reset_ctrl == 1'b1) begin
        wr_cnt <= 2'b00;
      end else if (data_wr_en == 1'b1) begin
        wr_cnt <= wr_cnt + 1'b1;
      end
    end
  end else begin
  end
  endgenerate

  assign data[0] = interleaved_data;


  /*
   * The routing network can be built from any type of MUX. When it comes to
   * resource usage 2:1 MUXes and 4:1 MUXes are the most efficient, both will
   * require the same amount of LUTs. But a network built from 4:1 MUXes only uses
   * half the number of stages of a network built from 2:1 MUXes, so it has a
   * shorter routing delay and is the preferred architecture.
   *
   * For a pure 4:1 MUX network the number of ports is a power of 4. For a 2:1 MUX
   * network the number of ports is a power of 2. To get the best from both worlds
   * build the last stage from 2:1 MUXes when the number of ports is a power of
   * 2 and use 4:1 MUXes for the other stages.
   */
  generate
  genvar j;

  for (i = 0; i < 2; i = i + 1) begin: gen_network
    localparam MUX_ORDER = i == 0 ? 2 : 1;
    localparam MIN_STAGE = i == 0 ?  PORT_ADDRESS_WIDTH % 2 : 0;
    localparam NUM_STAGES = i == 0 ? PORT_ADDRESS_WIDTH / 2 : PORT_ADDRESS_WIDTH % 2;

    if (NUM_STAGES != 0) begin
      pack_ctrl_interconnect #(
        .PACK (PACK),
        .PORT_ADDRESS_WIDTH (PORT_ADDRESS_WIDTH),
        .MUX_ORDER (MUX_ORDER),
        .MIN_STAGE (MIN_STAGE),
        .NUM_STAGES (NUM_STAGES),
        .PORT_DATA_WIDTH (SAMPLE_DATA_WIDTH)
      ) i_ctrl_interconnect (
        .clk (clk),
        .ce_ctrl (ce_ctrl),

        .rotate (rotate),
        .prefix_count (prefix_count),

        .data_in (data[i]),
        .data_out (data[i+1])
      );
    end else begin
      assign data[i+1] = data[i];
    end
  end

  endgenerate

  ad_perfect_shuffle #(
    .NUM_GROUPS (NUM_OF_CHANNELS),
    .WORDS_PER_GROUP (SAMPLES_PER_CHANNEL),
    .WORD_WIDTH (SAMPLE_DATA_WIDTH)
  ) i_interleave (
    .data_in (fifo_wr_data),
    .data_out (interleaved_data)
  );

  always @(posedge clk) begin
    if (reset_data == 1'b1) begin
      packed_fifo_wr_en <= 1'b0;
    end else if (ready == 1'b1 && data_wr_en == 1'b1) begin
      packed_fifo_wr_en <= 1'b1;
    end else begin
      packed_fifo_wr_en <= 1'b0;
    end
  end

  always @(posedge clk) begin
    if (ce_ctrl == 1'b1) begin
      packed_mask <= ((1 << (enable_count+1)) - 1) << rotate;
    end
  end

  integer n;

  always @(posedge clk) begin
    if (data_wr_en == 1'b1) begin
      for (n = 0; n < NUM_OF_PORTS; n = n+1) begin
        if (packed_mask[n] == 1'b1) begin
          packed_fifo_wr_data[n*SAMPLE_DATA_WIDTH+:SAMPLE_DATA_WIDTH] <= data[2][n*SAMPLE_DATA_WIDTH+:SAMPLE_DATA_WIDTH];
        end
      end
    end
  end

endmodule
