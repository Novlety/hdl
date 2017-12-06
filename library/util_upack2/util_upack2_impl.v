// ***************************************************************************
// ***************************************************************************
// Copyright 2017 (c) Analog Devices, Inc. All rights reserved.
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

module util_upack2_impl #(
  parameter NUM_OF_CHANNELS = 4,
  parameter SAMPLES_PER_CHANNEL = 1,
  parameter SAMPLE_DATA_WIDTH = 16
) (
  input clk,
  input reset,

  input [NUM_OF_CHANNELS-1:0] enable,

  input [NUM_OF_CHANNELS-1:0] fifo_rd_en,
  output reg fifo_rd_valid,
  output reg fifo_rd_underflow,
  output reg [NUM_OF_CHANNELS*SAMPLE_DATA_WIDTH*SAMPLES_PER_CHANNEL-1:0] fifo_rd_data,

  input s_axis_valid,
  output s_axis_ready,
  input [NUM_OF_CHANNELS*SAMPLE_DATA_WIDTH*SAMPLES_PER_CHANNEL-1:0] s_axis_data
);

  /* If the number of active channels can be a non-power of two */
  localparam NON_POWER_OF_TWO = NUM_OF_CHANNELS > 2;

  localparam CHANNEL_DATA_WIDTH = SAMPLES_PER_CHANNEL * SAMPLE_DATA_WIDTH;

  localparam TOTAL_DATA_WIDTH = CHANNEL_DATA_WIDTH * NUM_OF_CHANNELS;
  localparam NUM_OF_PORTS = NUM_OF_CHANNELS * SAMPLES_PER_CHANNEL;

  localparam PACK = 0;

  /*
   * Reset and control signals for the state machine. Data and control have
   * separate resets since control is pipelined and needs to be taken out of
   * reset before data so it can compute the control signals for the first data
   * cycle.
   */
  reg reset_data = 1'b1;
  reg reset_ctrl = 1'b1;
  reg startup_ctrl = 1'b0;
  reg startup_ctrl2 = 1'b0;
  reg [NUM_OF_CHANNELS-1:0] enable_int = 'h00;

  /*
   * Final output data of the routing network that will be written to
   * fifo_rd_data.
   */
  wire [TOTAL_DATA_WIDTH-1:0] deinterleaved_data;

  /*
   * Internal read signal.
   */
  wire data_rd_en;

  /*
   * Only the first signal of fifo_rd_en is used. All others are ignored. The
   * only reason why fifo_rd_en has multiple entries is to keep the interface
   * somewhat backwards compatible to the previous upack.
   */
  assign data_rd_en = fifo_rd_en[0];

  /*
   * Internal copy of the enable signals. This is used to detect changes in the
   * channel selection and reset the internal state when that happens.
   */
  always @(posedge clk) begin
    if (reset == 1'b1) begin
      enable_int <= {NUM_OF_PORTS{1'b0}};
    end else begin
      enable_int <= enable;
    end
  end

  /*
   * The internal state is reset whenever the selected channels change. The
   * control path is pipelined and computed one clock cycle in advance. This
   * means the control path needs to be taken out of reset one clock cycle
   * before the data path and a special startup cycles are required to compute
   * the first sets of control signals.
   *
   * In the case where there is only one channel no control signals are needed
   * and hence no startup cycle. In the case where there are two channels the
   * control signal pipeline is one cycle, so one startup cycle is required. For
   * more than two channels the startup pipeline is two channels and two startup
   * cycles are required.
   */
  always @(posedge clk) begin
    if (reset == 1'b1 || enable == {NUM_OF_PORTS{1'b0}}) begin
      reset_ctrl <= 1'b1;
      reset_data <= 1'b1;
      startup_ctrl <= 1'b0;
      startup_ctrl2 <= 1'b0;
    end else if (enable != enable_int) begin
      reset_ctrl <= 1'b1;
      reset_data <= 1'b1;
      startup_ctrl <= 1'b1;
      startup_ctrl2 <= 1'b1;
    end else begin
      reset_ctrl <= 1'b0;
      reset_data <= NUM_OF_CHANNELS != 1 ? startup_ctrl2 : 1'b0;
      startup_ctrl2 <= NON_POWER_OF_TWO ? reset_ctrl : 1'b0;
      startup_ctrl <= reset_ctrl | startup_ctrl2;
    end
  end

  generate
    if (NUM_OF_CHANNELS == 1) begin
      /*
       * In the one channel case there is not much to do. Nevertheless we should
       * support it to allow generic designs where the number of channels is
       * selected programmatically.
       */
      assign deinterleaved_data = s_axis_data;
      assign s_axis_ready = s_axis_valid & data_rd_en & ~reset_data;
    end else begin
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

      /*
       * `rotate` is used as an offset into the input data vector. When not all
       * samples are enabled it can take multiple cycles for the input vector to
       * be consumed. `rotate` points to the first sample in the input vector
       * that should consumed next. E.g. when there are 4 channels, but only 2
       * are enabled `rotate` will oscillate between 0 and 2. If there are 4
       * channels and 3 are enabled it will cycle through the sequence 0, 3, 2,
       * 1.
       */
      reg [PORT_ADDRESS_WIDTH-1:0] rotate = 'h00;

      /*
      * `prefix_count` counts the number of disabled channels that precede a
      * channel. E.g. if channel 0 is enabled and channel 1 and 2 are disabled
      * the prefix count for channel 3 is 2.
      */
      reg [PORT_ADDRESS_WIDTH*NUM_OF_PORTS-1:0] prefix_count;

      /*
       * Clock enable for all the control signals. When asserted the next cycle
       * for the control signals should computed
       */
      wire ce_ctrl;

      /*
       * Extended version of the `enable` signal that takes SAMPLES_PER_CHANNEL
       * into account.
       */
      wire [NUM_OF_PORTS-1:0] ports_enable;

      /*
       * Used to connect the different intermediary stages of the routing
       * network. There can be up to three sub-networks.
       */
      wire [TOTAL_DATA_WIDTH-1:0] data[0:2];

      /*
       * Unregistered version of `prefix_count`. This is used to add up the
       * enable ports.
       */
      wire [PORT_ADDRESS_WIDTH-1:0] prefix_count_s[0:NUM_OF_PORTS];

      assign ports_enable = {SAMPLES_PER_CHANNEL{enable_int}};

      /*
       * Control pipeline is active and should compute the next state either
       * during the startup phase or when a output data set is consumed.
       */
      assign ce_ctrl = startup_ctrl | (data_rd_en & s_axis_valid);

      /* First channel has no other channels before it */
      assign prefix_count_s[0] = 'h0;

      genvar i;
      for (i = 0; i < NUM_OF_PORTS; i = i + 1) begin: gen_prefix_count
        assign prefix_count_s[i+1] = prefix_count_s[i] + (ports_enable[i] ? 1'b0 : 1'b1);

        if (i < 2 || NUM_OF_CHANNELS <= 2) begin
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

      /*
       * Number of enabled channels - 1. Zero enabled channels is not a valid
       * configuration and storing it this way allows for better utilization.
       */
      reg [PORT_ADDRESS_WIDTH-1:0] enable_count = 'h0;

      always @(posedge clk) begin
        /*
         * `prefix_count` tracks the number of disabled channels. Invert it to
         * get the number of enabled channels - 1
         */
        enable_count <= ~prefix_count_s[NUM_OF_PORTS];
      end

      if (NON_POWER_OF_TWO == 1) begin: input_stage_gen
        /* Delayed data vector. Data from the previous cycle. */
        reg [TOTAL_DATA_WIDTH-2*CHANNEL_DATA_WIDTH-1:0] data_d1 = 'h00;

        /*
         * Same as `rotate`, but two pipeline stages ahead of the data path.
         * This is needed to compute some of the other control signals ahead of
         * time.
         */
        reg [PORT_ADDRESS_WIDTH:0] rotate_next;

        /*
         * MSB of the rotate control signal. This is used to move the source
         * data index to the delayed data vector. This will only ever be
         * asserted for one clock cycle at a time.
         */
        reg rotate_msb = 1'b0;

        /* Is asserted when the input vector should be reloaded */
        reg ready = 1'b0;

        /*
         * Extended version of the normal control and data signals that can
         * handle 2*NUM_OF_CHANNELS channels.
         */
        wire [(PORT_ADDRESS_WIDTH+1)*(2*NUM_OF_PORTS)-1:0] ext_prefix_count;
        wire [TOTAL_DATA_WIDTH*2-1:0] ext_data_in;
        wire [TOTAL_DATA_WIDTH*2-1:0] ext_data_out;
        wire [TOTAL_DATA_WIDTH*2-1:0] ext_data_shuffled;
        wire [PORT_ADDRESS_WIDTH:0] rotate_next_next;

        /*
         * This stage needs to handle 2*NUM_OF_CHANNELS channels so the prefix
         * count needs to padded with an extra bit.
         */
        for (i = 0; i < NUM_OF_PORTS; i = i + 1) begin: gen_ext_prefix_count1
          assign ext_prefix_count[i*(PORT_ADDRESS_WIDTH+1)+:PORT_ADDRESS_WIDTH+1] = {1'b0,prefix_count[i*PORT_ADDRESS_WIDTH+:PORT_ADDRESS_WIDTH]};
        end

        /*
         * The inversion and the addition of the constant will be folded into
         * the LUT that generates the control signals. This does not use up any
         * extra resources.
         */
        for (i = NUM_OF_PORTS; i < NUM_OF_PORTS * 2; i = i + 1) begin: gen_ext_prefix_count2
          assign ext_prefix_count[i*(PORT_ADDRESS_WIDTH+1)+:PORT_ADDRESS_WIDTH+1] = ~enable_count + i;
        end

        /*
         * For non power of two channel masks the previous data needs to be
         * saved since a single read can span over two consecutive input data
         * words. The lower two channels don't need to be saved since there are
         * no configurations in which they'd be required.
         */
        always @(posedge clk) begin
          if (s_axis_valid == 1'b1 && s_axis_ready == 1'b1) begin
            data_d1 <= s_axis_data[TOTAL_DATA_WIDTH-1:2*CHANNEL_DATA_WIDTH];
          end
        end

        /* Three pipeline steps ahead of data */
        assign rotate_next_next = rotate_next[PORT_ADDRESS_WIDTH-1:0] + enable_count + 1'b1;

        always @(posedge clk) begin
          if (reset_ctrl == 1'b1) begin
            ready <= 1'b0;
            rotate_msb <= 1'b0;

            rotate <= 'h0;
            rotate_next <= 'h0;
          end else if (ce_ctrl == 1'b1) begin
            ready <= 1'b0;
            rotate_msb <= 1'b0;

            /*
             * If the next cycle will consume more data than what is still
             * available ready needs to be asserted and the network needs to be
             * updated to source data from the delayed data register.
             */
            if (rotate_next_next[PORT_ADDRESS_WIDTH] &
                |rotate_next_next[PORT_ADDRESS_WIDTH-1:0]) begin
              ready <= 1'b1;
              rotate_msb <= 1'b1;
            end
            /*
             * If the current cycle consumes all available data ready needs to
             * be asserted, but only if it wasn't already asserted on the
             * previous cycle due to overconsumption.
             */
            if (rotate_next[PORT_ADDRESS_WIDTH] == 1'b1 && rotate_msb == 1'b0) begin
              ready <= 1'b1;
            end

            rotate <= rotate_next;
            rotate_next <= rotate_next_next;
          end
        end

        /*
         * First stage of the routing network. We know that we have at least 4
         * channels and hence 8 input to the network, so we'll always use a
         * 4-MUX based stage here.
         */
        pack_ctrl_interconnect #(
          .PORT_ADDRESS_WIDTH (PORT_ADDRESS_WIDTH + 1),
          .MUX_ORDER (2),
          .MIN_STAGE (0),
          .NUM_STAGES (1),
          .PORT_DATA_WIDTH (SAMPLE_DATA_WIDTH)
        ) i_ext_ctrl_interconnect (
          .clk (clk),
          .ce_ctrl (ce_ctrl),

          .rotate ({rotate_msb,rotate}),
          .prefix_count (ext_prefix_count),

          .data_in (ext_data_in),
          .data_out (ext_data_out)
        );

        /*
         * In order to go from this stage that has 2 * NUM_OF_PORTS inputs
         * and output to the remainder of the network that has only NUM_OF_PORTS
         * inputs and outputs every second two ports need to be skipped. I.e.
         * port 2, 3, 6, 7...
         * The shuffle groups all ports that are wanted into the first half of
         * `ext_data_shuffled` and the unwanted ports into the second half which
         * will be discarded.
         */
        ad_perfect_shuffle #(
          .NUM_GROUPS (NUM_OF_PORTS / 2),
          .WORDS_PER_GROUP (2),
          .WORD_WIDTH (2 * SAMPLE_DATA_WIDTH)
        ) i_ext_shuffle (
           .data_in (ext_data_out),
           .data_out (ext_data_shuffled)
        );

        assign ext_data_in = {data_d1,{2*CHANNEL_DATA_WIDTH{1'b0}},s_axis_data};
        assign data[0] = ext_data_shuffled[0+:TOTAL_DATA_WIDTH];
        assign s_axis_ready = ready & data_rd_en;
      end else begin
        reg ready = 1'b0;

        always @(posedge clk) begin
          if (reset_ctrl == 1'b1) begin
            ready <= 1'b0;
            rotate <= 'h0;
          end else if (ce_ctrl == 1'b1) begin
            /*
             * When all samples in the input vector has been consumed ready is
             * asserted for a single clock cycle. Here the number of enabled
             * channels is always a power of two. That means the input vector is
             * evenly divisible into the output data and there is no fractional
             * residual data. I.e. when ready is asserted rotate is 0.
             */
            {ready,rotate} <= rotate + enable_count + 1'b1;
          end
        end

        assign data[0] = s_axis_data;
        assign s_axis_ready = ready & data_rd_en;
      end

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
      for (i = 0; i < 2; i = i + 1) begin: gen_network
        localparam MUX_ORDER = i == PACK ? 2 : 1;
        localparam MIN_STAGE = i == PACK ? NON_POWER_OF_TWO : PORT_ADDRESS_WIDTH - 1;
        localparam NUM_STAGES = i == PACK ? (PORT_ADDRESS_WIDTH - NON_POWER_OF_TWO) / 2 : (PORT_ADDRESS_WIDTH - NON_POWER_OF_TWO) % 2;

        if (NUM_STAGES > 0) begin
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

      /*
       * Data at the output of the routing network is interleaved. The upack
       * core is supposed to produce deinterleaved data. This just rearrange the
       * bits in the data vector and does not consume any FPGA resources.
       */
      ad_perfect_shuffle #(
        .NUM_GROUPS (SAMPLES_PER_CHANNEL),
        .WORDS_PER_GROUP (NUM_OF_CHANNELS),
        .WORD_WIDTH (SAMPLE_DATA_WIDTH)
      ) i_deinterleave (
        .data_in (data[2]),
        .data_out (deinterleaved_data)
      );
    end
  endgenerate

  always @(posedge clk) begin
    /* In case of an underflow the output vector should be zeroed */
    if (reset_data == 1'b1 ||
        (data_rd_en == 1'b1 && s_axis_valid == 1'b0)) begin
      fifo_rd_data <= 'h00;
    end else if (data_rd_en == 1'b1) begin
      fifo_rd_data <= deinterleaved_data;
    end
  end

  always @(posedge clk) begin
    if (data_rd_en == 1'b1) begin
      fifo_rd_valid <= s_axis_valid & ~reset_data;
      fifo_rd_underflow <= ~(s_axis_valid & ~reset_data);
    end else begin
      fifo_rd_valid <= 1'b0;
      fifo_rd_underflow <= 1'b0;
    end
  end

endmodule
