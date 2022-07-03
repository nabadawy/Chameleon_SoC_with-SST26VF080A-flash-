// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Example clock gating module for yosys synthesis
`default_nettype none
module prim_clock_gating (
  input  wire clk_i,
  input wire  en_i,
  input wire test_en_i,
  output wire clk_o
);
/*
  reg en_latch;

  always @* begin
    if (!clk_i) begin
      en_latch = en_i | test_en_i;
    end
  end
  assign clk_o = en_latch & clk_i;
*/
  assign clk_o = clk_i;
endmodule
