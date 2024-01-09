// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// HSIAO SECDED Encoder/Decoder for 64-bit words

module secded_72_64_enc (
  input        [63:0] in,
  output logic [71:0] out
);

  always_comb begin : p_encode
    out[63:0] = in;
    out[64] = ^(in & 64'hF8000000001FFFFF);
    out[65] = ^(in & 64'h9D00000FFFE0003F);
    out[66] = ^(in & 64'h8F003FF003E007C1);
    out[67] = ^(in & 64'hF10FC0F03C207842);
    out[68] = ^(in & 64'h6E71C711C4438884);
    out[69] = ^(in & 64'h3EB65926488C9108);
    out[70] = ^(in & 64'hD3DAAA4A91152210);
    out[71] = ^(in & 64'h67ED348D221A4420);
  end

endmodule : secded_72_64_enc

module secded_72_64_dec (
  input        [71:0] in,
  output logic [63:0] d_o,
  output logic [7:0] syndrome_o,
  output logic [1:0] err_o
);

  logic single_error;

  // Syndrome calculation
  assign syndrome_o[0] = ^(in & 72'h01F8000000001FFFFF);
  assign syndrome_o[1] = ^(in & 72'h029D00000FFFE0003F);
  assign syndrome_o[2] = ^(in & 72'h048F003FF003E007C1);
  assign syndrome_o[3] = ^(in & 72'h08F10FC0F03C207842);
  assign syndrome_o[4] = ^(in & 72'h106E71C711C4438884);
  assign syndrome_o[5] = ^(in & 72'h203EB65926488C9108);
  assign syndrome_o[6] = ^(in & 72'h40D3DAAA4A91152210);
  assign syndrome_o[7] = ^(in & 72'h8067ED348D221A4420);

  // Corrected output calculation
  assign d_o[0] = (syndrome_o == 8'h7) ^ in[0];
  assign d_o[1] = (syndrome_o == 8'hb) ^ in[1];
  assign d_o[2] = (syndrome_o == 8'h13) ^ in[2];
  assign d_o[3] = (syndrome_o == 8'h23) ^ in[3];
  assign d_o[4] = (syndrome_o == 8'h43) ^ in[4];
  assign d_o[5] = (syndrome_o == 8'h83) ^ in[5];
  assign d_o[6] = (syndrome_o == 8'hd) ^ in[6];
  assign d_o[7] = (syndrome_o == 8'h15) ^ in[7];
  assign d_o[8] = (syndrome_o == 8'h25) ^ in[8];
  assign d_o[9] = (syndrome_o == 8'h45) ^ in[9];
  assign d_o[10] = (syndrome_o == 8'h85) ^ in[10];
  assign d_o[11] = (syndrome_o == 8'h19) ^ in[11];
  assign d_o[12] = (syndrome_o == 8'h29) ^ in[12];
  assign d_o[13] = (syndrome_o == 8'h49) ^ in[13];
  assign d_o[14] = (syndrome_o == 8'h89) ^ in[14];
  assign d_o[15] = (syndrome_o == 8'h31) ^ in[15];
  assign d_o[16] = (syndrome_o == 8'h51) ^ in[16];
  assign d_o[17] = (syndrome_o == 8'h91) ^ in[17];
  assign d_o[18] = (syndrome_o == 8'h61) ^ in[18];
  assign d_o[19] = (syndrome_o == 8'ha1) ^ in[19];
  assign d_o[20] = (syndrome_o == 8'hc1) ^ in[20];
  assign d_o[21] = (syndrome_o == 8'he) ^ in[21];
  assign d_o[22] = (syndrome_o == 8'h16) ^ in[22];
  assign d_o[23] = (syndrome_o == 8'h26) ^ in[23];
  assign d_o[24] = (syndrome_o == 8'h46) ^ in[24];
  assign d_o[25] = (syndrome_o == 8'h86) ^ in[25];
  assign d_o[26] = (syndrome_o == 8'h1a) ^ in[26];
  assign d_o[27] = (syndrome_o == 8'h2a) ^ in[27];
  assign d_o[28] = (syndrome_o == 8'h4a) ^ in[28];
  assign d_o[29] = (syndrome_o == 8'h8a) ^ in[29];
  assign d_o[30] = (syndrome_o == 8'h32) ^ in[30];
  assign d_o[31] = (syndrome_o == 8'h52) ^ in[31];
  assign d_o[32] = (syndrome_o == 8'h92) ^ in[32];
  assign d_o[33] = (syndrome_o == 8'h62) ^ in[33];
  assign d_o[34] = (syndrome_o == 8'ha2) ^ in[34];
  assign d_o[35] = (syndrome_o == 8'hc2) ^ in[35];
  assign d_o[36] = (syndrome_o == 8'h1c) ^ in[36];
  assign d_o[37] = (syndrome_o == 8'h2c) ^ in[37];
  assign d_o[38] = (syndrome_o == 8'h4c) ^ in[38];
  assign d_o[39] = (syndrome_o == 8'h8c) ^ in[39];
  assign d_o[40] = (syndrome_o == 8'h34) ^ in[40];
  assign d_o[41] = (syndrome_o == 8'h54) ^ in[41];
  assign d_o[42] = (syndrome_o == 8'h94) ^ in[42];
  assign d_o[43] = (syndrome_o == 8'h64) ^ in[43];
  assign d_o[44] = (syndrome_o == 8'ha4) ^ in[44];
  assign d_o[45] = (syndrome_o == 8'hc4) ^ in[45];
  assign d_o[46] = (syndrome_o == 8'h38) ^ in[46];
  assign d_o[47] = (syndrome_o == 8'h58) ^ in[47];
  assign d_o[48] = (syndrome_o == 8'h98) ^ in[48];
  assign d_o[49] = (syndrome_o == 8'h68) ^ in[49];
  assign d_o[50] = (syndrome_o == 8'ha8) ^ in[50];
  assign d_o[51] = (syndrome_o == 8'hc8) ^ in[51];
  assign d_o[52] = (syndrome_o == 8'h70) ^ in[52];
  assign d_o[53] = (syndrome_o == 8'hb0) ^ in[53];
  assign d_o[54] = (syndrome_o == 8'hd0) ^ in[54];
  assign d_o[55] = (syndrome_o == 8'he0) ^ in[55];
  assign d_o[56] = (syndrome_o == 8'hce) ^ in[56];
  assign d_o[57] = (syndrome_o == 8'hf4) ^ in[57];
  assign d_o[58] = (syndrome_o == 8'hb6) ^ in[58];
  assign d_o[59] = (syndrome_o == 8'h37) ^ in[59];
  assign d_o[60] = (syndrome_o == 8'h6b) ^ in[60];
  assign d_o[61] = (syndrome_o == 8'hb9) ^ in[61];
  assign d_o[62] = (syndrome_o == 8'hd9) ^ in[62];
  assign d_o[63] = (syndrome_o == 8'h4f) ^ in[63];

  // err_o calc. bit0: single error, bit1: double error
  assign single_error = ^syndrome_o;
  assign err_o[0] = single_error;
  assign err_o[1] = ~single_error & (|syndrome_o);

endmodule : secded_72_64_dec
