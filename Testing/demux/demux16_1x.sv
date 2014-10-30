module testbench_demux();
    logic clk;
    logic f;
    logic [3:0] s;
    logic [15:0] y, expected;
    logic [20:0] vectors[4:0], currentvec;
    logic [3:0] vectornum, errors;

    // device under test
    demux16_1x dut(f, s, y);

    // read test file and initialize test
    initial begin
        $readmemb("demux16_1x-vectors.txt", vectors);
        vectornum = 0; errors = 0;
    end
    // generate clock
    always begin
        clk = 1; #10; clk = 0; #10;
    end
    // apply test
    always @(posedge clk) begin
        currentvec = vectors[vectornum];
        f = currentvec[20];
        s = currentvec[19:16];
        if (currentvec[0] === 1'bx) begin
            $display("Completed %d tests with %d errors.", 
                        vectornum, errors);
            $stop;
        end
    end
    // check errors
    always @(negedge clk) begin
      expected = currentvec[15:0];
        if (y !== expected) begin
            $display("       output mismatches as %h (%h expected)", 
                        expected, y);
            errors = errors + 1;
        end
        vectornum = vectornum + 1;
    end
endmodule /* testbench */

/* Verilog for cell 'demux16_1x{sch}' from library 'vlsi' */
/* Created on ÐÇÆÚÈý Ê®ÔÂ 29, 2014 19:24:31 */
/* Last revised on ÐÇÆÚÈý Ê®ÔÂ 29, 2014 23:16:10 */
/* Written on ÐÇÆÚËÄ Ê®ÔÂ 30, 2014 00:17:45 by Electric VLSI Design System, version 8.06 */

module muddlib07__and2_1x(a, b, y);
  input a;
  input b;
  output y;

  supply1 vdd;
  supply0 gnd;
  wire net_1, net_2;

  tranif1 nmos_0(net_1, net_2, b);
  tranif1 nmos_1(gnd, net_1, a);
  tranif1 nmos_2(gnd, y, net_2);
  tranif0 pmos_0(net_2, vdd, b);
  tranif0 pmos_1(net_2, vdd, a);
  tranif0 pmos_2(y, vdd, net_2);
endmodule   /* muddlib07__and2_1x */

module muddlib07__inv_1x(a, y);
  input a;
  output y;

  supply1 vdd;
  supply0 gnd;
  tranif1 nmos_0(gnd, y, a);
  tranif0 pmos_0(y, vdd, a);
endmodule   /* muddlib07__inv_1x */

module vlsi__demux2_1x(f, s, y0, y1);
  input f;
  input s;
  output y0;
  output y1;

  supply1 vdd;
  supply0 gnd;
  wire net_4;

  muddlib07__and2_1x and2_1x_0(.a(s), .b(f), .y(y1));
  muddlib07__and2_1x and2_1x_1(.a(net_4), .b(f), .y(y0));
  muddlib07__inv_1x inv_1x_0(.a(s), .y(net_4));
endmodule   /* vlsi__demux2_1x */

module vlsi__demux4_1x(f, s0, s1, y0, y1, y2, y3);
  input f;
  input s0;
  input s1;
  output y0;
  output y1;
  output y2;
  output y3;

  supply1 vdd;
  supply0 gnd;
  wire net_11, net_13;

  vlsi__demux2_1x demux2_1_0(.f(net_11), .s(s0), .y0(y2), .y1(y3));
  vlsi__demux2_1x demux2_1_1(.f(net_13), .s(s0), .y0(y0), .y1(y1));
  vlsi__demux2_1x demux2_1_2(.f(f), .s(s1), .y0(net_13), .y1(net_11));
endmodule   /* vlsi__demux4_1x */

module vlsi__demux8_1x(f, s, y);
  input f;
  input [2:0] s;
  output [7:0] y;

  supply1 vdd;
  supply0 gnd;
  wire net_0, net_2;

  vlsi__demux2_1x demux2_1_0(.f(f), .s(s[2]), .y0(net_2), .y1(net_0));
  vlsi__demux4_1x demux4_1_0(.f(net_0), .s0(s[0]), .s1(s[1]), .y0(y[4]), 
      .y1(y[5]), .y2(y[6]), .y3(y[7]));
  vlsi__demux4_1x demux4_1_1(.f(net_2), .s0(s[0]), .s1(s[1]), .y0(y[0]), 
      .y1(y[1]), .y2(y[2]), .y3(y[3]));
endmodule   /* vlsi__demux8_1x */

module demux16_1x(f, s, y);
  input f;
  input [3:0] s;
  output [15:0] y;

  supply1 vdd;
  supply0 gnd;
  wire net_0, net_2;

  vlsi__demux2_1x demux2_1_0(.f(f), .s(s[3]), .y0(net_2), .y1(net_0));
  vlsi__demux8_1x demux8_1_0(.f(net_0), .s(s[2:0]), .y(y[15:8]));
  vlsi__demux8_1x demux8_1_1(.f(net_2), .s(s[2:0]), .y(y[7:0]));
endmodule   /* demux16_1x */
