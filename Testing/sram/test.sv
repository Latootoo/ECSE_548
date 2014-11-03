module testbench();
    logic clk;
    logic f;
    logic [3:0] s;
    logic [15:0] y, expected;
    logic [2:0] vectors[7:0], currentvec;
    logic [3:0] vectornum, errors;

	logic out, out_b;
	logic in, wl;
	logic we;
	
    // device under test
    //sramread dut(clk, clk, out, out_b);;
	//sramwrite dut(in, clk, clk, we);
	sramrw dut(in, ~clk, ~clk, we, out, out_b);
	// read test file and initialize test
    initial begin
        $readmemb("test-vectors.txt", vectors);
        vectornum = 0; errors = 0;
    end
    // generate clock
    always begin
        clk = 1; #10; clk = 0; #10;
    end
    // apply test
    always @(posedge clk) begin
        currentvec = vectors[vectornum];

		in = currentvec[0];
		wl = currentvec[1];
		we = currentvec[2];
		
        if (currentvec[0] === 1'bx) begin
            $stop;
        end
    end
    // check errors
    always @(negedge clk) begin

        vectornum = vectornum + 1;
    end
endmodule /* testbench */

/* Verilog for cell 'sramrw{sch}' from library 'vlsi' */
/* Created on Thu Oct 30, 2014 23:37:31 */
/* Last revised on Mon Nov 03, 2014 13:19:37 */
/* Written on Mon Nov 03, 2014 13:20:08 by Electric VLSI Design System, version 8.06 */

module muddlib07__inv_1x(a, y);
  input a;
  output y;

  supply1 vdd;
  supply0 gnd;
  tranif1 nmos_0(gnd, y, a);
  tranif0 pmos_0(y, vdd, a);
endmodule   /* muddlib07__inv_1x */

module vlsi__inv_hi(a, y);
  input a;
  output y;

  supply1 vdd;
  supply0 gnd;
  tranif1 nmos_0(gnd, y, a);
  tranif0 pmos_0(y, vdd, a);
endmodule   /* vlsi__inv_hi */

module muddlib07__srambit(bit_a, bit_b, word);
  input bit_a;
  input bit_b;
  input word;

  supply1 vdd;
  supply0 gnd;
  wire net_67, net_68;

  tranif1 nmos_4(gnd, net_67, net_68);
  tranif1 nmos_5(net_68, gnd, net_67);
  tranif1 nmos_6(bit_a, net_68, word);
  tranif1 nmos_7(net_67, bit_b, word);
  rtranif0 pmos_2(net_67, vdd, net_68);
  rtranif0 pmos_3(vdd, net_68, net_67);
endmodule   /* muddlib07__srambit */

module sramrw(in, ph1, ph2, we, out, out_b);
  input in;
  input ph1;
  input ph2;
  input we;
  output out;
  output out_b;

  supply1 vdd;
  supply0 gnd;
  wire net_17, net_2, net_21, net_3, net_8;

  tranif1 nmos_0(net_8, net_2, we);
  tranif1 nmos_1(net_17, net_3, we);
  tranif1 nmos_2(gnd, net_8, in);
  tranif1 nmos_3(gnd, net_17, net_21);
  tranif0 pmos_0(net_3, vdd, ph2);
  tranif0 pmos_1(net_2, vdd, ph2);
  muddlib07__inv_1x inv_1x_0(.a(in), .y(net_21));
  vlsi__inv_hi inv_hi_0(.a(net_2), .y(out_b));
  vlsi__inv_hi inv_hi_2(.a(net_3), .y(out));
  muddlib07__srambit srambit_0(.bit_a(net_2), .bit_b(net_3), .word(ph1));
endmodule   /* sramrw */
