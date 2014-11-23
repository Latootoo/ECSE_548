//---------------------------------------------------------
// or2.sv
// Yan Li 09/09/2014
// Reference: nand2.sv Nathaniel Pinckney 08/06/07
// Model and testbench of NAND2 gate
//--------------------------------------------------------

module testbench();
    logic clk;
    //logic a, b, y;
    //logic [2:0] vectors[4:0], currentvec;
    //logic [3:0] vectornum, errors;
    
    logic [3:0] a, b;
    logic y;
    logic [9:0] vectors[4:0], currentvec;
    logic [3:0] vectornum, errors;
  
    
    // The device under test
    //or2 dut(a, b, y);
    comp2_1x_4 lala(a, b, y);
  
    // read test vector file and initialize test
    initial begin
       $readmemb("comp-vectors.txt", vectors);
       vectornum = 0; errors = 0;
    end
    // generate a clock to sequence tests
    always begin
       clk = 1; #10; clk = 0; #10;
    end
    // apply test
    always @(posedge clk) begin
       currentvec = vectors[vectornum];
       a = currentvec[8:5];
       b = currentvec[4:1];
       if (currentvec[0] === 1'bx) begin
         $display("Completed %d tests with %d errors.", 
                  vectornum, errors);
         $stop;
       end
    end
    // check if test was sucessful and apply next one
    always @(negedge clk) begin
       if ((y !== currentvec[0])) begin
          $display("Error: inputs were a=%h b=%h", a, b);
          $display("       output mismatches as %h (%h expected)", 
                   y,currentvec[0]);
          errors = errors + 1;
       end
       vectornum = vectornum + 1;
    end
endmodule  

/*module or2(input  logic [3:0]a,
             input  logic [3:0]b,
             output logic y);
             
   logic [3:0] c;
   assign #1 c = (a ^ b); 
   assign #1 y= ~(c[0]|c[1]|c[2]|c[3]);   
endmodule*/
/* Verilog for cell 'wordlib8:comp2_1x_4{sch}' from library 'wordlib8' */
/* Created on 星期三 十月 29, 2014 22:23:08 */
/* Last revised on 星期四 十月 30, 2014 11:27:35 */
/* Written on 星期四 十月 30, 2014 11:27:47 by Electric VLSI Design System, version 8.06 */

module muddlib07__nor4_1x(a, b, c, d, y);
  input a;
  input b;
  input c;
  input d;
  output y;

  supply1 vdd;
  supply0 gnd;
  wire net_2, net_3, net_41;

  tranif1 nmos_0(gnd, y, d);
  tranif1 nmos_1(gnd, y, c);
  tranif1 nmos_2(gnd, y, b);
  tranif1 nmos_3(gnd, y, a);
  tranif0 pmos_0(net_41, vdd, a);
  tranif0 pmos_1(net_2, net_41, b);
  tranif0 pmos_2(net_3, net_2, c);
  tranif0 pmos_3(y, net_3, d);
endmodule   /* muddlib07__nor4_1x */

module muddlib07__xor2_1x(a, b, y);
  input a;
  input b;
  output y;

  supply1 vdd;
  supply0 gnd;
  wire ab, bb, net_3, net_4, net_7, net_8;

  tranif1 nmos_0(gnd, net_3, a);
  tranif1 nmos_1(gnd, net_4, ab);
  tranif1 nmos_2(net_3, y, b);
  tranif1 nmos_3(net_4, y, bb);
  tranif1 nmos_4(gnd, bb, b);
  tranif1 nmos_5(gnd, ab, a);
  tranif0 pmos_0(y, net_7, b);
  tranif0 pmos_1(net_7, vdd, ab);
  tranif0 pmos_2(y, net_8, bb);
  tranif0 pmos_3(net_8, vdd, a);
  tranif0 pmos_4(bb, vdd, b);
  tranif0 pmos_5(ab, vdd, a);
endmodule   /* muddlib07__xor2_1x */

module comp2_1x_4(a, b, status);
  input [3:0] a;
  input [3:0] b;
  output status;

  supply1 vdd;
  supply0 gnd;
  wire net_16, net_20, net_24, net_28;

  muddlib07__nor4_1x nor4_1x_0(.a(net_16), .b(net_20), .c(net_24), .d(net_28), 
      .y(status));
  muddlib07__xor2_1x xor2_1x_0(.a(a[0]), .b(b[0]), .y(net_16));
  muddlib07__xor2_1x xor2_1x_1(.a(a[1]), .b(b[1]), .y(net_20));
  muddlib07__xor2_1x xor2_1x_3(.a(a[2]), .b(b[2]), .y(net_24));
  muddlib07__xor2_1x xor2_1x_4(.a(a[3]), .b(b[3]), .y(net_28));
endmodule   /* comp2_1x_4 */




