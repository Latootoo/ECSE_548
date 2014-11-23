/* Verilog for cell 'comparator{sch}' from library 'mips8' */
/* Created on 星期五 十一月 21, 2014 16:36:11 */
/* Last revised on 星期五 十一月 21, 2014 17:14:03 */
/* Written on 星期五 十一月 21, 2014 17:14:35 by Electric VLSI Design System, version 8.06 */

module muddlib07__and4_1x(a, b, c, d, y);
  input a;
  input b;
  input c;
  input d;
  output y;

  supply1 vdd;
  supply0 gnd;
  wire net_1, net_45, net_46, net_9;

  tranif1 nmos_0(net_45, net_9, c);
  tranif1 nmos_1(net_9, net_1, d);
  tranif1 nmos_3(gnd, y, net_1);
  tranif1 nmos_4(net_46, net_45, b);
  tranif1 nmos_6(gnd, net_46, a);
  tranif0 pmos_0(net_1, vdd, d);
  tranif0 pmos_1(y, vdd, net_1);
  tranif0 pmos_2(net_1, vdd, c);
  tranif0 pmos_3(net_1, vdd, b);
  tranif0 pmos_4(net_1, vdd, a);
endmodule   /* muddlib07__and4_1x */

module muddlib07__xnor2_1x(a, b, y);
  input a;
  input b;
  output y;

  supply1 vdd;
  supply0 gnd;
  wire ab, bb, net_3, net_4, net_7, net_8;

  tranif1 nmos_0(gnd, net_3, a);
  tranif1 nmos_1(gnd, net_4, ab);
  tranif1 nmos_2(net_3, y, bb);
  tranif1 nmos_3(net_4, y, b);
  tranif1 nmos_4(gnd, bb, b);
  tranif1 nmos_5(gnd, ab, a);
  tranif0 pmos_0(y, net_7, b);
  tranif0 pmos_1(net_7, vdd, a);
  tranif0 pmos_2(y, net_8, bb);
  tranif0 pmos_3(net_8, vdd, ab);
  tranif0 pmos_4(bb, vdd, b);
  tranif0 pmos_5(ab, vdd, a);
endmodule   /* muddlib07__xnor2_1x */

module comparator(a, b, status);
  input [3:0] a;
  input [3:0] b;
  output status;

  supply1 vdd;
  supply0 gnd;
  wire net_0, net_14, net_4, net_8;

  muddlib07__and4_1x and4_1x_0(.a(net_0), .b(net_4), .c(net_8), .d(net_14), 
      .y(status));
  muddlib07__xnor2_1x xnor2_1x_0(.a(a[3]), .b(b[3]), .y(net_0));
  muddlib07__xnor2_1x xnor2_1x_1(.a(a[2]), .b(b[2]), .y(net_4));
  muddlib07__xnor2_1x xnor2_1x_3(.a(a[1]), .b(b[1]), .y(net_8));
  muddlib07__xnor2_1x xnor2_1x_4(.a(a[0]), .b(b[0]), .y(net_14));
endmodule   /* comparator */
