/* Verilog for cell 'decoder16_1x{sch}' from library 'vlsi' */
/* Created on 星期三 十一月 12, 2014 19:13:28 */
/* Last revised on 星期三 十一月 12, 2014 19:28:06 */
/* Written on 星期三 十一月 12, 2014 19:30:53 by Electric VLSI Design System, version 8.06 */

module muddlib07__inv_1x(a, y);
  input a;
  output y;

  supply1 vdd;
  supply0 gnd;
  tranif1 nmos_0(gnd, y, a);
  tranif0 pmos_0(y, vdd, a);
endmodule   /* muddlib07__inv_1x */

module muddlib07__nand2_1x(a, b, y);
  input a;
  input b;
  output y;

  supply1 vdd;
  supply0 gnd;
  wire net_5;

  tranif1 nmos_0(net_5, y, b);
  tranif1 nmos_1(gnd, net_5, a);
  tranif0 pmos_0(y, vdd, b);
  tranif0 pmos_1(y, vdd, a);
endmodule   /* muddlib07__nand2_1x */

module muddlib07__nor2_1x(a, b, y);
  input a;
  input b;
  output y;

  supply1 vdd;
  supply0 gnd;
  wire net_9;

  tranif1 nmos_0(gnd, y, a);
  tranif1 nmos_1(gnd, y, b);
  tranif0 pmos_0(y, net_9, b);
  tranif0 pmos_1(net_9, vdd, a);
endmodule   /* muddlib07__nor2_1x */

module decoder16_1x(a, y);
  input [3:0] a;
  output [15:0] y;

  supply1 vdd;
  supply0 gnd;
  wire net_16, net_26, net_36, net_38, net_46, net_54, net_6, net_62, net_70;
  wire net_72, net_74, net_76;

  muddlib07__inv_1x inv_1x_0(.a(a[3]), .y(net_6));
  muddlib07__inv_1x inv_1x_1(.a(a[2]), .y(net_16));
  muddlib07__inv_1x inv_1x_2(.a(a[1]), .y(net_26));
  muddlib07__inv_1x inv_1x_3(.a(a[0]), .y(net_36));
  muddlib07__nand2_1x nand2_1x_0(.a(a[2]), .b(a[3]), .y(net_62));
  muddlib07__nand2_1x nand2_1x_2(.a(net_16), .b(a[3]), .y(net_54));
  muddlib07__nand2_1x nand2_1x_3(.a(a[2]), .b(net_6), .y(net_46));
  muddlib07__nand2_1x nand2_1x_4(.a(net_16), .b(net_6), .y(net_38));
  muddlib07__nand2_1x nand2_1x_5(.a(a[0]), .b(a[1]), .y(net_76));
  muddlib07__nand2_1x nand2_1x_6(.a(net_36), .b(a[1]), .y(net_74));
  muddlib07__nand2_1x nand2_1x_7(.a(a[0]), .b(net_26), .y(net_72));
  muddlib07__nand2_1x nand2_1x_8(.a(net_36), .b(net_26), .y(net_70));
  muddlib07__nor2_1x nor2_1x_0(.a(net_70), .b(net_38), .y(y[0]));
  muddlib07__nor2_1x nor2_1x_1(.a(net_72), .b(net_38), .y(y[1]));
  muddlib07__nor2_1x nor2_1x_2(.a(net_74), .b(net_38), .y(y[2]));
  muddlib07__nor2_1x nor2_1x_3(.a(net_76), .b(net_38), .y(y[3]));
  muddlib07__nor2_1x nor2_1x_4(.a(net_74), .b(net_46), .y(y[6]));
  muddlib07__nor2_1x nor2_1x_5(.a(net_76), .b(net_46), .y(y[7]));
  muddlib07__nor2_1x nor2_1x_6(.a(net_70), .b(net_46), .y(y[4]));
  muddlib07__nor2_1x nor2_1x_7(.a(net_72), .b(net_46), .y(y[5]));
  muddlib07__nor2_1x nor2_1x_8(.a(net_70), .b(net_54), .y(y[8]));
  muddlib07__nor2_1x nor2_1x_9(.a(net_72), .b(net_54), .y(y[9]));
  muddlib07__nor2_1x nor2_1x_10(.a(net_74), .b(net_54), .y(y[10]));
  muddlib07__nor2_1x nor2_1x_11(.a(net_76), .b(net_54), .y(y[11]));
  muddlib07__nor2_1x nor2_1x_12(.a(net_74), .b(net_62), .y(y[14]));
  muddlib07__nor2_1x nor2_1x_13(.a(net_76), .b(net_62), .y(y[15]));
  muddlib07__nor2_1x nor2_1x_14(.a(net_70), .b(net_62), .y(y[12]));
  muddlib07__nor2_1x nor2_1x_15(.a(net_72), .b(net_62), .y(y[13]));
endmodule   /* decoder16_1x */
