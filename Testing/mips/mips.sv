//---------------------------------------------------------
// mips.sv
// David_Harris@hmc.edu 23 Jan 2008
// Changes 7/3/07
//   Updated to SystemVerilog
//   Fixed endianness
// 23 Jan 08 added $stop
//
// Model of subset of MIPS processor described in Ch 1
//  note that no sign extension is done because width is
//  only 8 bits
//---------------------

// states and instructions

  typedef enum logic [3:0] {FETCH1 = 4'b0000, FETCH2, FETCH3, FETCH4,
                            DECODE, MEMADR, LBRD, LBWR, SBWR,
                            RTYPEEX, RTYPEWR, BEQEX, JEX} statetype;
  typedef enum logic [5:0] {LB    = 6'b100000,
                            SB    = 6'b101000,
                            RTYPE = 6'b000000,
                            BEQ   = 6'b000100,
                            J     = 6'b000010} opcode;
  typedef enum logic [5:0] {ADD = 6'b100000,
                            SUB = 6'b100010,
                            AND = 6'b100100,
                            OR  = 6'b100101,
                            SLT = 6'b101010} functcode;

// testbench for testing
module testbench #(parameter WIDTH = 8, REGBITS = 3)();

  logic             clk;
  logic             reset;
  logic             memread, memwrite;
  logic [WIDTH-1:0] adr, writedata;
  logic [WIDTH-1:0] memdata;

  // instantiate devices to be tested
  mips #(WIDTH,REGBITS) dut(clk, reset, memdata, memread, 
                            memwrite, adr, writedata);

  // external memory for code and data
  exmemory #(WIDTH) exmem(clk, memwrite, adr, writedata, memdata);

  // initialize test
  initial
    begin
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

  always@(negedge clk)
    begin
      if(memwrite) begin
        assert(adr == 76 & writedata == 7)
          $display("Simulation completely successful");
        else $error("Simulation failed");
        $stop;
      end
    end
	
endmodule

// external memory accessed by MIPS
module exmemory #(parameter WIDTH = 8)
                 (input  logic             clk,
                  input  logic             memwrite,
                  input  logic [WIDTH-1:0] adr, writedata,
                  output logic [WIDTH-1:0] memdata);

  logic [31:0]      mem [2**(WIDTH-2)-1:0];
  logic [31:0]      word;
  logic [1:0]       bytesel;
  logic [WIDTH-2:0] wordadr;

  initial
    $readmemh("memfile.dat", mem);

  assign bytesel = adr[1:0];
  assign wordadr = adr[WIDTH-1:2];

  // read and write bytes from 32-bit word
  always @(posedge clk)
    if(memwrite) 
      case (bytesel)
        2'b00: mem[wordadr][7:0]   <= writedata;
        2'b01: mem[wordadr][15:8]  <= writedata;
        2'b10: mem[wordadr][23:16] <= writedata;
        2'b11: mem[wordadr][31:24] <= writedata;
      endcase

   assign word = mem[wordadr];
   always_comb
     case (bytesel)
       2'b00: memdata = word[7:0];
       2'b01: memdata = word[15:8];
       2'b10: memdata = word[23:16];
       2'b11: memdata = word[31:24];
     endcase
endmodule

// simplified MIPS processor
module mips #(parameter WIDTH = 8, REGBITS = 3)
             (input  logic             clk, reset, 
              input  logic [WIDTH-1:0] memdata, 
              output logic             memread, memwrite, 
              output logic [WIDTH-1:0] adr, writedata);

   logic [31:0] instr;
   logic        zero, alusrca, memtoreg, iord, pcen, regwrite, regdst;
   logic [1:0]  pcsrc, alusrcb;
   logic [3:0]  irwrite;
   logic [2:0]  alucontrol;
   opcode       op;
   functcode    funct;

   logic hit;
   logic cachewrite, morc, morw;
   logic [WIDTH-1:0] cacheout, dpdata, cachedata;
   
   /*
   controller_plabased  cont(clk, reset, hit, op, funct, zero, memread, memwrite, cachewrite, morc, morw,
                    alusrca, memtoreg, iord, pcen, regwrite, regdst,
                    pcsrc, alusrcb, alucontrol, irwrite);
	*/
	
	controller cont(funct[3:0], hit, op, clk, ~clk, reset, zero, alucontrol, alusrca, 
      alusrcb, cachewrite, iord, irwrite, memread, memtoreg, memwrite, morc, 
      morw, pcen, pcsrc, regdst, regwrite);
	  
/*   datapath    #(WIDTH, REGBITS) 
               dp(clk, reset, dpdata, alusrca, memtoreg, iord, pcen,
                  regwrite, regdst, pcsrc, alusrcb, irwrite, alucontrol,
                  zero, op, funct, adr, writedata);
 */
 datapath    dp(alucontrol[0], alucontrol[1], alucontrol[2], alusrca, alusrcb[0], 
      alusrcb[1], iord, irwrite[0], irwrite[1], irwrite[2], irwrite[3], memdata[0], 
      memdata[1], memdata[2], memdata[3], memdata[4], memdata[5], memdata[6], 
      memdata[7], memtoreg, pcen, pcsrc[0], pcsrc[1], clk, ~clk, regdst, regwrite, 
      reset, adr[0], adr[1], adr[2], adr[3], adr[4], adr[5], adr[6], adr[7], funct[0], 
      funct[1], funct[2], funct[3], funct[4], funct[5], op[0], op[1], op[2], op[3], op[4], 
      op[5], writedata[0], writedata[1], writedata[2], writedata[3], writedata[4], 
      writedata[5], writedata[6], writedata[7], zero, vdd, gnd);
				  
	mux2       #(WIDTH)  cachemux(memdata, writedata, morw, cachedata);	
	cache idcache(adr, clk, data_in, cachewrite, data_out, hit);
	mux2       #(WIDTH)  dpmux(memdata, cacheout, morc, dpdata);
endmodule


module controller_plabased(input logic clk, reset, hit, 
                  input  opcode      op,
                  input  functcode   funct,
                  input  logic       zero, 
                  output logic       memread, memwrite, cachewrite, morc, morw, alusrca,  
                  output logic       memtoreg, iord, pcen, 
                  output logic       regwrite, regdst, 
                  output logic [1:0] pcsrc, alusrcb,
                  output logic [2:0] alucontrol,
                  output logic [3:0] irwrite);

  logic            pcwrite, branch;
  logic     [1:0]  aluop;
  logic     [11:0]  in;
  logic     [26:0] out;
  logic     [4:0]  state;
  logic     [4:0]  nextstate;
  
  always_ff @(posedge clk)
     if(reset) state <= 5'b00000;
     else      state <= nextstate;
  
  assign in = {op,state, hit};
  assign {aluop, branch, pcwrite, irwrite, alusrcb, pcsrc, regdst, regwrite, 
          iord, memtoreg, alusrca, memwrite, memread, nextstate, cachewrite, morc, morw} = out;

  aludec  ac(aluop, funct, alucontrol);
  assign pcen = pcwrite | (branch & zero); // program counter enable

  always_comb 
    casez(in)
	
	
      12'b??????00000?: out <= 27'b000000000000000000010000000;
      12'b??????100000: out <= 27'b000100010100000000100001100;
      12'b??????100001: out <= 27'b000100010100000000100001010;
      12'b??????00001?: out <= 27'b000000000000000000010001000;
      12'b??????100010: out <= 27'b000100100100000000100010100;
      12'b??????100011: out <= 27'b000100100100000000100010010;
      12'b??????00010?: out <= 27'b000000000000000000010010000;
      12'b??????100100: out <= 27'b000101000100000000100011100;
      12'b??????100101: out <= 27'b000101000100000000100011010;
      12'b??????00011?: out <= 27'b000000000000000000010011000;	
      12'b??????100110: out <= 27'b000110000100000000100100100;	
      12'b??????100111: out <= 27'b000110000100000000100100010;
	  
      12'b10000000100?: out <= 27'b000000001100000000000101000;
      12'b10100000100?: out <= 27'b000000001100000000000101000;
      12'b00000000100?: out <= 27'b000000001100000000001001000;
      12'b00010000100?: out <= 27'b000000001100000000001011000;
      12'b00001000100?: out <= 27'b000000001100000000001100000;
      12'b10000000101?: out <= 27'b000000001000000010000110000;
      12'b10100000101?: out <= 27'b000000001000000010001000000;

	  12'b??????00110?: out <= 27'b000000001000000010010110000;
      12'b??????101100: out <= 27'b000000000000001000100111100;
      12'b??????101101: out <= 27'b000000000000001000100111010;

      12'b??????00111?: out <= 27'b000000000000010100000000000;
      12'b??????01000?: out <= 27'b000000000000001001000000100;
      12'b??????01001?: out <= 27'b100000000000000010001010000;
      12'b??????01010?: out <= 27'b000000000000110000000000000;
	  12'b??????01011?: out <= 27'b011000000001000010000000000;
      12'b??????01100?: out <= 27'b000100000010000000000000000;
	  
      default:        	out <= 27'bxxxxxxxxxxxxxxxxxxxxxxxxxxx;
	  
    endcase

endmodule

module aludec(input  logic [1:0] aluop, 
              input  logic [5:0] funct, 
              output logic [2:0] alucontrol);

  always_comb
    case (aluop)
      2'b00: alucontrol = 3'b010;  // add for lb/sb/addi
      2'b01: alucontrol = 3'b110;  // subtract (for beq)
      default: case(funct)      // R-Type instructions
                 ADD: alucontrol = 3'b010;
                 SUB: alucontrol = 3'b110;
                 AND: alucontrol = 3'b000;
                 OR:  alucontrol = 3'b001;
                 SLT: alucontrol = 3'b111;
                 default:   alucontrol = 3'b101; // should never happen
               endcase
    endcase
endmodule

module datapath0 #(parameter WIDTH = 8, REGBITS = 3)
                 (input  logic             clk, reset, 
                  input  logic [WIDTH-1:0] memdata, 
                  input  logic             alusrca, memtoreg, iord, 
                  input  logic             pcen, regwrite, regdst,
                  input  logic [1:0]       pcsrc, alusrcb, 
                  input  logic [3:0]       irwrite, 
                  input  logic [2:0]       alucontrol, 
                  output logic             zero,
                  output opcode            op,
                  output functcode         funct,
                  output logic [WIDTH-1:0] adr, writedata);

  logic [REGBITS-1:0] ra1, ra2, wa;
  logic [WIDTH-1:0]   pc, nextpc, data, rd1, rd2, wd, a, srca, 
                      srcb, aluresult, aluout, immx4;
  logic [31:0]        instr;

  logic [WIDTH-1:0] CONST_ZERO = 0;
  logic [WIDTH-1:0] CONST_ONE =  1;
  
  assign op = opcode'(instr[31:26]);
  assign funct = functcode'(instr[5:0]);

  // shift left immediate field by 2
  assign immx4 = {instr[WIDTH-3:0],2'b00};

  // register file address fields
  assign ra1 = instr[REGBITS+20:21];
  assign ra2 = instr[REGBITS+15:16];
  mux2       #(REGBITS) regmux(instr[REGBITS+15:16], 
                               instr[REGBITS+10:11], regdst, wa);

   // independent of bit width, load instruction into four 8-bit registers over four cycles
  flopen     #(8)      ir0(clk, irwrite[0], memdata[7:0], instr[7:0]);
  flopen     #(8)      ir1(clk, irwrite[1], memdata[7:0], instr[15:8]);
  flopen     #(8)      ir2(clk, irwrite[2], memdata[7:0], instr[23:16]);
  flopen     #(8)      ir3(clk, irwrite[3], memdata[7:0], instr[31:24]);

  // datapath
  flopenr    #(WIDTH)  pcreg(clk, reset, pcen, nextpc, pc);
  flop       #(WIDTH)  datareg(clk, memdata, data);
  flop       #(WIDTH)  areg(clk, rd1, a);
  flop       #(WIDTH)  wrdreg(clk, rd2, writedata);
  flop       #(WIDTH)  resreg(clk, aluresult, aluout);
  mux2       #(WIDTH)  adrmux(pc, aluout, iord, adr);
  mux2       #(WIDTH)  src1mux(pc, a, alusrca, srca);
  mux4       #(WIDTH)  src2mux(writedata, CONST_ONE, instr[WIDTH-1:0], 
                               immx4, alusrcb, srcb);
  mux3       #(WIDTH)  pcmux(aluresult, aluout, immx4, 
                             pcsrc, nextpc);
  mux2       #(WIDTH)  wdmux(aluout, data, memtoreg, wd);
  regfile    #(WIDTH,REGBITS) rf(clk, regwrite, ra1, ra2, 
                                 wa, wd, rd1, rd2);
  alu        #(WIDTH) alunit(srca, srcb, alucontrol, aluresult, zero);
endmodule

module alu #(parameter WIDTH = 8)
            (input  logic [WIDTH-1:0] a, b, 
             input  logic [2:0]       alucontrol, 
             output logic [WIDTH-1:0] result,
             output logic             zero);

  logic [WIDTH-1:0] b2, andresult, orresult, sumresult, sltresult;

  andN    andblock(a, b, andresult);
  orN     orblock(a, b, orresult);
  condinv binv(b, alucontrol[2], b2);
  adder   addblock(a, b2, alucontrol[2], sumresult);
  // slt should be 1 if most significant bit of sum is 1
  assign sltresult = sumresult[WIDTH-1];

  mux4 resultmux(andresult, orresult, sumresult, sltresult, alucontrol[1:0], result);
  zerodetect #(WIDTH) zd(result, zero);
endmodule

module regfile #(parameter WIDTH = 8, REGBITS = 3)
                (input  logic               clk, 
                 input  logic               regwrite, 
                 input  logic [REGBITS-1:0] ra1, ra2, wa, 
                 input  logic [WIDTH-1:0]   wd, 
                 output logic [WIDTH-1:0]   rd1, rd2);

   logic [WIDTH-1:0] RAM [2**REGBITS-1:0];

  // three ported register file
  // read two ports combinationally
  // write third port on rising edge of clock
  // register 0 hardwired to 0
  always @(posedge clk)
    if (regwrite) RAM[wa] <= wd;

  assign rd1 = ra1 ? RAM[ra1] : 0;
  assign rd2 = ra2 ? RAM[ra2] : 0;
endmodule

module zerodetect #(parameter WIDTH = 8)
                   (input  logic [WIDTH-1:0] a, 
                    output logic             y);

   assign y = (a==0);
endmodule	

module flop #(parameter WIDTH = 8)
             (input  logic             clk, 
              input  logic [WIDTH-1:0] d, 
              output logic [WIDTH-1:0] q);

  always_ff @(posedge clk)
    q <= d;
endmodule

module flopen #(parameter WIDTH = 8)
               (input  logic             clk, en,
                input  logic [WIDTH-1:0] d, 
                output logic [WIDTH-1:0] q);

  always_ff @(posedge clk)
    if (en) q <= d;
endmodule

module flopenr #(parameter WIDTH = 8)
                (input  logic             clk, reset, en,
                 input  logic [WIDTH-1:0] d, 
                 output logic [WIDTH-1:0] q);
 
  always_ff @(posedge clk)
    if      (reset) q <= 0;
    else if (en)    q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  always_comb 
    casez (s)
      2'b00: y = d0;
      2'b01: y = d1;
      2'b1?: y = d2;
    endcase
endmodule

module mux4 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2, d3,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  always_comb
    case (s)
      2'b00: y = d0;
      2'b01: y = d1;
      2'b10: y = d2;
      2'b11: y = d3;
    endcase
endmodule

module andN #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] a, b,
              output logic [WIDTH-1:0] y);

  assign y = a & b;
endmodule

module orN #(parameter WIDTH = 8)
            (input  logic [WIDTH-1:0] a, b,
             output logic [WIDTH-1:0] y);

  assign y = a | b;
endmodule

module inv #(parameter WIDTH = 8)
            (input  logic [WIDTH-1:0] a,
             output logic [WIDTH-1:0] y);

  assign y = ~a;
endmodule

module condinv #(parameter WIDTH = 8)
                (input  logic [WIDTH-1:0] a,
                 input  logic             invert,
                 output logic [WIDTH-1:0] y);

  logic [WIDTH-1:0] ab;

  inv  inverter(a, ab);
  mux2 invmux(a, ab, invert, y);
endmodule

module adder #(parameter WIDTH = 8)
              (input  logic [WIDTH-1:0] a, b,
               input  logic             cin,
               output logic [WIDTH-1:0] y);

  assign y = a + b + cin;
endmodule

module cache0(clk, we, addr, data_in, data_out, hit);
	input logic clk;
	input logic we;
	input logic [7:0] addr;
	input logic [7:0] data_in;
	output logic [7:0] data_out;
	output logic hit;
	
	logic [15:0] wl;
	decoder dec(addr[3:0], wl);
	//vlsi__decoder16_1x dec(addr[3:0], wl);

	
	logic [3:0] tag_in;
	assign tag_in = addr[7:4];
	
	logic [3:0] tag_out;
	sram block(clk, we, wl, tag_in, data_in, tag_out, data_out);
	
	assign hit = (tag_in===tag_out)?1:0;

endmodule


module sram(clk, we, wl, tag_in, data_in, tag_out, data_out);
	input logic clk;
	input logic we;
	input logic [15:0] wl;
	input logic [3:0] tag_in;
	input logic [7:0] data_in;
	output logic [3:0] tag_out;
	output logic [7:0] data_out;

	// Internal storage element
	// data
	logic [7:0] ramD[15:0];
	// tag
	logic [3:0] ramT[15:0];
	
	// temp address
	logic [3:0] addr;
	
	encoder enc(wl, addr);
	
	always @ (posedge clk)
	begin
		// Write
		if (we)
		begin
			ramT[addr] <= tag_in;		
			ramD[addr] <= data_in;
		end
	end
		
	assign	tag_out = ramT[addr];
	assign	data_out = ramD[addr];
	
endmodule


module encoder(input logic [15:0] in, output logic [3:0] out);    
	always_comb
	  casez (in) 
		  16'b0000000000000001 : out = 0; 
		  16'b0000000000000010 : out = 1; 
		  16'b0000000000000100 : out = 2; 
		  16'b0000000000001000 : out = 3; 
		  16'b0000000000010000 : out = 4; 
		  16'b0000000000100000 : out = 5; 
		  16'b0000000001000000 : out = 6; 
		  16'b0000000010000000 : out = 7; 
		  16'b0000000100000000 : out = 8; 
		  16'b0000001000000000 : out = 9; 
		  16'b0000010000000000 : out = 10; 
		  16'b0000100000000000 : out = 11; 
		  16'b0001000000000000 : out = 12; 
		  16'b0010000000000000 : out = 13; 
		  16'b0100000000000000 : out = 14; 
		  16'b1000000000000000 : out = 15; 
	  default: out = 4'bx;
	  endcase
endmodule

module decoder(input logic [3:0] in, output logic [15:0] out);
	always_comb
	  casez (in) 
		  0 : out = 16'b0000000000000001; 
		  1 : out = 16'b0000000000000010; 
		  2 : out = 16'b0000000000000100; 
		  3 : out = 16'b0000000000001000; 
		  4 : out = 16'b0000000000010000; 
		  5 : out = 16'b0000000000100000; 
		  6 : out = 16'b0000000001000000; 
		  7 : out = 16'b0000000010000000; 
		  8 : out = 16'b0000000100000000; 
		  9 : out = 16'b0000001000000000; 
		  10 : out = 16'b0000010000000000; 
		  11 : out = 16'b0000100000000000; 
		  12 : out = 16'b0001000000000000; 
		  13 : out = 16'b0010000000000000; 
		  14 : out = 16'b0100000000000000; 
		  15 : out = 16'b1000000000000000; 
	  default: out = 16'bx;
	  endcase
endmodule

/* Verilog for cell 'controller{sch}' from library 'mips8' */
/* Created on Tue Jul 17, 2007 15:16:24 */
/* Last revised on Fri Nov 21, 2014 11:24:14 */
/* Written on Fri Nov 21, 2014 11:24:27 by Electric VLSI Design System, version 8.06 */

module muddlib07__a2o1_1x(a, b, c, y);
  input a;
  input b;
  input c;
  output y;

  supply1 vdd;
  supply0 gnd;
  wire net_0, net_11, net_19;

  tranif1 nmos_0(gnd, net_19, a);
  tranif1 nmos_1(net_19, net_0, b);
  tranif1 nmos_2(gnd, net_0, c);
  tranif1 nmos_3(gnd, y, net_0);
  tranif0 pmos_0(net_0, net_11, c);
  tranif0 pmos_1(net_11, vdd, b);
  tranif0 pmos_2(net_11, vdd, a);
  tranif0 pmos_3(y, vdd, net_0);
endmodule   /* muddlib07__a2o1_1x */

module muddlib07__inv_1x(a, y);
  input a;
  output y;

  supply1 vdd;
  supply0 gnd;
  tranif1 nmos_0(gnd, y, a);
  tranif0 pmos_0(y, vdd, a);
endmodule   /* muddlib07__inv_1x */

module muddlib07__mux4_c_2x(d0, d1, d2, d3, s0, s1, y);
  input d0;
  input d1;
  input d2;
  input d3;
  input s0;
  input s1;
  output y;

  supply1 vdd;
  supply0 gnd;
  wire net_1, net_17, net_18, net_21, net_29, net_30, net_47, net_49, net_54;
  wire net_55, net_59, net_68, net_70, s0b, s1b;

  tranif1 nmos_0(gnd, net_47, d0);
  tranif1 nmos_1(gnd, net_59, d1);
  tranif1 nmos_2(gnd, net_70, d3);
  tranif1 nmos_3(net_68, net_49, s0b);
  tranif1 nmos_4(net_70, net_49, s0);
  tranif1 nmos_5(net_49, net_17, s1);
  tranif1 nmos_6(gnd, net_68, d2);
  tranif1 nmos_7(gnd, y, net_17);
  tranif1 nmos_8(gnd, s0b, s0);
  tranif1 nmos_9(gnd, s1b, s1);
  tranif1 nmos_10(net_47, net_1, s0b);
  tranif1 nmos_11(net_59, net_1, s0);
  tranif1 nmos_12(net_1, net_17, s1b);
  tranif0 pmos_0(net_17, net_18, s1);
  tranif0 pmos_1(net_55, vdd, d3);
  tranif0 pmos_2(net_21, net_54, s0);
  tranif0 pmos_3(net_54, vdd, d2);
  tranif0 pmos_4(net_21, net_55, s0b);
  tranif0 pmos_5(net_17, net_21, s1b);
  tranif0 pmos_6(y, vdd, net_17);
  tranif0 pmos_7(s0b, vdd, s0);
  tranif0 pmos_8(s1b, vdd, s1);
  tranif0 pmos_9(net_18, net_29, s0);
  tranif0 pmos_10(net_29, vdd, d0);
  tranif0 pmos_11(net_18, net_30, s0b);
  tranif0 pmos_12(net_30, vdd, d1);
endmodule   /* muddlib07__mux4_c_2x */

module muddlib07__or2_2x(a, b, y);
  input a;
  input b;
  output y;

  supply1 vdd;
  supply0 gnd;
  wire net_0, net_11;

  tranif1 nmos_0(gnd, y, net_0);
  tranif1 nmos_1(gnd, net_0, a);
  tranif1 nmos_2(gnd, net_0, b);
  tranif0 pmos_0(net_0, net_11, b);
  tranif0 pmos_1(net_11, vdd, a);
  tranif0 pmos_2(y, vdd, net_0);
endmodule   /* muddlib07__or2_2x */

module mips8__aludec(ALUOp, Funct, alucontrol);
  input [1:0] ALUOp;
  input [3:0] Funct;
  output [2:0] alucontrol;

  supply1 vdd;
  supply0 gnd;
  wire net_42, net_50, net_72;

  muddlib07__inv_1x inv_1x_0(.a(Funct[2]), .y(net_42));
  muddlib07__inv_1x inv_1x_1(.a(ALUOp[1]), .y(net_72));
  muddlib07__mux4_c_2x mux4_c_2_0(.d0(ALUOp[1]), .d1(ALUOp[1]), .d2(net_50), 
      .d3(net_50), .s0(ALUOp[0]), .s1(ALUOp[1]), .y(alucontrol[0]));
  muddlib07__mux4_c_2x mux4_c_2_1(.d0(net_72), .d1(net_72), .d2(net_42), 
      .d3(net_42), .s0(ALUOp[0]), .s1(ALUOp[1]), .y(alucontrol[1]));
  muddlib07__mux4_c_2x mux4_c_2_2(.d0(ALUOp[1]), .d1(ALUOp[0]), .d2(Funct[1]), 
      .d3(Funct[1]), .s0(ALUOp[0]), .s1(ALUOp[1]), .y(alucontrol[2]));
  muddlib07__or2_2x or2_2x_1(.a(Funct[0]), .b(Funct[3]), .y(net_50));
endmodule   /* mips8__aludec */

module mips8__controller_pla_Cell(in, in_1, in_10, in_11, in_2, in_3, in_4, 
      in_5, in_6, in_7, in_8, in_9, out, out_1, out_10, out_11, out_12, out_13, 
      out_14, out_15, out_16, out_17, out_18, out_19, out_2, out_20, out_21, 
      out_22, out_23, out_24, out_25, out_26, out_3, out_4, out_5, out_6, 
      out_7, out_8, out_9);
  input [0:0] in;
  input [1:1] in_1;
  input [10:10] in_10;
  input [11:11] in_11;
  input [2:2] in_2;
  input [3:3] in_3;
  input [4:4] in_4;
  input [5:5] in_5;
  input [6:6] in_6;
  input [7:7] in_7;
  input [8:8] in_8;
  input [9:9] in_9;
  output [0:0] out;
  output [1:1] out_1;
  output [10:10] out_10;
  output [11:11] out_11;
  output [12:12] out_12;
  output [13:13] out_13;
  output [14:14] out_14;
  output [15:15] out_15;
  output [16:16] out_16;
  output [17:17] out_17;
  output [18:18] out_18;
  output [19:19] out_19;
  output [2:2] out_2;
  output [20:20] out_20;
  output [21:21] out_21;
  output [22:22] out_22;
  output [23:23] out_23;
  output [24:24] out_24;
  output [25:25] out_25;
  output [26:26] out_26;
  output [3:3] out_3;
  output [4:4] out_4;
  output [5:5] out_5;
  output [6:6] out_6;
  output [7:7] out_7;
  output [8:8] out_8;
  output [9:9] out_9;

  supply1 vdd;
  supply0 gnd;
  wire net_0, net_1150, net_1160, net_1163, net_1166, net_1169, net_1172;
  wire net_1181, net_1201, net_1204, net_1210, net_1218, net_1229, net_1235;
  wire net_1241, net_1251, net_1264, net_1277, net_1284, net_14, net_1465;
  wire net_1473, net_1481, net_1545, net_1553, net_1561, net_1585, net_1601;
  wire net_1673, net_1980, net_1981, net_21, net_28, net_301, net_312, net_336;
  wire net_347, net_371, net_382, net_406, net_419, net_42, net_448, net_459;
  wire net_49, net_507, net_530, net_56, net_584, net_607, net_63, net_661;
  wire net_684, net_70, net_738, net_751, net_77, net_780, net_791, net_819;
  wire net_832, net_858, net_871, net_900, net_911, net_939, net_952;

  tranif1 nmos_3(gnd, net_0, in_11[11]);
  tranif1 nmos_10(gnd, net_1980, in_10[10]);
  tranif1 nmos_17(gnd, net_14, in_9[9]);
  tranif1 nmos_24(gnd, net_21, in_8[8]);
  tranif1 nmos_31(gnd, net_28, in_7[7]);
  tranif1 nmos_38(gnd, net_1981, in_6[6]);
  tranif1 nmos_45(gnd, net_42, in_5[5]);
  tranif1 nmos_52(gnd, net_49, in_4[4]);
  tranif1 nmos_59(gnd, net_56, in_3[3]);
  tranif1 nmos_66(gnd, net_63, in_2[2]);
  tranif1 nmos_73(gnd, net_70, in_1[1]);
  tranif1 nmos_80(gnd, net_77, in[0]);
  tranif1 nmos_86(net_301, gnd, in_5[5]);
  tranif1 nmos_87(net_301, gnd, net_49);
  tranif1 nmos_88(net_301, gnd, net_56);
  tranif1 nmos_89(net_301, gnd, in_2[2]);
  tranif1 nmos_90(net_301, gnd, in_1[1]);
  tranif1 nmos_91(gnd, net_312, in_5[5]);
  tranif1 nmos_92(gnd, net_312, net_49);
  tranif1 nmos_93(gnd, net_312, in_3[3]);
  tranif1 nmos_94(gnd, net_312, net_63);
  tranif1 nmos_95(gnd, net_312, net_70);
  tranif1 nmos_96(net_336, gnd, in_5[5]);
  tranif1 nmos_97(net_336, gnd, net_49);
  tranif1 nmos_98(net_336, gnd, in_3[3]);
  tranif1 nmos_99(net_336, gnd, net_63);
  tranif1 nmos_100(net_336, gnd, in_1[1]);
  tranif1 nmos_101(gnd, net_347, in_5[5]);
  tranif1 nmos_102(gnd, net_347, net_49);
  tranif1 nmos_103(gnd, net_347, in_3[3]);
  tranif1 nmos_104(gnd, net_347, in_2[2]);
  tranif1 nmos_105(gnd, net_347, net_70);
  tranif1 nmos_106(net_371, gnd, in_5[5]);
  tranif1 nmos_107(net_371, gnd, net_49);
  tranif1 nmos_108(net_371, gnd, in_3[3]);
  tranif1 nmos_109(net_371, gnd, in_2[2]);
  tranif1 nmos_110(net_371, gnd, in_1[1]);
  tranif1 nmos_111(gnd, net_382, in_5[5]);
  tranif1 nmos_112(gnd, net_382, in_4[4]);
  tranif1 nmos_113(gnd, net_382, net_56);
  tranif1 nmos_114(gnd, net_382, net_63);
  tranif1 nmos_115(gnd, net_382, net_70);
  tranif1 nmos_116(net_406, gnd, net_42);
  tranif1 nmos_117(net_406, gnd, in_4[4]);
  tranif1 nmos_118(net_406, gnd, net_56);
  tranif1 nmos_119(net_406, gnd, net_63);
  tranif1 nmos_120(net_406, gnd, in_1[1]);
  tranif1 nmos_121(net_406, gnd, net_77);
  tranif1 nmos_122(gnd, net_419, net_42);
  tranif1 nmos_123(gnd, net_419, in_4[4]);
  tranif1 nmos_124(gnd, net_419, net_56);
  tranif1 nmos_125(gnd, net_419, net_63);
  tranif1 nmos_126(gnd, net_419, in_1[1]);
  tranif1 nmos_127(gnd, net_419, in[0]);
  tranif1 nmos_128(net_448, gnd, in_5[5]);
  tranif1 nmos_129(net_448, gnd, in_4[4]);
  tranif1 nmos_130(net_448, gnd, net_56);
  tranif1 nmos_131(net_448, gnd, net_63);
  tranif1 nmos_132(net_448, gnd, in_1[1]);
  tranif1 nmos_133(gnd, net_459, net_0);
  tranif1 nmos_134(gnd, net_459, in_10[10]);
  tranif1 nmos_135(gnd, net_459, net_14);
  tranif1 nmos_136(gnd, net_459, in_8[8]);
  tranif1 nmos_137(gnd, net_459, in_7[7]);
  tranif1 nmos_138(gnd, net_459, in_6[6]);
  tranif1 nmos_139(gnd, net_459, in_5[5]);
  tranif1 nmos_140(gnd, net_459, in_4[4]);
  tranif1 nmos_141(gnd, net_459, net_56);
  tranif1 nmos_142(gnd, net_459, in_2[2]);
  tranif1 nmos_143(gnd, net_459, net_70);
  tranif1 nmos_144(net_507, gnd, net_0);
  tranif1 nmos_145(net_507, gnd, in_10[10]);
  tranif1 nmos_146(net_507, gnd, in_9[9]);
  tranif1 nmos_147(net_507, gnd, in_8[8]);
  tranif1 nmos_148(net_507, gnd, in_7[7]);
  tranif1 nmos_149(net_507, gnd, in_6[6]);
  tranif1 nmos_150(net_507, gnd, in_5[5]);
  tranif1 nmos_151(net_507, gnd, in_4[4]);
  tranif1 nmos_152(net_507, gnd, net_56);
  tranif1 nmos_153(net_507, gnd, in_2[2]);
  tranif1 nmos_154(net_507, gnd, net_70);
  tranif1 nmos_155(gnd, net_530, in_11[11]);
  tranif1 nmos_156(gnd, net_530, in_10[10]);
  tranif1 nmos_157(gnd, net_530, in_9[9]);
  tranif1 nmos_158(gnd, net_530, in_8[8]);
  tranif1 nmos_159(gnd, net_530, net_28);
  tranif1 nmos_160(gnd, net_530, in_6[6]);
  tranif1 nmos_161(gnd, net_530, in_5[5]);
  tranif1 nmos_162(gnd, net_530, in_4[4]);
  tranif1 nmos_163(gnd, net_530, net_56);
  tranif1 nmos_164(gnd, net_530, in_2[2]);
  tranif1 nmos_165(gnd, net_530, in_1[1]);
  tranif1 nmos_166(net_584, gnd, in_11[11]);
  tranif1 nmos_167(net_584, gnd, in_10[10]);
  tranif1 nmos_168(net_584, gnd, in_9[9]);
  tranif1 nmos_169(net_584, gnd, net_21);
  tranif1 nmos_170(net_584, gnd, in_7[7]);
  tranif1 nmos_171(net_584, gnd, in_6[6]);
  tranif1 nmos_172(net_584, gnd, in_5[5]);
  tranif1 nmos_173(net_584, gnd, in_4[4]);
  tranif1 nmos_174(net_584, gnd, net_56);
  tranif1 nmos_175(net_584, gnd, in_2[2]);
  tranif1 nmos_176(net_584, gnd, in_1[1]);
  tranif1 nmos_177(gnd, net_607, in_11[11]);
  tranif1 nmos_178(gnd, net_607, in_10[10]);
  tranif1 nmos_179(gnd, net_607, in_9[9]);
  tranif1 nmos_180(gnd, net_607, in_8[8]);
  tranif1 nmos_181(gnd, net_607, in_7[7]);
  tranif1 nmos_182(gnd, net_607, in_6[6]);
  tranif1 nmos_183(gnd, net_607, in_5[5]);
  tranif1 nmos_184(gnd, net_607, in_4[4]);
  tranif1 nmos_185(gnd, net_607, net_56);
  tranif1 nmos_186(gnd, net_607, in_2[2]);
  tranif1 nmos_187(gnd, net_607, in_1[1]);
  tranif1 nmos_188(net_661, gnd, net_0);
  tranif1 nmos_189(net_661, gnd, in_10[10]);
  tranif1 nmos_190(net_661, gnd, net_14);
  tranif1 nmos_191(net_661, gnd, in_8[8]);
  tranif1 nmos_192(net_661, gnd, in_7[7]);
  tranif1 nmos_193(net_661, gnd, in_6[6]);
  tranif1 nmos_194(net_661, gnd, in_5[5]);
  tranif1 nmos_195(net_661, gnd, in_4[4]);
  tranif1 nmos_196(net_661, gnd, net_56);
  tranif1 nmos_197(net_661, gnd, in_2[2]);
  tranif1 nmos_198(net_661, gnd, in_1[1]);
  tranif1 nmos_199(gnd, net_684, net_0);
  tranif1 nmos_200(gnd, net_684, in_10[10]);
  tranif1 nmos_201(gnd, net_684, in_9[9]);
  tranif1 nmos_202(gnd, net_684, in_8[8]);
  tranif1 nmos_203(gnd, net_684, in_7[7]);
  tranif1 nmos_204(gnd, net_684, in_6[6]);
  tranif1 nmos_205(gnd, net_684, in_5[5]);
  tranif1 nmos_206(gnd, net_684, in_4[4]);
  tranif1 nmos_207(gnd, net_684, net_56);
  tranif1 nmos_208(gnd, net_684, in_2[2]);
  tranif1 nmos_209(gnd, net_684, in_1[1]);
  tranif1 nmos_210(net_738, gnd, net_42);
  tranif1 nmos_211(net_738, gnd, in_4[4]);
  tranif1 nmos_212(net_738, gnd, in_3[3]);
  tranif1 nmos_213(net_738, gnd, net_63);
  tranif1 nmos_214(net_738, gnd, net_70);
  tranif1 nmos_215(net_738, gnd, net_77);
  tranif1 nmos_216(gnd, net_751, net_42);
  tranif1 nmos_217(gnd, net_751, in_4[4]);
  tranif1 nmos_218(gnd, net_751, in_3[3]);
  tranif1 nmos_219(gnd, net_751, net_63);
  tranif1 nmos_220(gnd, net_751, net_70);
  tranif1 nmos_221(gnd, net_751, in[0]);
  tranif1 nmos_222(net_780, gnd, in_5[5]);
  tranif1 nmos_223(net_780, gnd, in_4[4]);
  tranif1 nmos_224(net_780, gnd, in_3[3]);
  tranif1 nmos_225(net_780, gnd, net_63);
  tranif1 nmos_226(net_780, gnd, net_70);
  tranif1 nmos_227(gnd, net_791, net_42);
  tranif1 nmos_228(gnd, net_791, in_4[4]);
  tranif1 nmos_229(gnd, net_791, in_3[3]);
  tranif1 nmos_230(gnd, net_791, net_63);
  tranif1 nmos_231(gnd, net_791, in_1[1]);
  tranif1 nmos_232(gnd, net_791, net_77);
  tranif1 nmos_233(net_819, gnd, net_42);
  tranif1 nmos_234(net_819, gnd, in_4[4]);
  tranif1 nmos_235(net_819, gnd, in_3[3]);
  tranif1 nmos_236(net_819, gnd, net_63);
  tranif1 nmos_237(net_819, gnd, in_1[1]);
  tranif1 nmos_238(net_819, gnd, in[0]);
  tranif1 nmos_239(gnd, net_832, in_5[5]);
  tranif1 nmos_240(gnd, net_832, in_4[4]);
  tranif1 nmos_241(gnd, net_832, in_3[3]);
  tranif1 nmos_242(gnd, net_832, net_63);
  tranif1 nmos_243(gnd, net_832, in_1[1]);
  tranif1 nmos_244(net_858, gnd, net_42);
  tranif1 nmos_245(net_858, gnd, in_4[4]);
  tranif1 nmos_246(net_858, gnd, in_3[3]);
  tranif1 nmos_247(net_858, gnd, in_2[2]);
  tranif1 nmos_248(net_858, gnd, net_70);
  tranif1 nmos_249(net_858, gnd, net_77);
  tranif1 nmos_250(gnd, net_871, net_42);
  tranif1 nmos_251(gnd, net_871, in_4[4]);
  tranif1 nmos_252(gnd, net_871, in_3[3]);
  tranif1 nmos_253(gnd, net_871, in_2[2]);
  tranif1 nmos_254(gnd, net_871, net_70);
  tranif1 nmos_255(gnd, net_871, in[0]);
  tranif1 nmos_256(net_900, gnd, in_5[5]);
  tranif1 nmos_257(net_900, gnd, in_4[4]);
  tranif1 nmos_258(net_900, gnd, in_3[3]);
  tranif1 nmos_259(net_900, gnd, in_2[2]);
  tranif1 nmos_260(net_900, gnd, net_70);
  tranif1 nmos_261(gnd, net_911, net_42);
  tranif1 nmos_262(gnd, net_911, in_4[4]);
  tranif1 nmos_263(gnd, net_911, in_3[3]);
  tranif1 nmos_264(gnd, net_911, in_2[2]);
  tranif1 nmos_265(gnd, net_911, in_1[1]);
  tranif1 nmos_266(gnd, net_911, net_77);
  tranif1 nmos_267(net_939, gnd, net_42);
  tranif1 nmos_268(net_939, gnd, in_4[4]);
  tranif1 nmos_269(net_939, gnd, in_3[3]);
  tranif1 nmos_270(net_939, gnd, in_2[2]);
  tranif1 nmos_271(net_939, gnd, in_1[1]);
  tranif1 nmos_272(net_939, gnd, in[0]);
  tranif1 nmos_273(gnd, net_952, in_5[5]);
  tranif1 nmos_274(gnd, net_952, in_4[4]);
  tranif1 nmos_275(gnd, net_952, in_3[3]);
  tranif1 nmos_276(gnd, net_952, in_2[2]);
  tranif1 nmos_277(gnd, net_952, in_1[1]);
  tranif1 nmos_646(gnd, net_1150, net_301);
  tranif1 nmos_647(gnd, net_1545, net_301);
  tranif1 nmos_648(gnd, net_1473, net_312);
  tranif1 nmos_649(gnd, net_1481, net_312);
  tranif1 nmos_650(gnd, net_1553, net_312);
  tranif1 nmos_651(gnd, net_1210, net_312);
  tranif1 nmos_652(gnd, net_1561, net_336);
  tranif1 nmos_653(gnd, net_1201, net_336);
  tranif1 nmos_654(gnd, net_1465, net_347);
  tranif1 nmos_655(gnd, net_1210, net_347);
  tranif1 nmos_656(gnd, net_1235, net_347);
  tranif1 nmos_657(gnd, net_1251, net_347);
  tranif1 nmos_658(gnd, net_1204, net_371);
  tranif1 nmos_659(gnd, net_1601, net_371);
  tranif1 nmos_660(gnd, net_1277, net_371);
  tranif1 nmos_661(gnd, net_1201, net_382);
  tranif1 nmos_662(gnd, net_1585, net_382);
  tranif1 nmos_663(gnd, net_1204, net_406);
  tranif1 nmos_664(gnd, net_1218, net_406);
  tranif1 nmos_665(gnd, net_1241, net_406);
  tranif1 nmos_666(gnd, net_1251, net_406);
  tranif1 nmos_667(gnd, net_1264, net_406);
  tranif1 nmos_668(gnd, net_1284, net_406);
  tranif1 nmos_669(gnd, net_1204, net_419);
  tranif1 nmos_670(gnd, net_1218, net_419);
  tranif1 nmos_671(gnd, net_1241, net_419);
  tranif1 nmos_672(gnd, net_1251, net_419);
  tranif1 nmos_673(gnd, net_1264, net_419);
  tranif1 nmos_674(gnd, net_1277, net_419);
  tranif1 nmos_675(gnd, net_1172, net_448);
  tranif1 nmos_676(gnd, net_1210, net_448);
  tranif1 nmos_677(gnd, net_1229, net_448);
  tranif1 nmos_678(gnd, net_1241, net_448);
  tranif1 nmos_679(gnd, net_1251, net_448);
  tranif1 nmos_680(gnd, net_1172, net_459);
  tranif1 nmos_681(gnd, net_1210, net_459);
  tranif1 nmos_682(gnd, net_1235, net_459);
  tranif1 nmos_683(gnd, net_1172, net_507);
  tranif1 nmos_684(gnd, net_1210, net_507);
  tranif1 nmos_685(gnd, net_1241, net_507);
  tranif1 nmos_686(gnd, net_1251, net_507);
  tranif1 nmos_687(gnd, net_1172, net_530);
  tranif1 nmos_688(gnd, net_1181, net_530);
  tranif1 nmos_689(gnd, net_1235, net_530);
  tranif1 nmos_690(gnd, net_1241, net_530);
  tranif1 nmos_691(gnd, net_1172, net_584);
  tranif1 nmos_692(gnd, net_1181, net_584);
  tranif1 nmos_693(gnd, net_1235, net_584);
  tranif1 nmos_694(gnd, net_1251, net_584);
  tranif1 nmos_695(gnd, net_1264, net_584);
  tranif1 nmos_696(gnd, net_1172, net_607);
  tranif1 nmos_697(gnd, net_1181, net_607);
  tranif1 nmos_698(gnd, net_1235, net_607);
  tranif1 nmos_699(gnd, net_1264, net_607);
  tranif1 nmos_700(gnd, net_1172, net_661);
  tranif1 nmos_701(gnd, net_1181, net_661);
  tranif1 nmos_702(gnd, net_1241, net_661);
  tranif1 nmos_703(gnd, net_1264, net_661);
  tranif1 nmos_704(gnd, net_1172, net_684);
  tranif1 nmos_705(gnd, net_1181, net_684);
  tranif1 nmos_706(gnd, net_1241, net_684);
  tranif1 nmos_707(gnd, net_1264, net_684);
  tranif1 nmos_708(gnd, net_1150, net_738);
  tranif1 nmos_709(gnd, net_1160, net_738);
  tranif1 nmos_710(gnd, net_1181, net_738);
  tranif1 nmos_711(gnd, net_1218, net_738);
  tranif1 nmos_712(gnd, net_1241, net_738);
  tranif1 nmos_713(gnd, net_1284, net_738);
  tranif1 nmos_714(gnd, net_1150, net_751);
  tranif1 nmos_715(gnd, net_1160, net_751);
  tranif1 nmos_716(gnd, net_1181, net_751);
  tranif1 nmos_717(gnd, net_1218, net_751);
  tranif1 nmos_718(gnd, net_1241, net_751);
  tranif1 nmos_719(gnd, net_1277, net_751);
  tranif1 nmos_720(gnd, net_1229, net_780);
  tranif1 nmos_721(gnd, net_1251, net_780);
  tranif1 nmos_722(gnd, net_1264, net_780);
  tranif1 nmos_723(gnd, net_1150, net_791);
  tranif1 nmos_724(gnd, net_1163, net_791);
  tranif1 nmos_725(gnd, net_1181, net_791);
  tranif1 nmos_726(gnd, net_1218, net_791);
  tranif1 nmos_727(gnd, net_1251, net_791);
  tranif1 nmos_728(gnd, net_1264, net_791);
  tranif1 nmos_729(gnd, net_1284, net_791);
  tranif1 nmos_730(gnd, net_1150, net_819);
  tranif1 nmos_731(gnd, net_1163, net_819);
  tranif1 nmos_732(gnd, net_1181, net_819);
  tranif1 nmos_733(gnd, net_1218, net_819);
  tranif1 nmos_734(gnd, net_1251, net_819);
  tranif1 nmos_735(gnd, net_1264, net_819);
  tranif1 nmos_736(gnd, net_1277, net_819);
  tranif1 nmos_737(gnd, net_1229, net_832);
  tranif1 nmos_738(gnd, net_1251, net_832);
  tranif1 nmos_739(gnd, net_1150, net_858);
  tranif1 nmos_740(gnd, net_1166, net_858);
  tranif1 nmos_741(gnd, net_1181, net_858);
  tranif1 nmos_742(gnd, net_1218, net_858);
  tranif1 nmos_743(gnd, net_1251, net_858);
  tranif1 nmos_744(gnd, net_1284, net_858);
  tranif1 nmos_745(gnd, net_1150, net_871);
  tranif1 nmos_746(gnd, net_1166, net_871);
  tranif1 nmos_747(gnd, net_1181, net_871);
  tranif1 nmos_748(gnd, net_1218, net_871);
  tranif1 nmos_749(gnd, net_1251, net_871);
  tranif1 nmos_750(gnd, net_1277, net_871);
  tranif1 nmos_751(gnd, net_1229, net_900);
  tranif1 nmos_752(gnd, net_1264, net_900);
  tranif1 nmos_753(gnd, net_1150, net_911);
  tranif1 nmos_754(gnd, net_1169, net_911);
  tranif1 nmos_755(gnd, net_1181, net_911);
  tranif1 nmos_756(gnd, net_1218, net_911);
  tranif1 nmos_757(gnd, net_1264, net_911);
  tranif1 nmos_758(gnd, net_1284, net_911);
  tranif1 nmos_759(gnd, net_1150, net_939);
  tranif1 nmos_760(gnd, net_1169, net_939);
  tranif1 nmos_761(gnd, net_1181, net_939);
  tranif1 nmos_762(gnd, net_1218, net_939);
  tranif1 nmos_763(gnd, net_1264, net_939);
  tranif1 nmos_764(gnd, net_1277, net_939);
  tranif1 nmos_765(gnd, net_1229, net_952);
  tranif1 nmos_878(gnd, out_26[26], net_1465);
  tranif1 nmos_886(gnd, out_25[25], net_1473);
  tranif1 nmos_894(gnd, out_24[24], net_1481);
  tranif1 nmos_902(gnd, out_23[23], net_1150);
  tranif1 nmos_910(gnd, out_22[22], net_1160);
  tranif1 nmos_918(gnd, out_21[21], net_1163);
  tranif1 nmos_926(gnd, out_20[20], net_1166);
  tranif1 nmos_934(gnd, out_19[19], net_1169);
  tranif1 nmos_942(gnd, out_18[18], net_1172);
  tranif1 nmos_950(gnd, out_17[17], net_1181);
  tranif1 nmos_958(gnd, out_16[16], net_1545);
  tranif1 nmos_966(gnd, out_15[15], net_1553);
  tranif1 nmos_974(gnd, out_14[14], net_1561);
  tranif1 nmos_982(gnd, out_13[13], net_1201);
  tranif1 nmos_990(gnd, out_12[12], net_1204);
  tranif1 nmos_998(gnd, out_11[11], net_1585);
  tranif1 nmos_1006(gnd, out_10[10], net_1210);
  tranif1 nmos_1014(gnd, out_9[9], net_1601);
  tranif1 nmos_1022(gnd, out_8[8], net_1218);
  tranif1 nmos_1030(gnd, out_7[7], net_1229);
  tranif1 nmos_1038(gnd, out_6[6], net_1235);
  tranif1 nmos_1046(gnd, out_5[5], net_1241);
  tranif1 nmos_1054(gnd, out_4[4], net_1251);
  tranif1 nmos_1062(gnd, out_3[3], net_1264);
  tranif1 nmos_1070(gnd, out_2[2], net_1277);
  tranif1 nmos_1078(gnd, out_1[1], net_1284);
  tranif1 nmos_1086(gnd, out[0], net_1673);
  tranif0 pmos_4(net_0, vdd, in_11[11]);
  tranif0 pmos_11(net_1980, vdd, in_10[10]);
  tranif0 pmos_18(net_14, vdd, in_9[9]);
  tranif0 pmos_25(net_21, vdd, in_8[8]);
  tranif0 pmos_32(net_28, vdd, in_7[7]);
  tranif0 pmos_39(net_1981, vdd, in_6[6]);
  tranif0 pmos_46(net_42, vdd, in_5[5]);
  tranif0 pmos_53(net_49, vdd, in_4[4]);
  tranif0 pmos_60(net_56, vdd, in_3[3]);
  tranif0 pmos_67(net_63, vdd, in_2[2]);
  tranif0 pmos_74(net_70, vdd, in_1[1]);
  tranif0 pmos_81(net_77, vdd, in[0]);
  tranif0 pmos_879(out_26[26], vdd, net_1465);
  tranif0 pmos_887(out_25[25], vdd, net_1473);
  tranif0 pmos_895(out_24[24], vdd, net_1481);
  tranif0 pmos_903(out_23[23], vdd, net_1150);
  tranif0 pmos_911(out_22[22], vdd, net_1160);
  tranif0 pmos_919(out_21[21], vdd, net_1163);
  tranif0 pmos_927(out_20[20], vdd, net_1166);
  tranif0 pmos_935(out_19[19], vdd, net_1169);
  tranif0 pmos_943(out_18[18], vdd, net_1172);
  tranif0 pmos_951(out_17[17], vdd, net_1181);
  tranif0 pmos_959(out_16[16], vdd, net_1545);
  tranif0 pmos_967(out_15[15], vdd, net_1553);
  tranif0 pmos_975(out_14[14], vdd, net_1561);
  tranif0 pmos_983(out_13[13], vdd, net_1201);
  tranif0 pmos_991(out_12[12], vdd, net_1204);
  tranif0 pmos_999(out_11[11], vdd, net_1585);
  tranif0 pmos_1007(out_10[10], vdd, net_1210);
  tranif0 pmos_1015(out_9[9], vdd, net_1601);
  tranif0 pmos_1023(out_8[8], vdd, net_1218);
  tranif0 pmos_1031(out_7[7], vdd, net_1229);
  tranif0 pmos_1039(out_6[6], vdd, net_1235);
  tranif0 pmos_1047(out_5[5], vdd, net_1241);
  tranif0 pmos_1055(out_4[4], vdd, net_1251);
  tranif0 pmos_1063(out_3[3], vdd, net_1264);
  tranif0 pmos_1071(out_2[2], vdd, net_1277);
  tranif0 pmos_1079(out_1[1], vdd, net_1284);
  tranif0 pmos_1087(out[0], vdd, net_1673);
  rtranif0 pmos_1092(net_1465, vdd, gnd);
  rtranif0 pmos_1094(net_1473, vdd, gnd);
  rtranif0 pmos_1096(net_1481, vdd, gnd);
  rtranif0 pmos_1098(net_1150, vdd, gnd);
  rtranif0 pmos_1100(net_1160, vdd, gnd);
  rtranif0 pmos_1102(net_1163, vdd, gnd);
  rtranif0 pmos_1104(net_1166, vdd, gnd);
  rtranif0 pmos_1106(net_1169, vdd, gnd);
  rtranif0 pmos_1108(net_1172, vdd, gnd);
  rtranif0 pmos_1110(net_1181, vdd, gnd);
  rtranif0 pmos_1112(net_1545, vdd, gnd);
  rtranif0 pmos_1114(net_1553, vdd, gnd);
  rtranif0 pmos_1116(net_1561, vdd, gnd);
  rtranif0 pmos_1118(net_1201, vdd, gnd);
  rtranif0 pmos_1120(net_1204, vdd, gnd);
  rtranif0 pmos_1122(net_1585, vdd, gnd);
  rtranif0 pmos_1124(net_1210, vdd, gnd);
  rtranif0 pmos_1126(net_1601, vdd, gnd);
  rtranif0 pmos_1128(net_1218, vdd, gnd);
  rtranif0 pmos_1130(net_1229, vdd, gnd);
  rtranif0 pmos_1132(net_1235, vdd, gnd);
  rtranif0 pmos_1134(net_1241, vdd, gnd);
  rtranif0 pmos_1136(net_1251, vdd, gnd);
  rtranif0 pmos_1138(net_1264, vdd, gnd);
  rtranif0 pmos_1140(net_1277, vdd, gnd);
  rtranif0 pmos_1142(net_1284, vdd, gnd);
  rtranif0 pmos_1144(net_1673, vdd, gnd);
  rtranif0 pmos_1148(vdd, net_952, gnd);
  rtranif0 pmos_1150(vdd, net_939, gnd);
  rtranif0 pmos_1152(vdd, net_911, gnd);
  rtranif0 pmos_1154(vdd, net_900, gnd);
  rtranif0 pmos_1156(vdd, net_871, gnd);
  rtranif0 pmos_1158(vdd, net_858, gnd);
  rtranif0 pmos_1160(vdd, net_832, gnd);
  rtranif0 pmos_1162(vdd, net_819, gnd);
  rtranif0 pmos_1164(vdd, net_791, gnd);
  rtranif0 pmos_1166(vdd, net_780, gnd);
  rtranif0 pmos_1168(vdd, net_751, gnd);
  rtranif0 pmos_1170(vdd, net_738, gnd);
  rtranif0 pmos_1172(vdd, net_684, gnd);
  rtranif0 pmos_1174(vdd, net_661, gnd);
  rtranif0 pmos_1176(vdd, net_607, gnd);
  rtranif0 pmos_1178(vdd, net_584, gnd);
  rtranif0 pmos_1180(vdd, net_530, gnd);
  rtranif0 pmos_1182(vdd, net_507, gnd);
  rtranif0 pmos_1184(vdd, net_459, gnd);
  rtranif0 pmos_1186(vdd, net_448, gnd);
  rtranif0 pmos_1188(vdd, net_419, gnd);
  rtranif0 pmos_1190(vdd, net_406, gnd);
  rtranif0 pmos_1192(vdd, net_382, gnd);
  rtranif0 pmos_1194(vdd, net_371, gnd);
  rtranif0 pmos_1196(vdd, net_347, gnd);
  rtranif0 pmos_1198(vdd, net_336, gnd);
  rtranif0 pmos_1200(vdd, net_312, gnd);
  rtranif0 pmos_1202(vdd, net_301, gnd);
endmodule   /* mips8__controller_pla_Cell */

module muddlib07__flopr_c_1x(ph1, ph2, d, resetb, q);
  input ph1;
  input ph2;
  input d;
  input resetb;
  output q;

  supply1 vdd;
  supply0 gnd;
  wire master, masterinb, n6, n7, n8, n9, net_429, ph1b, ph1buf, ph2b, ph2buf;
  wire slaveb;
  trireg masterb, slave;

  tranif1 nmos_2(masterinb, masterb, ph2buf);
  tranif1 nmos_3(gnd, master, masterb);
  rtranif1 nmos_4(master, slave, ph1buf);
  tranif1 nmos_5(n6, masterb, ph2b);
  tranif1 nmos_6(gnd, n6, master);
  tranif1 nmos_7(gnd, n8, slaveb);
  tranif1 nmos_8(gnd, slaveb, slave);
  tranif1 nmos_10(n8, slave, ph1b);
  tranif1 nmos_11(gnd, q, slaveb);
  tranif1 nmos_17(gnd, net_429, resetb);
  tranif1 nmos_19(net_429, masterinb, d);
  tranif1 nmos_22(gnd, ph2b, ph2);
  tranif1 nmos_25(gnd, ph2buf, ph2b);
  tranif1 nmos_26(gnd, ph1buf, ph1b);
  tranif1 nmos_27(gnd, ph1b, ph1);
  tranif0 pmos_2(masterb, masterinb, ph2b);
  tranif0 pmos_3(master, vdd, masterb);
  rtranif0 pmos_4(slave, master, ph1b);
  tranif0 pmos_5(masterb, n7, ph2buf);
  tranif0 pmos_6(n7, vdd, master);
  tranif0 pmos_7(n9, vdd, slaveb);
  tranif0 pmos_8(slaveb, vdd, slave);
  tranif0 pmos_10(slave, n9, ph1buf);
  tranif0 pmos_11(q, vdd, slaveb);
  tranif0 pmos_16(masterinb, vdd, d);
  tranif0 pmos_18(masterinb, vdd, resetb);
  tranif0 pmos_21(ph1b, vdd, ph1);
  tranif0 pmos_22(ph2b, vdd, ph2);
  tranif0 pmos_24(ph1buf, vdd, ph1b);
  tranif0 pmos_25(ph2buf, vdd, ph2b);
endmodule   /* muddlib07__flopr_c_1x */

module controller(funct, hit, op, ph1, ph2, reset, zero, alucontrol, alusrca, 
      alusrcb, cachewrite, iord, irwrite, memread, memtoreg, memwrite, morc, 
      morw, pcen, pcsrc, regdst, regwrite);
  input [3:0] funct;
  input hit;
  input [5:0] op;
  input ph1;
  input ph2;
  input reset;
  input zero;
  output [2:0] alucontrol;
  output alusrca;
  output [1:0] alusrcb;
  output cachewrite;
  output iord;
  output [3:0] irwrite;
  output memread;
  output memtoreg;
  output memwrite;
  output morc;
  output morw;
  output pcen;
  output [1:0] pcsrc;
  output regdst;
  output regwrite;

  supply1 vdd;
  supply0 gnd;
  wire branch, net_86, pcwrite;
  wire [1:0] ALUOp;
  wire [4:0] nextstate;
  wire [4:0] state;

  muddlib07__a2o1_1x a2o1_1x_0(.a(zero), .b(branch), .c(pcwrite), .y(pcen));
  mips8__aludec aludec_1(.ALUOp(ALUOp[1:0]), .Funct(funct[3:0]), 
      .alucontrol(alucontrol[2:0]));
  mips8__controller_pla_Cell controll_4(.in({hit}), .in_1(state[0:0]), 
      .in_10(op[4:4]), .in_11(op[5:5]), .in_2(state[1:1]), .in_3(state[2:2]), 
      .in_4(state[3:3]), .in_5(state[4:4]), .in_6(op[0:0]), .in_7(op[1:1]), 
      .in_8(op[2:2]), .in_9(op[3:3]), .out({morw}), .out_1({morc}), 
      .out_10({alusrca}), .out_11({memtoreg}), .out_12({iord}), 
      .out_13({regwrite}), .out_14({regdst}), .out_15(pcsrc[0:0]), 
      .out_16(pcsrc[1:1]), .out_17(alusrcb[0:0]), .out_18(alusrcb[1:1]), 
      .out_19(irwrite[0:0]), .out_2({cachewrite}), .out_20(irwrite[1:1]), 
      .out_21(irwrite[2:2]), .out_22(irwrite[3:3]), .out_23({pcwrite}), 
      .out_24({branch}), .out_25(ALUOp[0:0]), .out_26(ALUOp[1:1]), 
      .out_3(nextstate[0:0]), .out_4(nextstate[1:1]), .out_5(nextstate[2:2]), 
      .out_6(nextstate[3:3]), .out_7(nextstate[4:4]), .out_8({memread}), 
      .out_9({memwrite}));
  muddlib07__inv_1x inv_1x_0(.a(reset), .y(net_86));
  muddlib07__flopr_c_1x stateflop_4_(.ph1(ph1), .ph2(ph2), .d(nextstate[4]), 
      .resetb(net_86), .q(state[4]));
  muddlib07__flopr_c_1x stateflop_3_(.ph1(ph1), .ph2(ph2), .d(nextstate[3]), 
      .resetb(net_86), .q(state[3]));
  muddlib07__flopr_c_1x stateflop_2_(.ph1(ph1), .ph2(ph2), .d(nextstate[2]), 
      .resetb(net_86), .q(state[2]));
  muddlib07__flopr_c_1x stateflop_1_(.ph1(ph1), .ph2(ph2), .d(nextstate[1]), 
      .resetb(net_86), .q(state[1]));
  muddlib07__flopr_c_1x stateflop_0_(.ph1(ph1), .ph2(ph2), .d(nextstate[0]), 
      .resetb(net_86), .q(state[0]));
endmodule   /* controller */


/* Verilog for cell 'datapath{lay}' from library 'mips8' */
/* Created on Thu Jul 12, 2007 12:24:38 */
/* Last revised on Tue Sep 16, 2014 16:53:10 */
/* Written on Tue Sep 16, 2014 16:54:39 by Electric VLSI Design System, version 8.06 */

module muddlib07__invbuf_4x(s, s_out, sb_out, vdd, gnd);
  input s;
  output s_out;
  output sb_out;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire plnode_0_well, plnode_1_well;

  tranif1 nmos_0(gnd, sb_out, s);
  tranif1 nmos_1(gnd, s_out, sb_out);
  tranif0 pmos_0(vdd, sb_out, s);
  tranif0 pmos_1(vdd, s_out, sb_out);
endmodule   /* muddlib07__invbuf_4x */

module muddlib07__mux2_dp_1x(d0, d1, s, sb, y, vdd, gnd);
  input d0;
  input d1;
  input s;
  input sb;
  output y;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire net_186, net_187, net_188, net_189, net_80, plnode_2_well, plnode_3_well;

  tranif1 nmos_1(gnd, net_189, d1);
  tranif1 nmos_2(net_189, net_80, s);
  tranif1 nmos_3(net_80, net_188, sb);
  tranif1 nmos_4(net_188, gnd, d0);
  tranif1 nmos_5(gnd, y, net_80);
  tranif0 pmos_0(vdd, net_187, d1);
  tranif0 pmos_1(net_187, net_80, sb);
  tranif0 pmos_2(net_80, net_186, s);
  tranif0 pmos_3(net_186, vdd, d0);
  tranif0 pmos_4(vdd, y, net_80);
endmodule   /* muddlib07__mux2_dp_1x */

module wordlib8__mux2_1x_8(d0, d0_1, d0_2, d0_3, d0_4, d0_5, d0_6, d0_7, d1, 
      d1_1, d1_2, d1_3, d1_4, d1_5, d1_6, d1_7, s, y, y_1, y_2, y_3, y_4, y_5, 
      y_6, y_7, vdd, vdd_1, vdd_32, vdd_33, vdd_34, vdd_35, vdd_36, vdd_37, 
      vdd_38, gnd, gnd_1, gnd_32, gnd_33, gnd_34, gnd_35, gnd_36, gnd_37, 
      gnd_38);
  input [0:0] d0;
  input [1:1] d0_1;
  input [2:2] d0_2;
  input [3:3] d0_3;
  input [4:4] d0_4;
  input [5:5] d0_5;
  input [6:6] d0_6;
  input [7:7] d0_7;
  input [0:0] d1;
  input [1:1] d1_1;
  input [2:2] d1_2;
  input [3:3] d1_3;
  input [4:4] d1_4;
  input [5:5] d1_5;
  input [6:6] d1_6;
  input [7:7] d1_7;
  input s;
  output [0:0] y;
  output [1:1] y_1;
  output [2:2] y_2;
  output [3:3] y_3;
  output [4:4] y_4;
  output [5:5] y_5;
  output [6:6] y_6;
  output [7:7] y_7;
  input vdd;
  input vdd_1;
  input vdd_32;
  input vdd_33;
  input vdd_34;
  input vdd_35;
  input vdd_36;
  input vdd_37;
  input vdd_38;
  input gnd;
  input gnd_1;
  input gnd_32;
  input gnd_33;
  input gnd_34;
  input gnd_35;
  input gnd_36;
  input gnd_37;
  input gnd_38;

  supply1 vdd;
  supply0 gnd;
  wire net_67, net_74;

  muddlib07__invbuf_4x invbuf_4_0(.s(s), .s_out(net_74), .sb_out(net_67), 
      .vdd(vdd_32), .gnd(gnd_32));
  muddlib07__mux2_dp_1x mux2_dp__0(.d0(d0[0]), .d1(d1[0]), .s(net_74), 
      .sb(net_67), .y(y[0]), .vdd(vdd_1), .gnd(gnd_1));
  muddlib07__mux2_dp_1x mux2_dp__32(.d0(d0_1[1]), .d1(d1_1[1]), .s(net_74), 
      .sb(net_67), .y(y_1[1]), .vdd(vdd_33), .gnd(gnd_33));
  muddlib07__mux2_dp_1x mux2_dp__33(.d0(d0_2[2]), .d1(d1_2[2]), .s(net_74), 
      .sb(net_67), .y(y_2[2]), .vdd(vdd_34), .gnd(gnd_34));
  muddlib07__mux2_dp_1x mux2_dp__34(.d0(d0_3[3]), .d1(d1_3[3]), .s(net_74), 
      .sb(net_67), .y(y_3[3]), .vdd(vdd_35), .gnd(gnd_35));
  muddlib07__mux2_dp_1x mux2_dp__35(.d0(d0_4[4]), .d1(d1_4[4]), .s(net_74), 
      .sb(net_67), .y(y_4[4]), .vdd(vdd_36), .gnd(gnd_36));
  muddlib07__mux2_dp_1x mux2_dp__36(.d0(d0_5[5]), .d1(d1_5[5]), .s(net_74), 
      .sb(net_67), .y(y_5[5]), .vdd(vdd_37), .gnd(gnd_37));
  muddlib07__mux2_dp_1x mux2_dp__37(.d0(d0_6[6]), .d1(d1_6[6]), .s(net_74), 
      .sb(net_67), .y(y_6[6]), .vdd(vdd_38), .gnd(gnd_38));
  muddlib07__mux2_dp_1x mux2_dp__38(.d0(d0_7[7]), .d1(d1_7[7]), .s(net_74), 
      .sb(net_67), .y(y_7[7]), .vdd(vdd), .gnd(gnd));
endmodule   /* wordlib8__mux2_1x_8 */

module muddlib07__fulladder(a, b, c, cout, s, vdd, gnd);
  input a;
  input b;
  input c;
  output cout;
  output s;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire coutb, net_10, net_11, net_122, net_123, net_124, net_13, net_134;
  wire net_135, net_27, net_30, plnode_0_well, plnode_1_well, plnode_2_well;
  wire plnode_3_well, sumb;

  tranif1 nmos_0(gnd, cout, coutb);
  tranif1 nmos_1(gnd, s, sumb);
  tranif1 nmos_2(net_124, gnd, a);
  tranif1 nmos_3(net_122, net_124, b);
  tranif1 nmos_4(sumb, net_122, c);
  tranif1 nmos_5(net_123, sumb, coutb);
  tranif1 nmos_6(gnd, net_123, c);
  tranif1 nmos_7(net_123, gnd, b);
  tranif1 nmos_8(gnd, net_123, a);
  tranif1 nmos_9(net_134, gnd, a);
  tranif1 nmos_10(coutb, net_134, b);
  tranif1 nmos_11(net_135, coutb, c);
  tranif1 nmos_12(gnd, net_135, b);
  tranif1 nmos_13(net_135, gnd, a);
  tranif0 pmos_0(vdd, cout, coutb);
  tranif0 pmos_1(vdd, s, sumb);
  tranif0 pmos_2(net_10, vdd, a);
  tranif0 pmos_3(net_11, net_10, b);
  tranif0 pmos_4(sumb, net_11, c);
  tranif0 pmos_5(net_27, sumb, coutb);
  tranif0 pmos_6(vdd, net_27, c);
  tranif0 pmos_7(net_27, vdd, b);
  tranif0 pmos_8(vdd, net_27, a);
  tranif0 pmos_9(net_13, vdd, a);
  tranif0 pmos_10(coutb, net_13, b);
  tranif0 pmos_11(net_30, coutb, c);
  tranif0 pmos_12(vdd, net_30, b);
  tranif0 pmos_13(net_30, vdd, a);
endmodule   /* muddlib07__fulladder */

module wordlib8__adder_8(a, a_1, a_2, a_3, a_4, a_5, a_6, a_7, b, b_1, b_2, 
      b_3, b_4, b_5, b_6, b_7, cin, cout, s, s_1, s_2, s_3, s_4, s_5, s_6, s_7, 
      vdd, vdd_1, vdd_1_1, vdd_2, vdd_3, vdd_4, vdd_5, vdd_6, gnd, gnd_1, 
      gnd_1_1, gnd_2, gnd_3, gnd_4, gnd_5, gnd_6);
  input [0:0] a;
  input [1:1] a_1;
  input [2:2] a_2;
  input [3:3] a_3;
  input [4:4] a_4;
  input [5:5] a_5;
  input [6:6] a_6;
  input [7:7] a_7;
  input [0:0] b;
  input [1:1] b_1;
  input [2:2] b_2;
  input [3:3] b_3;
  input [4:4] b_4;
  input [5:5] b_5;
  input [6:6] b_6;
  input [7:7] b_7;
  input cin;
  output cout;
  output [0:0] s;
  output [1:1] s_1;
  output [2:2] s_2;
  output [3:3] s_3;
  output [4:4] s_4;
  output [5:5] s_5;
  output [6:6] s_6;
  output [7:7] s_7;
  input vdd;
  input vdd_1;
  input vdd_1_1;
  input vdd_2;
  input vdd_3;
  input vdd_4;
  input vdd_5;
  input vdd_6;
  input gnd;
  input gnd_1;
  input gnd_1_1;
  input gnd_2;
  input gnd_3;
  input gnd_4;
  input gnd_5;
  input gnd_6;

  supply1 vdd;
  supply0 gnd;
  wire net_0, net_1, net_2, net_3, net_30, net_4, net_5;

  muddlib07__fulladder fulladde_0(.a(a[0]), .b(b[0]), .c(cin), .cout(net_0), 
      .s(s[0]), .vdd(vdd_1), .gnd(gnd_1));
  muddlib07__fulladder fulladde_1(.a(a_1[1]), .b(b_1[1]), .c(net_0), 
      .cout(net_30), .s(s_1[1]), .vdd(vdd_1_1), .gnd(gnd_1_1));
  muddlib07__fulladder fulladde_2(.a(a_2[2]), .b(b_2[2]), .c(net_30), 
      .cout(net_1), .s(s_2[2]), .vdd(vdd_2), .gnd(gnd_2));
  muddlib07__fulladder fulladde_3(.a(a_3[3]), .b(b_3[3]), .c(net_1), 
      .cout(net_2), .s(s_3[3]), .vdd(vdd_3), .gnd(gnd_3));
  muddlib07__fulladder fulladde_4(.a(a_4[4]), .b(b_4[4]), .c(net_2), 
      .cout(net_3), .s(s_4[4]), .vdd(vdd_4), .gnd(gnd_4));
  muddlib07__fulladder fulladde_5(.a(a_5[5]), .b(b_5[5]), .c(net_3), 
      .cout(net_4), .s(s_5[5]), .vdd(vdd_5), .gnd(gnd_5));
  muddlib07__fulladder fulladde_6(.a(a_6[6]), .b(b_6[6]), .c(net_4), 
      .cout(net_5), .s(s_6[6]), .vdd(vdd_6), .gnd(gnd_6));
  muddlib07__fulladder fulladde_7(.a(a_7[7]), .b(b_7[7]), .c(net_5), 
      .cout(cout), .s(s_7[7]), .vdd(vdd), .gnd(gnd));
endmodule   /* wordlib8__adder_8 */

module muddlib07__and2_1x(a, b, y, vdd, gnd);
  input a;
  input b;
  output y;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire net_72, net_73, plno_2_well, plnode_0_well;

  tranif1 nmos_0(gnd, net_72, a);
  tranif1 nmos_1(net_72, net_73, b);
  tranif1 nmos_2(y, gnd, net_73);
  tranif0 pmos_0(vdd, net_73, a);
  tranif0 pmos_1(net_73, vdd, b);
  tranif0 pmos_2(y, vdd, net_73);
endmodule   /* muddlib07__and2_1x */

module wordlib8__and2_1x_8(a, a_1, a_2, a_3, a_4, a_5, a_6, a_7, b, b_1, b_2, 
      b_3, b_4, b_5, b_6, b_7, y, y_1, y_2, y_3, y_4, y_5, y_6, y_7, vdd, 
      vdd_1, vdd_1_1, vdd_2, vdd_3, vdd_4, vdd_5, vdd_6, gnd, gnd_1, gnd_1_1, 
      gnd_2, gnd_3, gnd_4, gnd_5, gnd_6);
  input [0:0] a;
  input [1:1] a_1;
  input [2:2] a_2;
  input [3:3] a_3;
  input [4:4] a_4;
  input [5:5] a_5;
  input [6:6] a_6;
  input [7:7] a_7;
  input [0:0] b;
  input [1:1] b_1;
  input [2:2] b_2;
  input [3:3] b_3;
  input [4:4] b_4;
  input [5:5] b_5;
  input [6:6] b_6;
  input [7:7] b_7;
  output [0:0] y;
  output [1:1] y_1;
  output [2:2] y_2;
  output [3:3] y_3;
  output [4:4] y_4;
  output [5:5] y_5;
  output [6:6] y_6;
  output [7:7] y_7;
  input vdd;
  input vdd_1;
  input vdd_1_1;
  input vdd_2;
  input vdd_3;
  input vdd_4;
  input vdd_5;
  input vdd_6;
  input gnd;
  input gnd_1;
  input gnd_1_1;
  input gnd_2;
  input gnd_3;
  input gnd_4;
  input gnd_5;
  input gnd_6;

  supply1 vdd;
  supply0 gnd;
  muddlib07__and2_1x and2_1x_0_(.a(a[0]), .b(b[0]), .y(y[0]), .vdd(vdd), 
      .gnd(gnd_1));
  muddlib07__and2_1x and2_1x_1_(.a(a_1[1]), .b(b_1[1]), .y(y_1[1]), 
      .vdd(vdd_1_1), .gnd(gnd_1_1));
  muddlib07__and2_1x and2_1x_2_(.a(a_2[2]), .b(b_2[2]), .y(y_2[2]), 
      .vdd(vdd_2), .gnd(gnd_2));
  muddlib07__and2_1x and2_1x_3_(.a(a_3[3]), .b(b_3[3]), .y(y_3[3]), 
      .vdd(vdd_3), .gnd(gnd_3));
  muddlib07__and2_1x and2_1x_4_(.a(a_4[4]), .b(b_4[4]), .y(y_4[4]), 
      .vdd(vdd_4), .gnd(gnd_4));
  muddlib07__and2_1x and2_1x_5_(.a(a_5[5]), .b(b_5[5]), .y(y_5[5]), 
      .vdd(vdd_5), .gnd(gnd_5));
  muddlib07__and2_1x and2_1x_6_(.a(a_6[6]), .b(b_6[6]), .y(y_6[6]), 
      .vdd(vdd_6), .gnd(gnd_6));
  muddlib07__and2_1x and2_1x_7_(.a(a_7[7]), .b(b_7[7]), .y(y_7[7]), 
      .vdd(vdd_1), .gnd(gnd));
endmodule   /* wordlib8__and2_1x_8 */

module muddlib07__inv_1x(a, y, vdd, gnd);
  input a;
  output y;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire plnode_0_well, plnode_1_well;

  tranif1 nmos_0(gnd, y, a);
  tranif0 pmos_0(vdd, y, a);
endmodule   /* muddlib07__inv_1x */

module wordlib8__inv_1x_8(a, a_1, a_2, a_3, a_4, a_5, a_6, a_7, y, y_1, y_2, 
      y_3, y_4, y_5, y_6, y_7, vdd, vdd_1, vdd_1_1, vdd_2, vdd_3, vdd_4, vdd_5, 
      vdd_6, gnd, gnd_1, gnd_1_1, gnd_2, gnd_3, gnd_4, gnd_5, gnd_6);
  input [0:0] a;
  input [1:1] a_1;
  input [2:2] a_2;
  input [3:3] a_3;
  input [4:4] a_4;
  input [5:5] a_5;
  input [6:6] a_6;
  input [7:7] a_7;
  output [0:0] y;
  output [1:1] y_1;
  output [2:2] y_2;
  output [3:3] y_3;
  output [4:4] y_4;
  output [5:5] y_5;
  output [6:6] y_6;
  output [7:7] y_7;
  input vdd;
  input vdd_1;
  input vdd_1_1;
  input vdd_2;
  input vdd_3;
  input vdd_4;
  input vdd_5;
  input vdd_6;
  input gnd;
  input gnd_1;
  input gnd_1_1;
  input gnd_2;
  input gnd_3;
  input gnd_4;
  input gnd_5;
  input gnd_6;

  supply1 vdd;
  supply0 gnd;
  muddlib07__inv_1x inv_1x_0(.a(a[0]), .y(y[0]), .vdd(vdd), .gnd(gnd_1));
  muddlib07__inv_1x inv_1x_1(.a(a_1[1]), .y(y_1[1]), .vdd(vdd_1_1), 
      .gnd(gnd_1_1));
  muddlib07__inv_1x inv_1x_2(.a(a_2[2]), .y(y_2[2]), .vdd(vdd_2), 
      .gnd(gnd_2));
  muddlib07__inv_1x inv_1x_3(.a(a_3[3]), .y(y_3[3]), .vdd(vdd_3), 
      .gnd(gnd_3));
  muddlib07__inv_1x inv_1x_4(.a(a_4[4]), .y(y_4[4]), .vdd(vdd_4), 
      .gnd(gnd_4));
  muddlib07__inv_1x inv_1x_5(.a(a_5[5]), .y(y_5[5]), .vdd(vdd_5), 
      .gnd(gnd_5));
  muddlib07__inv_1x inv_1x_6(.a(a_6[6]), .y(y_6[6]), .vdd(vdd_6), 
      .gnd(gnd_6));
  muddlib07__inv_1x inv_1x_7(.a(a_7[7]), .y(y_7[7]), .vdd(vdd_1), .gnd(gnd));
endmodule   /* wordlib8__inv_1x_8 */

module mips8__condinv(a, a_1, a_2, a_3, a_4, a_5, a_6, a_7, invert, y, y_1, 
      y_2, y_3, y_4, y_5, y_6, y_7, vdd, vdd_1, vdd_32, vdd_33, vdd_34, vdd_35, 
      vdd_36, vdd_37, vdd_38, gnd, gnd_1, gnd_32, gnd_33, gnd_34, gnd_35, 
      gnd_36, gnd_37, gnd_38);
  input [0:0] a;
  input [1:1] a_1;
  input [2:2] a_2;
  input [3:3] a_3;
  input [4:4] a_4;
  input [5:5] a_5;
  input [6:6] a_6;
  input [7:7] a_7;
  input invert;
  output [0:0] y;
  output [1:1] y_1;
  output [2:2] y_2;
  output [3:3] y_3;
  output [4:4] y_4;
  output [5:5] y_5;
  output [6:6] y_6;
  output [7:7] y_7;
  input vdd;
  input vdd_1;
  input vdd_32;
  input vdd_33;
  input vdd_34;
  input vdd_35;
  input vdd_36;
  input vdd_37;
  input vdd_38;
  input gnd;
  input gnd_1;
  input gnd_32;
  input gnd_33;
  input gnd_34;
  input gnd_35;
  input gnd_36;
  input gnd_37;
  input gnd_38;

  supply1 vdd;
  supply0 gnd;
  wire net_16, net_18, net_19, net_20, net_21, net_22, net_23, net_24;

  wordlib8__inv_1x_8 inv_1x_8_0(.a(a[0:0]), .a_1(a_1[1:1]), .a_2(a_2[2:2]), 
      .a_3(a_3[3:3]), .a_4(a_4[4:4]), .a_5(a_5[5:5]), .a_6(a_6[6:6]), 
      .a_7(a_7[7:7]), .y({net_16}), .y_1({net_18}), .y_2({net_19}), 
      .y_3({net_20}), .y_4({net_21}), .y_5({net_22}), .y_6({net_23}), 
      .y_7({net_24}), .vdd(vdd), .vdd_1(vdd_1), .vdd_1_1(vdd_33), 
      .vdd_2(vdd_34), .vdd_3(vdd_35), .vdd_4(vdd_36), .vdd_5(vdd_37), 
      .vdd_6(vdd_38), .gnd(gnd_33), .gnd_1(gnd_1), .gnd_1_1(gnd), 
      .gnd_2(gnd_38), .gnd_3(gnd_37), .gnd_4(gnd_36), .gnd_5(gnd_35), 
      .gnd_6(gnd_34));
  wordlib8__mux2_1x_8 mux2_1x__0(.d0(a[0:0]), .d0_1(a_1[1:1]), .d0_2(a_2[2:2]), 
      .d0_3(a_3[3:3]), .d0_4(a_4[4:4]), .d0_5(a_5[5:5]), .d0_6(a_6[6:6]), 
      .d0_7(a_7[7:7]), .d1({net_16}), .d1_1({net_18}), .d1_2({net_19}), 
      .d1_3({net_20}), .d1_4({net_21}), .d1_5({net_22}), .d1_6({net_23}), 
      .d1_7({net_24}), .s(invert), .y(y[0:0]), .y_1(y_1[1:1]), .y_2(y_2[2:2]), 
      .y_3(y_3[3:3]), .y_4(y_4[4:4]), .y_5(y_5[5:5]), .y_6(y_6[6:6]), 
      .y_7(y_7[7:7]), .vdd(vdd_1), .vdd_1(vdd), .vdd_32(vdd_32), 
      .vdd_33(vdd_33), .vdd_34(vdd_34), .vdd_35(vdd_35), .vdd_36(vdd_36), 
      .vdd_37(vdd_37), .vdd_38(vdd_38), .gnd(gnd_33), .gnd_1(gnd_1), 
      .gnd_32(gnd_32), .gnd_33(gnd), .gnd_34(gnd_38), .gnd_35(gnd_37), 
      .gnd_36(gnd_36), .gnd_37(gnd_35), .gnd_38(gnd_34));
endmodule   /* mips8__condinv */

module muddlib07__mux4_dp_1x(d0, d1, d2, d3, s0, s0b, s1, s1b, y, vdd, gnd);
  input d0;
  input d1;
  input d2;
  input d3;
  input s0;
  input s0b;
  input s1;
  input s1b;
  output y;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire net_306, net_318, net_338, net_390, net_501, net_835, net_856, net_857;
  wire net_858, net_880, net_902, net_904, net_907, plno_2_well, plnode_0_well;

  tranif1 nmos_6(gnd, y, net_835);
  tranif1 nmos_7(net_338, net_318, s0b);
  tranif1 nmos_8(net_318, gnd, d0);
  tranif1 nmos_9(gnd, net_880, d1);
  tranif1 nmos_10(net_880, net_338, s0);
  tranif1 nmos_12(net_338, net_835, s1b);
  tranif1 nmos_13(net_835, net_902, s1);
  tranif1 nmos_18(net_902, net_904, s0b);
  tranif1 nmos_19(net_904, gnd, d2);
  tranif1 nmos_20(gnd, net_907, d3);
  tranif1 nmos_21(net_907, net_902, s0);
  tranif0 pmos_5(vdd, y, net_835);
  tranif0 pmos_7(net_306, net_390, s0);
  tranif0 pmos_9(net_390, vdd, d0);
  tranif0 pmos_18(vdd, net_501, d1);
  tranif0 pmos_20(net_501, net_306, s0b);
  tranif0 pmos_21(net_306, net_835, s1);
  tranif0 pmos_22(net_835, net_856, s1b);
  tranif0 pmos_23(vdd, net_858, d3);
  tranif0 pmos_24(net_858, net_856, s0b);
  tranif0 pmos_25(net_856, net_857, s0);
  tranif0 pmos_26(net_857, vdd, d2);
endmodule   /* muddlib07__mux4_dp_1x */

module wordlib8__mux4_1x_8(d0, d0_1, d0_2, d0_3, d0_4, d0_5, d0_6, d0_7, d1, 
      d1_1, d1_2, d1_3, d1_4, d1_5, d1_6, d1_7, d2, d2_1, d2_2, d2_3, d2_4, 
      d2_5, d2_6, d2_7, d3, d3_1, d3_2, d3_3, d3_4, d3_5, d3_6, d3_7, s0, s1, 
      y, y_1, y_2, y_3, y_4, y_5, y_6, y_7, vdd, vdd_1, vdd_2, vdd_34, vdd_35, 
      vdd_36, vdd_37, vdd_38, vdd_39, gnd, gnd_1, gnd_2, gnd_34, gnd_35, 
      gnd_36, gnd_37, gnd_38, gnd_39);
  input [0:0] d0;
  input [1:1] d0_1;
  input [2:2] d0_2;
  input [3:3] d0_3;
  input [4:4] d0_4;
  input [5:5] d0_5;
  input [6:6] d0_6;
  input [7:7] d0_7;
  input [0:0] d1;
  input [1:1] d1_1;
  input [2:2] d1_2;
  input [3:3] d1_3;
  input [4:4] d1_4;
  input [5:5] d1_5;
  input [6:6] d1_6;
  input [7:7] d1_7;
  input [0:0] d2;
  input [1:1] d2_1;
  input [2:2] d2_2;
  input [3:3] d2_3;
  input [4:4] d2_4;
  input [5:5] d2_5;
  input [6:6] d2_6;
  input [7:7] d2_7;
  input [0:0] d3;
  input [1:1] d3_1;
  input [2:2] d3_2;
  input [3:3] d3_3;
  input [4:4] d3_4;
  input [5:5] d3_5;
  input [6:6] d3_6;
  input [7:7] d3_7;
  input s0;
  input s1;
  output [0:0] y;
  output [1:1] y_1;
  output [2:2] y_2;
  output [3:3] y_3;
  output [4:4] y_4;
  output [5:5] y_5;
  output [6:6] y_6;
  output [7:7] y_7;
  input vdd;
  input vdd_1;
  input vdd_2;
  input vdd_34;
  input vdd_35;
  input vdd_36;
  input vdd_37;
  input vdd_38;
  input vdd_39;
  input gnd;
  input gnd_1;
  input gnd_2;
  input gnd_34;
  input gnd_35;
  input gnd_36;
  input gnd_37;
  input gnd_38;
  input gnd_39;

  supply1 vdd;
  supply0 gnd;
  wire net_14, net_174, net_181, net_8;

  muddlib07__invbuf_4x invbuf_4_0(.s(s0), .s_out(net_181), .sb_out(net_8), 
      .vdd(vdd_1), .gnd(gnd));
  muddlib07__invbuf_4x invbuf_4_1(.s(s1), .s_out(net_174), .sb_out(net_14), 
      .vdd(vdd_1), .gnd(gnd));
  muddlib07__mux4_dp_1x mux4_dp__0(.d0(d0[0]), .d1(d1[0]), .d2(d2[0]), 
      .d3(d3[0]), .s0(net_181), .s0b(net_8), .s1(net_174), .s1b(net_14), 
      .y(y[0]), .vdd(vdd_2), .gnd(gnd_2));
  muddlib07__mux4_dp_1x mux4_dp__32(.d0(d0_1[1]), .d1(d1_1[1]), .d2(d2_1[1]), 
      .d3(d3_1[1]), .s0(net_181), .s0b(net_8), .s1(net_174), .s1b(net_14), 
      .y(y_1[1]), .vdd(vdd_34), .gnd(gnd_34));
  muddlib07__mux4_dp_1x mux4_dp__33(.d0(d0_2[2]), .d1(d1_2[2]), .d2(d2_2[2]), 
      .d3(d3_2[2]), .s0(net_181), .s0b(net_8), .s1(net_174), .s1b(net_14), 
      .y(y_2[2]), .vdd(vdd_35), .gnd(gnd_35));
  muddlib07__mux4_dp_1x mux4_dp__34(.d0(d0_3[3]), .d1(d1_3[3]), .d2(d2_3[3]), 
      .d3(d3_3[3]), .s0(net_181), .s0b(net_8), .s1(net_174), .s1b(net_14), 
      .y(y_3[3]), .vdd(vdd_36), .gnd(gnd_36));
  muddlib07__mux4_dp_1x mux4_dp__35(.d0(d0_4[4]), .d1(d1_4[4]), .d2(d2_4[4]), 
      .d3(d3_4[4]), .s0(net_181), .s0b(net_8), .s1(net_174), .s1b(net_14), 
      .y(y_4[4]), .vdd(vdd_37), .gnd(gnd_37));
  muddlib07__mux4_dp_1x mux4_dp__36(.d0(d0_5[5]), .d1(d1_5[5]), .d2(d2_5[5]), 
      .d3(d3_5[5]), .s0(net_181), .s0b(net_8), .s1(net_174), .s1b(net_14), 
      .y(y_5[5]), .vdd(vdd_38), .gnd(gnd_38));
  muddlib07__mux4_dp_1x mux4_dp__37(.d0(d0_6[6]), .d1(d1_6[6]), .d2(d2_6[6]), 
      .d3(d3_6[6]), .s0(net_181), .s0b(net_8), .s1(net_174), .s1b(net_14), 
      .y(y_6[6]), .vdd(vdd_39), .gnd(gnd_39));
  muddlib07__mux4_dp_1x mux4_dp__38(.d0(d0_7[7]), .d1(d1_7[7]), .d2(d2_7[7]), 
      .d3(d3_7[7]), .s0(net_181), .s0b(net_8), .s1(net_174), .s1b(net_14), 
      .y(y_7[7]), .vdd(vdd), .gnd(gnd_1));
endmodule   /* wordlib8__mux4_1x_8 */

module muddlib07__or2_1x(a, b, y, vdd, gnd);
  input a;
  input b;
  output y;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire net_75, net_8, plnode_0_well, plnode_1_well;

  tranif1 nmos_0(gnd, net_8, a);
  tranif1 nmos_1(net_8, gnd, b);
  tranif1 nmos_4(y, gnd, net_8);
  tranif0 pmos_0(vdd, net_75, a);
  tranif0 pmos_1(net_75, net_8, b);
  tranif0 pmos_4(y, vdd, net_8);
endmodule   /* muddlib07__or2_1x */

module wordlib8__or2_1x_8(a, a_1, a_2, a_3, a_4, a_5, a_6, a_7, b, b_1, b_2, 
      b_3, b_4, b_5, b_6, b_7, y, y_1, y_2, y_3, y_4, y_5, y_6, y_7, vdd, 
      vdd_1, vdd_1_1, vdd_2, vdd_3, vdd_4, vdd_5, vdd_6, gnd, gnd_1, gnd_1_1, 
      gnd_2, gnd_3, gnd_4, gnd_5, gnd_6);
  input [0:0] a;
  input [1:1] a_1;
  input [2:2] a_2;
  input [3:3] a_3;
  input [4:4] a_4;
  input [5:5] a_5;
  input [6:6] a_6;
  input [7:7] a_7;
  input [0:0] b;
  input [1:1] b_1;
  input [2:2] b_2;
  input [3:3] b_3;
  input [4:4] b_4;
  input [5:5] b_5;
  input [6:6] b_6;
  input [7:7] b_7;
  output [0:0] y;
  output [1:1] y_1;
  output [2:2] y_2;
  output [3:3] y_3;
  output [4:4] y_4;
  output [5:5] y_5;
  output [6:6] y_6;
  output [7:7] y_7;
  input vdd;
  input vdd_1;
  input vdd_1_1;
  input vdd_2;
  input vdd_3;
  input vdd_4;
  input vdd_5;
  input vdd_6;
  input gnd;
  input gnd_1;
  input gnd_1_1;
  input gnd_2;
  input gnd_3;
  input gnd_4;
  input gnd_5;
  input gnd_6;

  supply1 vdd;
  supply0 gnd;
  muddlib07__or2_1x or2_1x_0_(.a(a[0]), .b(b[0]), .y(y[0]), .vdd(vdd), 
      .gnd(gnd));
  muddlib07__or2_1x or2_1x_1_(.a(a_1[1]), .b(b_1[1]), .y(y_1[1]), 
      .vdd(vdd_1_1), .gnd(gnd_1_1));
  muddlib07__or2_1x or2_1x_2_(.a(a_2[2]), .b(b_2[2]), .y(y_2[2]), .vdd(vdd_2), 
      .gnd(gnd_2));
  muddlib07__or2_1x or2_1x_3_(.a(a_3[3]), .b(b_3[3]), .y(y_3[3]), .vdd(vdd_3), 
      .gnd(gnd_3));
  muddlib07__or2_1x or2_1x_4_(.a(a_4[4]), .b(b_4[4]), .y(y_4[4]), .vdd(vdd_4), 
      .gnd(gnd_4));
  muddlib07__or2_1x or2_1x_5_(.a(a_5[5]), .b(b_5[5]), .y(y_5[5]), .vdd(vdd_5), 
      .gnd(gnd_5));
  muddlib07__or2_1x or2_1x_6_(.a(a_6[6]), .b(b_6[6]), .y(y_6[6]), .vdd(vdd_6), 
      .gnd(gnd_6));
  muddlib07__or2_1x or2_1x_7_(.a(a_7[7]), .b(b_7[7]), .y(y_7[7]), .vdd(vdd_1), 
      .gnd(gnd_1));
endmodule   /* wordlib8__or2_1x_8 */

module muddlib07__nand2_1x(a, b, y, vdd, gnd);
  input a;
  input b;
  output y;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire net_66, plno_2_well, plnode_0_well;

  tranif1 nmos_0(net_66, y, b);
  tranif1 nmos_1(gnd, net_66, a);
  tranif0 pmos_0(y, vdd, b);
  tranif0 pmos_1(vdd, y, a);
endmodule   /* muddlib07__nand2_1x */

module muddlib07__nor2_1x(a, b, y, vdd, gnd);
  input a;
  input b;
  output y;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire net_55, plno_2_well, plnode_0_well;

  tranif1 nmos_0(gnd, y, a);
  tranif1 nmos_1(y, gnd, b);
  tranif0 pmos_0(vdd, net_55, a);
  tranif0 pmos_1(net_55, y, b);
endmodule   /* muddlib07__nor2_1x */

module mips8__yzdetect_8(a, a_1, a_2, a_3, a_4, a_5, a_6, a_7, yzero, vdd, 
      vdd_1, vdd_1_1, vdd_2, vdd_3, vdd_4, vdd_5, gnd, gnd_1, gnd_1_1, gnd_2, 
      gnd_3, gnd_4, gnd_5);
  input [0:0] a;
  input [1:1] a_1;
  input [2:2] a_2;
  input [3:3] a_3;
  input [4:4] a_4;
  input [5:5] a_5;
  input [6:6] a_6;
  input [7:7] a_7;
  output yzero;
  input vdd;
  input vdd_1;
  input vdd_1_1;
  input vdd_2;
  input vdd_3;
  input vdd_4;
  input vdd_5;
  input gnd;
  input gnd_1;
  input gnd_1_1;
  input gnd_2;
  input gnd_3;
  input gnd_4;
  input gnd_5;

  supply1 vdd;
  supply0 gnd;
  wire net_11, net_16, net_2, net_21, net_5, net_8;

  muddlib07__nand2_1x nand2_1x_1(.a(net_11), .b(net_8), .y(net_21), 
      .vdd(vdd_1), .gnd(gnd_1));
  muddlib07__nand2_1x nand2_1x_2(.a(net_2), .b(net_5), .y(net_16), .vdd(vdd), 
      .gnd(gnd));
  muddlib07__nor2_1x nor2_1x_0(.a(a[0]), .b(a_1[1]), .y(net_2), .vdd(vdd_1_1), 
      .gnd(gnd_1_1));
  muddlib07__nor2_1x nor2_1x_1(.a(net_21), .b(net_16), .y(yzero), .vdd(vdd_2), 
      .gnd(gnd_2));
  muddlib07__nor2_1x nor2_1x_2(.a(a_2[2]), .b(a_3[3]), .y(net_5), .vdd(vdd_3), 
      .gnd(gnd_3));
  muddlib07__nor2_1x nor2_1x_3(.a(a_5[5]), .b(a_4[4]), .y(net_8), .vdd(vdd_4), 
      .gnd(gnd_4));
  muddlib07__nor2_1x nor2_1x_4(.a(a_6[6]), .b(a_7[7]), .y(net_11), .vdd(vdd_5), 
      .gnd(gnd_5));
endmodule   /* mips8__yzdetect_8 */

module mips8__alu(a, a_1, a_2, a_3, a_4, a_5, a_6, a_7, alucontrol, 
      alucontrol_1, alucontrol_2, b, b_1, b_2, b_3, b_4, b_5, b_6, b_7, result, 
      result_1, result_2, result_3, result_4, result_5, result_6, result_7, 
      zero, vdd, vdd_1, vdd_2, vdd_3, vdd_32, vdd_4, vdd_40, vdd_5, vdd_6, 
      vdd_7, gnd, gnd_1, gnd_2, gnd_3, gnd_32, gnd_4, gnd_40, gnd_5, gnd_6, 
      gnd_7);
  input [0:0] a;
  input [1:1] a_1;
  input [2:2] a_2;
  input [3:3] a_3;
  input [4:4] a_4;
  input [5:5] a_5;
  input [6:6] a_6;
  input [7:7] a_7;
  input [0:0] alucontrol;
  input [1:1] alucontrol_1;
  input [2:2] alucontrol_2;
  input [0:0] b;
  input [1:1] b_1;
  input [2:2] b_2;
  input [3:3] b_3;
  input [4:4] b_4;
  input [5:5] b_5;
  input [6:6] b_6;
  input [7:7] b_7;
  output [0:0] result;
  output [1:1] result_1;
  output [2:2] result_2;
  output [3:3] result_3;
  output [4:4] result_4;
  output [5:5] result_5;
  output [6:6] result_6;
  output [7:7] result_7;
  output zero;
  input vdd;
  input vdd_1;
  input vdd_2;
  input vdd_3;
  input vdd_32;
  input vdd_4;
  input vdd_40;
  input vdd_5;
  input vdd_6;
  input vdd_7;
  input gnd;
  input gnd_1;
  input gnd_2;
  input gnd_3;
  input gnd_32;
  input gnd_4;
  input gnd_40;
  input gnd_5;
  input gnd_6;
  input gnd_7;

  supply1 vdd;
  supply0 gnd;
  wire adder_8_1_cout, plnode_0_select, plnode_10_select, plnode_11_select;
  wire plnode_12_select, plnode_13_select, plnode_14_select, plnode_15_select;
  wire plnode_1_select, plnode_2_select, plnode_3_select, plnode_4_select;
  wire plnode_5_select, plnode_6_select, plnode_7_select, plnode_8_select;
  wire plnode_9_select;
  wire [7:0] andresult;
  wire [7:0] b2;
  wire [7:0] orresult;
  wire [7:0] sumresult;

  wordlib8__adder_8 adder_8_1(.a(a[0:0]), .a_1(a_1[1:1]), .a_2(a_2[2:2]), 
      .a_3(a_3[3:3]), .a_4(a_4[4:4]), .a_5(a_5[5:5]), .a_6(a_6[6:6]), 
      .a_7(a_7[7:7]), .b(b2[0:0]), .b_1(b2[1:1]), .b_2(b2[2:2]), .b_3(b2[3:3]), 
      .b_4(b2[4:4]), .b_5(b2[5:5]), .b_6(b2[6:6]), .b_7(b2[7:7]), 
      .cin(alucontrol_2[2]), .cout(adder_8_1_cout), .s(sumresult[0:0]), 
      .s_1(sumresult[1:1]), .s_2(sumresult[2:2]), .s_3(sumresult[3:3]), 
      .s_4(sumresult[4:4]), .s_5(sumresult[5:5]), .s_6(sumresult[6:6]), 
      .s_7(sumresult[7:7]), .vdd(vdd_7), .vdd_1(vdd), .vdd_1_1(vdd_1), 
      .vdd_2(vdd_2), .vdd_3(vdd_3), .vdd_4(vdd_4), .vdd_5(vdd_5), 
      .vdd_6(vdd_6), .gnd(gnd_7), .gnd_1(gnd_1), .gnd_1_1(gnd), .gnd_2(gnd_2), 
      .gnd_3(gnd_3), .gnd_4(gnd_4), .gnd_5(gnd_5), .gnd_6(gnd_6));
  wordlib8__and2_1x_8 and2_1x__0(.a(a[0:0]), .a_1(a_1[1:1]), .a_2(a_2[2:2]), 
      .a_3(a_3[3:3]), .a_4(a_4[4:4]), .a_5(a_5[5:5]), .a_6(a_6[6:6]), 
      .a_7(a_7[7:7]), .b(b[0:0]), .b_1(b_1[1:1]), .b_2(b_2[2:2]), 
      .b_3(b_3[3:3]), .b_4(b_4[4:4]), .b_5(b_5[5:5]), .b_6(b_6[6:6]), 
      .b_7(b_7[7:7]), .y(andresult[0:0]), .y_1(andresult[1:1]), 
      .y_2(andresult[2:2]), .y_3(andresult[3:3]), .y_4(andresult[4:4]), 
      .y_5(andresult[5:5]), .y_6(andresult[6:6]), .y_7(andresult[7:7]), 
      .vdd(vdd), .vdd_1(vdd_7), .vdd_1_1(vdd_1), .vdd_2(vdd_2), .vdd_3(vdd_3), 
      .vdd_4(vdd_4), .vdd_5(vdd_5), .vdd_6(vdd_6), .gnd(gnd_7), .gnd_1(gnd_1), 
      .gnd_1_1(gnd), .gnd_2(gnd_2), .gnd_3(gnd_3), .gnd_4(gnd_4), 
      .gnd_5(gnd_5), .gnd_6(gnd_6));
  mips8__condinv condinv_1(.a(b[0:0]), .a_1(b_1[1:1]), .a_2(b_2[2:2]), 
      .a_3(b_3[3:3]), .a_4(b_4[4:4]), .a_5(b_5[5:5]), .a_6(b_6[6:6]), 
      .a_7(b_7[7:7]), .invert(alucontrol_2[2]), .y(b2[0:0]), .y_1(b2[1:1]), 
      .y_2(b2[2:2]), .y_3(b2[3:3]), .y_4(b2[4:4]), .y_5(b2[5:5]), 
      .y_6(b2[6:6]), .y_7(b2[7:7]), .vdd(vdd), .vdd_1(vdd_7), .vdd_32(vdd_32), 
      .vdd_33(vdd_1), .vdd_34(vdd_2), .vdd_35(vdd_3), .vdd_36(vdd_4), 
      .vdd_37(vdd_5), .vdd_38(vdd_6), .gnd(gnd), .gnd_1(gnd_1), 
      .gnd_32(gnd_32), .gnd_33(gnd_7), .gnd_34(gnd_6), .gnd_35(gnd_5), 
      .gnd_36(gnd_4), .gnd_37(gnd_3), .gnd_38(gnd_2));
  wordlib8__mux4_1x_8 mux4_1x__1(.d0(andresult[0:0]), .d0_1(andresult[1:1]), 
      .d0_2(andresult[2:2]), .d0_3(andresult[3:3]), .d0_4(andresult[4:4]), 
      .d0_5(andresult[5:5]), .d0_6(andresult[6:6]), .d0_7(andresult[7:7]), 
      .d1(orresult[0:0]), .d1_1(orresult[1:1]), .d1_2(orresult[2:2]), 
      .d1_3(orresult[3:3]), .d1_4(orresult[4:4]), .d1_5(orresult[5:5]), 
      .d1_6(orresult[6:6]), .d1_7(orresult[7:7]), .d2(sumresult[0:0]), 
      .d2_1(sumresult[1:1]), .d2_2(sumresult[2:2]), .d2_3(sumresult[3:3]), 
      .d2_4(sumresult[4:4]), .d2_5(sumresult[5:5]), .d2_6(sumresult[6:6]), 
      .d2_7(sumresult[7:7]), .d3(sumresult[7:7]), .d3_1({gnd}), .d3_2({gnd_2}), 
      .d3_3({gnd_3}), .d3_4({gnd_4}), .d3_5({gnd_5}), .d3_6({gnd_6}), 
      .d3_7({gnd_7}), .s0(alucontrol[0]), .s1(alucontrol_1[1]), 
      .y(result[0:0]), .y_1(result_1[1:1]), .y_2(result_2[2:2]), 
      .y_3(result_3[3:3]), .y_4(result_4[4:4]), .y_5(result_5[5:5]), 
      .y_6(result_6[6:6]), .y_7(result_7[7:7]), .vdd(vdd_7), .vdd_1(vdd_40), 
      .vdd_2(vdd), .vdd_34(vdd_1), .vdd_35(vdd_2), .vdd_36(vdd_3), 
      .vdd_37(vdd_4), .vdd_38(vdd_5), .vdd_39(vdd_6), .gnd(gnd_40), 
      .gnd_1(gnd_7), .gnd_2(gnd_1), .gnd_34(gnd), .gnd_35(gnd_2), 
      .gnd_36(gnd_3), .gnd_37(gnd_4), .gnd_38(gnd_5), .gnd_39(gnd_6));
  wordlib8__or2_1x_8 or2_1x_8_0(.a(a[0:0]), .a_1(a_1[1:1]), .a_2(a_2[2:2]), 
      .a_3(a_3[3:3]), .a_4(a_4[4:4]), .a_5(a_5[5:5]), .a_6(a_6[6:6]), 
      .a_7(a_7[7:7]), .b(b[0:0]), .b_1(b_1[1:1]), .b_2(b_2[2:2]), 
      .b_3(b_3[3:3]), .b_4(b_4[4:4]), .b_5(b_5[5:5]), .b_6(b_6[6:6]), 
      .b_7(b_7[7:7]), .y(orresult[0:0]), .y_1(orresult[1:1]), 
      .y_2(orresult[2:2]), .y_3(orresult[3:3]), .y_4(orresult[4:4]), 
      .y_5(orresult[5:5]), .y_6(orresult[6:6]), .y_7(orresult[7:7]), .vdd(vdd), 
      .vdd_1(vdd_7), .vdd_1_1(vdd_1), .vdd_2(vdd_2), .vdd_3(vdd_3), 
      .vdd_4(vdd_4), .vdd_5(vdd_5), .vdd_6(vdd_6), .gnd(gnd_1), .gnd_1(gnd_7), 
      .gnd_1_1(gnd), .gnd_2(gnd_2), .gnd_3(gnd_3), .gnd_4(gnd_4), 
      .gnd_5(gnd_5), .gnd_6(gnd_6));
  mips8__yzdetect_8 yzdetect_1(.a(result[0:0]), .a_1(result_1[1:1]), 
      .a_2(result_2[2:2]), .a_3(result_3[3:3]), .a_4(result_4[4:4]), 
      .a_5(result_5[5:5]), .a_6(result_6[6:6]), .a_7(result_7[7:7]), 
      .yzero(zero), .vdd(vdd_1), .vdd_1(vdd_5), .vdd_1_1(vdd), .vdd_2(vdd_3), 
      .vdd_3(vdd_2), .vdd_4(vdd_4), .vdd_5(vdd_6), .gnd(gnd), .gnd_1(gnd_5), 
      .gnd_1_1(gnd_1), .gnd_2(gnd_3), .gnd_3(gnd_2), .gnd_4(gnd_4), 
      .gnd_5(gnd_6));
endmodule   /* mips8__alu */

module muddlib07__clkinvbuf_4x(ph, phb, phbuf, vdd, gnd);
  output ph;
  output phb;
  output phbuf;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire notph, plnode_0_well, plnode_1_well;

  tranif1 nmos_0(gnd, notph, ph);
  tranif1 nmos_1(phb, gnd, ph);
  tranif1 nmos_2(gnd, phbuf, notph);
  tranif0 pmos_0(vdd, notph, ph);
  tranif0 pmos_1(phb, vdd, ph);
  tranif0 pmos_2(vdd, phbuf, notph);
endmodule   /* muddlib07__clkinvbuf_4x */

module muddlib07__clkinvbufdual_4x(ph1, ph2, ph1b, ph1buf, ph2b, ph2buf, vdd, 
      gnd);
  input ph1;
  input ph2;
  output ph1b;
  output ph1buf;
  output ph2b;
  output ph2buf;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  muddlib07__clkinvbuf_4x clkinvbu_0(.ph(ph2), .phb(ph2b), .phbuf(ph2buf), 
      .vdd(vdd), .gnd(gnd));
  muddlib07__clkinvbuf_4x clkinvbu_1(.ph(ph1), .phb(ph1b), .phbuf(ph1buf), 
      .vdd(vdd), .gnd(gnd));
endmodule   /* muddlib07__clkinvbufdual_4x */

module muddlib07__flop_dp_1x(d, ph1, ph1b, ph2, ph2b, q, vdd, gnd);
  input d;
  input ph1;
  input ph1b;
  input ph2;
  input ph2b;
  output q;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire master, masterinb, net_498, net_502, net_552, net_555, net_557;
  wire plno_2_well, plnode_0_well;
  trireg masterb, slave;

  tranif1 nmos_2(gnd, masterinb, d);
  tranif1 nmos_18(masterinb, masterb, ph2);
  tranif1 nmos_19(masterb, net_498, ph2b);
  tranif1 nmos_20(net_498, gnd, master);
  tranif1 nmos_21(gnd, master, masterb);
  rtranif1 nmos_22(master, slave, ph1);
  tranif1 nmos_23(slave, net_552, ph1b);
  tranif1 nmos_24(net_552, gnd, net_557);
  tranif1 nmos_25(gnd, net_557, slave);
  tranif1 nmos_26(gnd, q, net_557);
  tranif0 pmos_2(vdd, masterinb, d);
  tranif0 pmos_17(masterinb, masterb, ph2b);
  tranif0 pmos_18(masterb, net_502, ph2);
  tranif0 pmos_19(net_502, vdd, master);
  tranif0 pmos_20(vdd, master, masterb);
  rtranif0 pmos_21(master, slave, ph1b);
  tranif0 pmos_22(slave, net_555, ph1);
  tranif0 pmos_23(net_555, vdd, net_557);
  tranif0 pmos_24(vdd, net_557, slave);
  tranif0 pmos_25(vdd, q, net_557);
endmodule   /* muddlib07__flop_dp_1x */

module wordlib8__flop_1x_8(d, d_1, d_2, d_3, d_4, d_5, d_6, d_7, ph1, ph2, q, 
      q_1, q_2, q_3, q_4, q_5, q_6, q_7, vdd, vdd_1, vdd_10, vdd_11, vdd_12, 
      vdd_13, vdd_14, vdd_15, vdd_9, gnd, gnd_1, gnd_10, gnd_11, gnd_12, 
      gnd_13, gnd_14, gnd_15, gnd_9);
  input [0:0] d;
  input [1:1] d_1;
  input [2:2] d_2;
  input [3:3] d_3;
  input [4:4] d_4;
  input [5:5] d_5;
  input [6:6] d_6;
  input [7:7] d_7;
  input ph1;
  input ph2;
  output [0:0] q;
  output [1:1] q_1;
  output [2:2] q_2;
  output [3:3] q_3;
  output [4:4] q_4;
  output [5:5] q_5;
  output [6:6] q_6;
  output [7:7] q_7;
  input vdd;
  input vdd_1;
  input vdd_10;
  input vdd_11;
  input vdd_12;
  input vdd_13;
  input vdd_14;
  input vdd_15;
  input vdd_9;
  input gnd;
  input gnd_1;
  input gnd_10;
  input gnd_11;
  input gnd_12;
  input gnd_13;
  input gnd_14;
  input gnd_15;
  input gnd_9;

  supply1 vdd;
  supply0 gnd;
  wire net_242, net_244, net_248, net_341;

  muddlib07__clkinvbufdual_4x clkinvbu_1(.ph1(ph1), .ph2(ph2), .ph1b(net_242), 
      .ph1buf(net_244), .ph2b(net_341), .ph2buf(net_248), .vdd(vdd), 
      .gnd(gnd));
  muddlib07__flop_dp_1x flop_dp__0(.d(d[0]), .ph1(net_244), .ph1b(net_242), 
      .ph2(net_248), .ph2b(net_341), .q(q[0]), .vdd(vdd_1), .gnd(gnd_1));
  muddlib07__flop_dp_1x flop_dp__8(.d(d_1[1]), .ph1(net_244), .ph1b(net_242), 
      .ph2(net_248), .ph2b(net_341), .q(q_1[1]), .vdd(vdd_9), .gnd(gnd_9));
  muddlib07__flop_dp_1x flop_dp__9(.d(d_2[2]), .ph1(net_244), .ph1b(net_242), 
      .ph2(net_248), .ph2b(net_341), .q(q_2[2]), .vdd(vdd_10), .gnd(gnd_10));
  muddlib07__flop_dp_1x flop_dp__10(.d(d_3[3]), .ph1(net_244), .ph1b(net_242), 
      .ph2(net_248), .ph2b(net_341), .q(q_3[3]), .vdd(vdd_11), .gnd(gnd_11));
  muddlib07__flop_dp_1x flop_dp__11(.d(d_4[4]), .ph1(net_244), .ph1b(net_242), 
      .ph2(net_248), .ph2b(net_341), .q(q_4[4]), .vdd(vdd_12), .gnd(gnd_12));
  muddlib07__flop_dp_1x flop_dp__12(.d(d_5[5]), .ph1(net_244), .ph1b(net_242), 
      .ph2(net_248), .ph2b(net_341), .q(q_5[5]), .vdd(vdd_13), .gnd(gnd_13));
  muddlib07__flop_dp_1x flop_dp__13(.d(d_6[6]), .ph1(net_244), .ph1b(net_242), 
      .ph2(net_248), .ph2b(net_341), .q(q_6[6]), .vdd(vdd_14), .gnd(gnd_14));
  muddlib07__flop_dp_1x flop_dp__14(.d(d_7[7]), .ph1(net_244), .ph1b(net_242), 
      .ph2(net_248), .ph2b(net_341), .q(q_7[7]), .vdd(vdd_15), .gnd(gnd_15));
endmodule   /* wordlib8__flop_1x_8 */

module muddlib07__flopenr_dp_1x(d, en, enb, ph1, ph1b, ph2, ph2b, resetb, q, 
      vdd, gnd);
  input d;
  input en;
  input enb;
  input ph1;
  input ph1b;
  input ph2;
  input ph2b;
  input resetb;
  output q;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire master, masterinb, n1, n2, net_239, net_498, net_502, net_552, net_555;
  wire net_557, net_72, net_75, plno_2_well, plnode_0_well;
  trireg masterb, slave;

  tranif1 nmos_0(gnd, n1, resetb);
  tranif1 nmos_1(n1, n2, en);
  tranif1 nmos_2(n2, masterinb, d);
  tranif1 nmos_3(masterinb, net_239, slave);
  tranif1 nmos_5(net_239, n1, enb);
  tranif1 nmos_18(masterinb, masterb, ph2);
  tranif1 nmos_19(masterb, net_498, ph2b);
  tranif1 nmos_20(net_498, gnd, master);
  tranif1 nmos_21(gnd, master, masterb);
  rtranif1 nmos_22(master, slave, ph1);
  tranif1 nmos_23(slave, net_552, ph1b);
  tranif1 nmos_24(net_552, gnd, net_557);
  tranif1 nmos_25(gnd, net_557, slave);
  tranif1 nmos_26(gnd, q, net_557);
  tranif0 pmos_0(vdd, net_72, enb);
  tranif0 pmos_2(net_72, masterinb, d);
  tranif0 pmos_3(masterinb, net_75, slave);
  tranif0 pmos_4(net_75, vdd, en);
  tranif0 pmos_6(vdd, masterinb, resetb);
  tranif0 pmos_17(masterinb, masterb, ph2b);
  tranif0 pmos_18(masterb, net_502, ph2);
  tranif0 pmos_19(net_502, vdd, master);
  tranif0 pmos_20(vdd, master, masterb);
  rtranif0 pmos_21(master, slave, ph1b);
  tranif0 pmos_22(slave, net_555, ph1);
  tranif0 pmos_23(net_555, vdd, net_557);
  tranif0 pmos_24(vdd, net_557, slave);
  tranif0 pmos_25(vdd, q, net_557);
endmodule   /* muddlib07__flopenr_dp_1x */

module muddlib07__inv_4x(a, y, vdd, gnd);
  input a;
  output y;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire plnode_0_well, plnode_1_well;

  tranif1 nmos_0(gnd, y, a);
  tranif0 pmos_0(vdd, y, a);
endmodule   /* muddlib07__inv_4x */

module wordlib8__flopenr_1x_8(d, d_1, d_2, d_3, d_4, d_5, d_6, d_7, en, ph1, 
      ph2, reset, q, q_1, q_2, q_3, q_4, q_5, q_6, q_7, vdd, vdd_1, vdd_32, 
      vdd_33, vdd_34, vdd_35, vdd_36, vdd_37, vdd_38, gnd, gnd_1, gnd_32, 
      gnd_33, gnd_34, gnd_35, gnd_36, gnd_37, gnd_38);
  input [0:0] d;
  input [1:1] d_1;
  input [2:2] d_2;
  input [3:3] d_3;
  input [4:4] d_4;
  input [5:5] d_5;
  input [6:6] d_6;
  input [7:7] d_7;
  input en;
  input ph1;
  input ph2;
  input reset;
  output [0:0] q;
  output [1:1] q_1;
  output [2:2] q_2;
  output [3:3] q_3;
  output [4:4] q_4;
  output [5:5] q_5;
  output [6:6] q_6;
  output [7:7] q_7;
  input vdd;
  input vdd_1;
  input vdd_32;
  input vdd_33;
  input vdd_34;
  input vdd_35;
  input vdd_36;
  input vdd_37;
  input vdd_38;
  input gnd;
  input gnd_1;
  input gnd_32;
  input gnd_33;
  input gnd_34;
  input gnd_35;
  input gnd_36;
  input gnd_37;
  input gnd_38;

  supply1 vdd;
  supply0 gnd;
  wire net_223, net_226, net_228, net_299, net_302, net_370, net_377;

  muddlib07__clkinvbufdual_4x clkinvbu_0(.ph1(ph1), .ph2(ph2), .ph1b(net_377), 
      .ph1buf(net_370), .ph2b(net_302), .ph2buf(net_299), .vdd(vdd_32), 
      .gnd(gnd_32));
  muddlib07__flopenr_dp_1x flopenr__0(.d(d[0]), .en(net_226), .enb(net_223), 
      .ph1(net_370), .ph1b(net_377), .ph2(net_299), .ph2b(net_302), 
      .resetb(net_228), .q(q[0]), .vdd(vdd), .gnd(gnd));
  muddlib07__flopenr_dp_1x flopenr__32(.d(d_1[1]), .en(net_226), .enb(net_223), 
      .ph1(net_370), .ph1b(net_377), .ph2(net_299), .ph2b(net_302), 
      .resetb(net_228), .q(q_1[1]), .vdd(vdd_33), .gnd(gnd_33));
  muddlib07__flopenr_dp_1x flopenr__33(.d(d_2[2]), .en(net_226), .enb(net_223), 
      .ph1(net_370), .ph1b(net_377), .ph2(net_299), .ph2b(net_302), 
      .resetb(net_228), .q(q_2[2]), .vdd(vdd_34), .gnd(gnd_34));
  muddlib07__flopenr_dp_1x flopenr__34(.d(d_3[3]), .en(net_226), .enb(net_223), 
      .ph1(net_370), .ph1b(net_377), .ph2(net_299), .ph2b(net_302), 
      .resetb(net_228), .q(q_3[3]), .vdd(vdd_35), .gnd(gnd_35));
  muddlib07__flopenr_dp_1x flopenr__35(.d(d_4[4]), .en(net_226), .enb(net_223), 
      .ph1(net_370), .ph1b(net_377), .ph2(net_299), .ph2b(net_302), 
      .resetb(net_228), .q(q_4[4]), .vdd(vdd_36), .gnd(gnd_36));
  muddlib07__flopenr_dp_1x flopenr__36(.d(d_5[5]), .en(net_226), .enb(net_223), 
      .ph1(net_370), .ph1b(net_377), .ph2(net_299), .ph2b(net_302), 
      .resetb(net_228), .q(q_5[5]), .vdd(vdd_37), .gnd(gnd_37));
  muddlib07__flopenr_dp_1x flopenr__37(.d(d_6[6]), .en(net_226), .enb(net_223), 
      .ph1(net_370), .ph1b(net_377), .ph2(net_299), .ph2b(net_302), 
      .resetb(net_228), .q(q_6[6]), .vdd(vdd_38), .gnd(gnd_38));
  muddlib07__flopenr_dp_1x flopenr__38(.d(d_7[7]), .en(net_226), .enb(net_223), 
      .ph1(net_370), .ph1b(net_377), .ph2(net_299), .ph2b(net_302), 
      .resetb(net_228), .q(q_7[7]), .vdd(vdd_1), .gnd(gnd_1));
  muddlib07__inv_4x inv_4x_0(.a(reset), .y(net_228), .vdd(vdd_32), 
      .gnd(gnd_32));
  muddlib07__inv_4x inv_4x_1(.a(net_223), .y(net_226), .vdd(vdd_32), 
      .gnd(gnd_32));
  muddlib07__inv_4x inv_4x_2(.a(en), .y(net_223), .vdd(vdd_32), .gnd(gnd_32));
endmodule   /* wordlib8__flopenr_1x_8 */

module muddlib07__flopen_dp_1x(d, en, enb, ph1, ph1b, ph2, ph2b, q, vdd, gnd);
  input d;
  input en;
  input enb;
  input ph1;
  input ph1b;
  input ph2;
  input ph2b;
  output q;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire master, masterinb, net_239, net_498, net_502, net_552, net_555, net_557;
  wire net_72, net_75, net_775, plno_2_well, plnode_0_well;
  trireg masterb, slave;

  tranif1 nmos_1(gnd, net_775, en);
  tranif1 nmos_2(net_775, masterinb, d);
  tranif1 nmos_3(masterinb, net_239, slave);
  tranif1 nmos_5(net_239, gnd, enb);
  tranif1 nmos_18(masterinb, masterb, ph2);
  tranif1 nmos_19(masterb, net_498, ph2b);
  tranif1 nmos_20(net_498, gnd, master);
  tranif1 nmos_21(gnd, master, masterb);
  rtranif1 nmos_22(master, slave, ph1);
  tranif1 nmos_23(slave, net_552, ph1b);
  tranif1 nmos_24(net_552, gnd, net_557);
  tranif1 nmos_25(gnd, net_557, slave);
  tranif1 nmos_26(gnd, q, net_557);
  tranif0 pmos_0(vdd, net_72, enb);
  tranif0 pmos_2(net_72, masterinb, d);
  tranif0 pmos_3(masterinb, net_75, slave);
  tranif0 pmos_4(net_75, vdd, en);
  tranif0 pmos_17(masterinb, masterb, ph2b);
  tranif0 pmos_18(masterb, net_502, ph2);
  tranif0 pmos_19(net_502, vdd, master);
  tranif0 pmos_20(vdd, master, masterb);
  rtranif0 pmos_21(master, slave, ph1b);
  tranif0 pmos_22(slave, net_555, ph1);
  tranif0 pmos_23(net_555, vdd, net_557);
  tranif0 pmos_24(vdd, net_557, slave);
  tranif0 pmos_25(vdd, q, net_557);
endmodule   /* muddlib07__flopen_dp_1x */

module wordlib8__flopen_1x_8(d, d_1, d_2, d_3, d_4, d_5, d_6, d_7, en, ph1, 
      ph2, q, q_1, q_2, q_3, q_4, q_5, q_6, q_7, vdd, vdd_1, vdd_1_1, vdd_2, 
      vdd_3, vdd_4, vdd_5, vdd_6, vdd_7, gnd, gnd_1, gnd_1_1, gnd_2, gnd_3, 
      gnd_4, gnd_5, gnd_6, gnd_7);
  input [0:0] d;
  input [1:1] d_1;
  input [2:2] d_2;
  input [3:3] d_3;
  input [4:4] d_4;
  input [5:5] d_5;
  input [6:6] d_6;
  input [7:7] d_7;
  input en;
  input ph1;
  input ph2;
  output [0:0] q;
  output [1:1] q_1;
  output [2:2] q_2;
  output [3:3] q_3;
  output [4:4] q_4;
  output [5:5] q_5;
  output [6:6] q_6;
  output [7:7] q_7;
  input vdd;
  input vdd_1;
  input vdd_1_1;
  input vdd_2;
  input vdd_3;
  input vdd_4;
  input vdd_5;
  input vdd_6;
  input vdd_7;
  input gnd;
  input gnd_1;
  input gnd_1_1;
  input gnd_2;
  input gnd_3;
  input gnd_4;
  input gnd_5;
  input gnd_6;
  input gnd_7;

  supply1 vdd;
  supply0 gnd;
  wire net_232, net_237, net_242, net_244, net_248, net_272;

  muddlib07__clkinvbufdual_4x clkinvbu_1(.ph1(ph1), .ph2(ph2), .ph1b(net_242), 
      .ph1buf(net_244), .ph2b(net_272), .ph2buf(net_248), .vdd(vdd_1), 
      .gnd(gnd_1));
  muddlib07__flopen_dp_1x flopen_d_32(.d(d_1[1]), .en(net_237), .enb(net_232), 
      .ph1(net_244), .ph1b(net_242), .ph2(net_248), .ph2b(net_272), .q(q_1[1]), 
      .vdd(vdd), .gnd(gnd));
  muddlib07__flopen_dp_1x flopen_d_33(.d(d_2[2]), .en(net_237), .enb(net_232), 
      .ph1(net_244), .ph1b(net_242), .ph2(net_248), .ph2b(net_272), .q(q_2[2]), 
      .vdd(vdd_1_1), .gnd(gnd_1_1));
  muddlib07__flopen_dp_1x flopen_d_34(.d(d_3[3]), .en(net_237), .enb(net_232), 
      .ph1(net_244), .ph1b(net_242), .ph2(net_248), .ph2b(net_272), .q(q_3[3]), 
      .vdd(vdd_2), .gnd(gnd_2));
  muddlib07__flopen_dp_1x flopen_d_35(.d(d_4[4]), .en(net_237), .enb(net_232), 
      .ph1(net_244), .ph1b(net_242), .ph2(net_248), .ph2b(net_272), .q(q_4[4]), 
      .vdd(vdd_3), .gnd(gnd_3));
  muddlib07__flopen_dp_1x flopen_d_36(.d(d_5[5]), .en(net_237), .enb(net_232), 
      .ph1(net_244), .ph1b(net_242), .ph2(net_248), .ph2b(net_272), .q(q_5[5]), 
      .vdd(vdd_4), .gnd(gnd_4));
  muddlib07__flopen_dp_1x flopen_d_37(.d(d_6[6]), .en(net_237), .enb(net_232), 
      .ph1(net_244), .ph1b(net_242), .ph2(net_248), .ph2b(net_272), .q(q_6[6]), 
      .vdd(vdd_5), .gnd(gnd_5));
  muddlib07__flopen_dp_1x flopen_d_38(.d(d_7[7]), .en(net_237), .enb(net_232), 
      .ph1(net_244), .ph1b(net_242), .ph2(net_248), .ph2b(net_272), .q(q_7[7]), 
      .vdd(vdd_6), .gnd(gnd_6));
  muddlib07__flopen_dp_1x flopen_d_39(.d(d[0]), .en(net_237), .enb(net_232), 
      .ph1(net_244), .ph1b(net_242), .ph2(net_248), .ph2b(net_272), .q(q[0]), 
      .vdd(vdd_7), .gnd(gnd_7));
  muddlib07__inv_4x inv_4x_2(.a(net_232), .y(net_237), .vdd(vdd_1), 
      .gnd(gnd_1));
  muddlib07__inv_4x inv_4x_3(.a(en), .y(net_232), .vdd(vdd_1), .gnd(gnd_1));
endmodule   /* wordlib8__flopen_1x_8 */

module muddlib07__mux2_c_1x(d0, d1, s, y, vdd, gnd);
  input d0;
  input d1;
  input s;
  output y;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire net_114, net_186, net_187, net_188, net_189, net_80, plno_2_well;
  wire plnode_0_well;

  tranif1 nmos_1(gnd, net_189, d1);
  tranif1 nmos_2(net_189, net_80, s);
  tranif1 nmos_3(net_80, net_188, net_114);
  tranif1 nmos_4(net_188, gnd, d0);
  tranif1 nmos_5(net_114, gnd, s);
  tranif1 nmos_6(gnd, y, net_80);
  tranif0 pmos_0(vdd, net_187, d1);
  tranif0 pmos_1(net_187, net_80, net_114);
  tranif0 pmos_2(net_80, net_186, s);
  tranif0 pmos_3(net_186, vdd, d0);
  tranif0 pmos_4(net_114, vdd, s);
  tranif0 pmos_5(vdd, y, net_80);
endmodule   /* muddlib07__mux2_c_1x */

module muddlib07__mux3_dp_1x(d0, d1, d2, s0, s0b, s1, s1b, y, vdd, gnd);
  input d0;
  input d1;
  input d2;
  input s0;
  input s0b;
  input s1;
  input s1b;
  output y;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire net_0, net_15, net_155, net_160, net_22, net_24, net_30, net_44, net_87;
  wire plnode_0_well, plnode_1_well;

  tranif1 nmos_0(net_15, net_22, s0);
  tranif1 nmos_1(net_22, gnd, d1);
  tranif1 nmos_2(gnd, net_24, d0);
  tranif1 nmos_3(net_24, net_15, s0b);
  tranif1 nmos_4(net_15, net_44, s1b);
  tranif1 nmos_5(net_44, net_160, s1);
  tranif1 nmos_6(net_160, gnd, d2);
  tranif1 nmos_7(y, gnd, net_44);
  tranif0 pmos_0(vdd, y, net_44);
  tranif0 pmos_1(net_0, net_44, s1);
  tranif0 pmos_2(net_155, net_0, s0b);
  tranif0 pmos_3(vdd, net_155, d1);
  tranif0 pmos_4(net_87, vdd, d0);
  tranif0 pmos_5(net_0, net_87, s0);
  tranif0 pmos_6(net_44, net_30, s1b);
  tranif0 pmos_7(net_30, vdd, d2);
endmodule   /* muddlib07__mux3_dp_1x */

module wordlib8__mux3_1x_8(d0, d0_1, d0_2, d0_3, d0_4, d0_5, d0_6, d0_7, d1, 
      d1_1, d1_2, d1_3, d1_4, d1_5, d1_6, d1_7, d2, d2_1, d2_2, d2_3, d2_4, 
      d2_5, d2_6, d2_7, s0, s1, y, y_1, y_2, y_3, y_4, y_5, y_6, y_7, vdd, 
      vdd_1, vdd_32, vdd_33, vdd_34, vdd_35, vdd_36, vdd_37, vdd_38, gnd, 
      gnd_1, gnd_32, gnd_33, gnd_34, gnd_35, gnd_36, gnd_37, gnd_38);
  input [0:0] d0;
  input [1:1] d0_1;
  input [2:2] d0_2;
  input [3:3] d0_3;
  input [4:4] d0_4;
  input [5:5] d0_5;
  input [6:6] d0_6;
  input [7:7] d0_7;
  input [0:0] d1;
  input [1:1] d1_1;
  input [2:2] d1_2;
  input [3:3] d1_3;
  input [4:4] d1_4;
  input [5:5] d1_5;
  input [6:6] d1_6;
  input [7:7] d1_7;
  input [0:0] d2;
  input [1:1] d2_1;
  input [2:2] d2_2;
  input [3:3] d2_3;
  input [4:4] d2_4;
  input [5:5] d2_5;
  input [6:6] d2_6;
  input [7:7] d2_7;
  input s0;
  input s1;
  output [0:0] y;
  output [1:1] y_1;
  output [2:2] y_2;
  output [3:3] y_3;
  output [4:4] y_4;
  output [5:5] y_5;
  output [6:6] y_6;
  output [7:7] y_7;
  input vdd;
  input vdd_1;
  input vdd_32;
  input vdd_33;
  input vdd_34;
  input vdd_35;
  input vdd_36;
  input vdd_37;
  input vdd_38;
  input gnd;
  input gnd_1;
  input gnd_32;
  input gnd_33;
  input gnd_34;
  input gnd_35;
  input gnd_36;
  input gnd_37;
  input gnd_38;

  supply1 vdd;
  supply0 gnd;
  wire net_161, net_168, net_175, net_182;

  muddlib07__invbuf_4x invbuf_4_0(.s(s0), .s_out(net_161), .sb_out(net_168), 
      .vdd(vdd_32), .gnd(gnd_32));
  muddlib07__invbuf_4x invbuf_4_1(.s(s1), .s_out(net_182), .sb_out(net_175), 
      .vdd(vdd_32), .gnd(gnd_32));
  muddlib07__mux3_dp_1x mux3_dp__0(.d0(d0[0]), .d1(d1[0]), .d2(d2[0]), 
      .s0(net_161), .s0b(net_168), .s1(net_182), .s1b(net_175), .y(y[0]), 
      .vdd(vdd), .gnd(gnd));
  muddlib07__mux3_dp_1x mux3_dp__32(.d0(d0_1[1]), .d1(d1_1[1]), .d2(d2_1[1]), 
      .s0(net_161), .s0b(net_168), .s1(net_182), .s1b(net_175), .y(y_1[1]), 
      .vdd(vdd_33), .gnd(gnd_33));
  muddlib07__mux3_dp_1x mux3_dp__33(.d0(d0_2[2]), .d1(d1_2[2]), .d2(d2_2[2]), 
      .s0(net_161), .s0b(net_168), .s1(net_182), .s1b(net_175), .y(y_2[2]), 
      .vdd(vdd_34), .gnd(gnd_34));
  muddlib07__mux3_dp_1x mux3_dp__34(.d0(d0_3[3]), .d1(d1_3[3]), .d2(d2_3[3]), 
      .s0(net_161), .s0b(net_168), .s1(net_182), .s1b(net_175), .y(y_3[3]), 
      .vdd(vdd_35), .gnd(gnd_35));
  muddlib07__mux3_dp_1x mux3_dp__35(.d0(d0_4[4]), .d1(d1_4[4]), .d2(d2_4[4]), 
      .s0(net_161), .s0b(net_168), .s1(net_182), .s1b(net_175), .y(y_4[4]), 
      .vdd(vdd_36), .gnd(gnd_36));
  muddlib07__mux3_dp_1x mux3_dp__36(.d0(d0_5[5]), .d1(d1_5[5]), .d2(d2_5[5]), 
      .s0(net_161), .s0b(net_168), .s1(net_182), .s1b(net_175), .y(y_5[5]), 
      .vdd(vdd_37), .gnd(gnd_37));
  muddlib07__mux3_dp_1x mux3_dp__37(.d0(d0_6[6]), .d1(d1_6[6]), .d2(d2_6[6]), 
      .s0(net_161), .s0b(net_168), .s1(net_182), .s1b(net_175), .y(y_6[6]), 
      .vdd(vdd_38), .gnd(gnd_38));
  muddlib07__mux3_dp_1x mux3_dp__38(.d0(d0_7[7]), .d1(d1_7[7]), .d2(d2_7[7]), 
      .s0(net_161), .s0b(net_168), .s1(net_182), .s1b(net_175), .y(y_7[7]), 
      .vdd(vdd_1), .gnd(gnd_1));
endmodule   /* wordlib8__mux3_1x_8 */

module muddlib07__nand3_1x(a, b, c, y, vdd, gnd);
  input a;
  input b;
  input c;
  output y;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire net_177, net_178, plno_2_well, plnode_0_well;

  tranif1 nmos_6(gnd, net_177, a);
  tranif1 nmos_7(net_177, net_178, b);
  tranif1 nmos_8(net_178, y, c);
  tranif0 pmos_6(vdd, y, c);
  tranif0 pmos_7(y, vdd, b);
  tranif0 pmos_8(vdd, y, a);
endmodule   /* muddlib07__nand3_1x */

module muddlib07__nand5_1x(a, b, c, d, e, y, vdd, gnd);
  input a;
  input b;
  input c;
  input d;
  input e;
  output y;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire net_2, net_3, net_4, net_71, plnode_0_well, plnode_1_well;

  tranif1 nmos_0(net_4, net_3, b);
  tranif1 nmos_1(net_3, net_2, c);
  tranif1 nmos_2(net_2, net_71, d);
  tranif1 nmos_3(gnd, net_4, a);
  tranif1 nmos_4(net_71, y, e);
  tranif0 pmos_0(vdd, y, a);
  tranif0 pmos_1(y, vdd, b);
  tranif0 pmos_2(vdd, y, c);
  tranif0 pmos_3(y, vdd, d);
  tranif0 pmos_4(vdd, y, e);
endmodule   /* muddlib07__nand5_1x */

module mips8__regram_zipper(RegWrite, ph2, r1a, r1a_1, r1a_2, r2a, r2a_1, 
      r2a_2, wa, wa_1, wa_2, read1, read1b, read2, read2b, write, writeb, vdd, 
      vdd_1, vdd_1_1, vdd_2, gnd, gnd_1, gnd_1_1, gnd_2);
  input RegWrite;
  input ph2;
  input [0:0] r1a;
  input [1:1] r1a_1;
  input [2:2] r1a_2;
  input [0:0] r2a;
  input [1:1] r2a_1;
  input [2:2] r2a_2;
  input [0:0] wa;
  input [1:1] wa_1;
  input [2:2] wa_2;
  output read1;
  output read1b;
  output read2;
  output read2b;
  output write;
  output writeb;
  input vdd;
  input vdd_1;
  input vdd_1_1;
  input vdd_2;
  input gnd;
  input gnd_1;
  input gnd_1_1;
  input gnd_2;

  supply1 vdd;
  supply0 gnd;
  wire net_25, net_27, net_32;

  muddlib07__invbuf_4x invbuf_4_0(.s(net_27), .s_out(writeb), .sb_out(write), 
      .vdd(vdd), .gnd(gnd));
  muddlib07__invbuf_4x invbuf_4_1(.s(net_25), .s_out(read1b), .sb_out(read1), 
      .vdd(vdd_2), .gnd(gnd_2));
  muddlib07__invbuf_4x invbuf_4_3(.s(net_32), .s_out(read2b), .sb_out(read2), 
      .vdd(vdd), .gnd(gnd));
  muddlib07__nand3_1x nand3_1x_0(.a(r2a[0]), .b(r2a_1[1]), .c(r2a_2[2]), 
      .y(net_32), .vdd(vdd_1), .gnd(gnd_1));
  muddlib07__nand3_1x nand3_1x_1(.a(r1a[0]), .b(r1a_1[1]), .c(r1a_2[2]), 
      .y(net_25), .vdd(vdd_2), .gnd(gnd_2));
  muddlib07__nand5_1x nand5_1x_0(.a(wa[0]), .b(wa_1[1]), .c(wa_2[2]), .d(ph2), 
      .e(RegWrite), .y(net_27), .vdd(vdd_1_1), .gnd(gnd_1_1));
endmodule   /* mips8__regram_zipper */

module mips8__regram_dp(read1, read1b, read2, read2b, w, write, writeb, r1, r2, 
      vdd, gnd);
  input read1;
  input read1b;
  input read2;
  input read2b;
  input w;
  input write;
  input writeb;
  output r1;
  output r2;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire net_0, net_1, net_107, net_118, net_129, net_86, net_95, net_96;
  wire plnode_0_well, plnode_1_well, plnode_2_select, plnode_3_select;

  tranif1 nmos_0(net_118, r2, read2);
  tranif1 nmos_1(gnd, net_118, net_0);
  tranif1 nmos_2(w, net_1, write);
  tranif1 nmos_3(net_1, net_95, net_0);
  tranif1 nmos_4(net_95, gnd, writeb);
  tranif1 nmos_5(gnd, net_0, net_1);
  tranif1 nmos_6(net_96, gnd, net_0);
  tranif1 nmos_7(r1, net_96, read1);
  tranif0 pmos_0(net_86, vdd, write);
  tranif0 pmos_1(vdd, net_0, net_1);
  tranif0 pmos_2(net_129, r2, read2b);
  tranif0 pmos_3(vdd, net_129, net_0);
  tranif0 pmos_4(w, net_1, writeb);
  tranif0 pmos_5(net_1, net_86, net_0);
  tranif0 pmos_6(net_107, vdd, net_0);
  tranif0 pmos_7(r1, net_107, read1b);
endmodule   /* mips8__regram_dp */

module mips8__regram_8(read1, read1b, read2, read2b, w, w_1, w_2, w_3, w_4, 
      w_5, w_6, w_7, write, writeb, r1, r1_1, r1_2, r1_3, r1_4, r1_5, r1_6, 
      r1_7, r2, r2_1, r2_2, r2_3, r2_4, r2_5, r2_6, r2_7, vdd, vdd_1, vdd_1_1, 
      vdd_2, vdd_3, vdd_4, vdd_5, vdd_6, gnd, gnd_1, gnd_1_1, gnd_2, gnd_3, 
      gnd_4, gnd_5, gnd_6);
  input read1;
  input read1b;
  input read2;
  input read2b;
  input [0:0] w;
  input [1:1] w_1;
  input [2:2] w_2;
  input [3:3] w_3;
  input [4:4] w_4;
  input [5:5] w_5;
  input [6:6] w_6;
  input [7:7] w_7;
  input write;
  input writeb;
  output [0:0] r1;
  output [1:1] r1_1;
  output [2:2] r1_2;
  output [3:3] r1_3;
  output [4:4] r1_4;
  output [5:5] r1_5;
  output [6:6] r1_6;
  output [7:7] r1_7;
  output [0:0] r2;
  output [1:1] r2_1;
  output [2:2] r2_2;
  output [3:3] r2_3;
  output [4:4] r2_4;
  output [5:5] r2_5;
  output [6:6] r2_6;
  output [7:7] r2_7;
  input vdd;
  input vdd_1;
  input vdd_1_1;
  input vdd_2;
  input vdd_3;
  input vdd_4;
  input vdd_5;
  input vdd_6;
  input gnd;
  input gnd_1;
  input gnd_1_1;
  input gnd_2;
  input gnd_3;
  input gnd_4;
  input gnd_5;
  input gnd_6;

  supply1 vdd;
  supply0 gnd;
  mips8__regram_dp regram_d_0(.read1(read1), .read1b(read1b), .read2(read2), 
      .read2b(read2b), .w(w[0]), .write(write), .writeb(writeb), .r1(r1[0]), 
      .r2(r2[0]), .vdd(vdd_1), .gnd(gnd_1));
  mips8__regram_dp regram_d_1(.read1(read1), .read1b(read1b), .read2(read2), 
      .read2b(read2b), .w(w_1[1]), .write(write), .writeb(writeb), 
      .r1(r1_1[1]), .r2(r2_1[1]), .vdd(vdd_1_1), .gnd(gnd_1_1));
  mips8__regram_dp regram_d_2(.read1(read1), .read1b(read1b), .read2(read2), 
      .read2b(read2b), .w(w_2[2]), .write(write), .writeb(writeb), 
      .r1(r1_2[2]), .r2(r2_2[2]), .vdd(vdd_2), .gnd(gnd_2));
  mips8__regram_dp regram_d_3(.read1(read1), .read1b(read1b), .read2(read2), 
      .read2b(read2b), .w(w_3[3]), .write(write), .writeb(writeb), 
      .r1(r1_3[3]), .r2(r2_3[3]), .vdd(vdd_3), .gnd(gnd_3));
  mips8__regram_dp regram_d_4(.read1(read1), .read1b(read1b), .read2(read2), 
      .read2b(read2b), .w(w_4[4]), .write(write), .writeb(writeb), 
      .r1(r1_4[4]), .r2(r2_4[4]), .vdd(vdd_4), .gnd(gnd_4));
  mips8__regram_dp regram_d_5(.read1(read1), .read1b(read1b), .read2(read2), 
      .read2b(read2b), .w(w_5[5]), .write(write), .writeb(writeb), 
      .r1(r1_5[5]), .r2(r2_5[5]), .vdd(vdd_5), .gnd(gnd_5));
  mips8__regram_dp regram_d_6(.read1(read1), .read1b(read1b), .read2(read2), 
      .read2b(read2b), .w(w_6[6]), .write(write), .writeb(writeb), 
      .r1(r1_6[6]), .r2(r2_6[6]), .vdd(vdd_6), .gnd(gnd_6));
  mips8__regram_dp regram_d_7(.read1(read1), .read1b(read1b), .read2(read2), 
      .read2b(read2b), .w(w_7[7]), .write(write), .writeb(writeb), 
      .r1(r1_7[7]), .r2(r2_7[7]), .vdd(vdd), .gnd(gnd));
endmodule   /* mips8__regram_8 */

module mips8__regramvector_dp(RegWrite, ph2, r1a, r1a_1, r1a_2, r2a, r2a_1, 
      r2a_2, w, w_1, w_2, w_3, w_4, w_5, w_6, w_7, wa, wa_1, wa_2, r1, r1_1, 
      r1_2, r1_3, r1_4, r1_5, r1_6, r1_7, r2, r2_1, r2_2, r2_3, r2_4, r2_5, 
      r2_6, r2_7, vdd, vdd_1, vdd_10, vdd_1_1, vdd_2, vdd_3, vdd_4, vdd_5, 
      vdd_6, vdd_7, vdd_8, vdd_9, gnd, gnd_1, gnd_10, gnd_1_1, gnd_2, gnd_3, 
      gnd_4, gnd_5, gnd_6, gnd_7, gnd_8, gnd_9);
  input RegWrite;
  input ph2;
  input [0:0] r1a;
  input [1:1] r1a_1;
  input [2:2] r1a_2;
  input [0:0] r2a;
  input [1:1] r2a_1;
  input [2:2] r2a_2;
  input [0:0] w;
  input [1:1] w_1;
  input [2:2] w_2;
  input [3:3] w_3;
  input [4:4] w_4;
  input [5:5] w_5;
  input [6:6] w_6;
  input [7:7] w_7;
  input [0:0] wa;
  input [1:1] wa_1;
  input [2:2] wa_2;
  output [0:0] r1;
  output [1:1] r1_1;
  output [2:2] r1_2;
  output [3:3] r1_3;
  output [4:4] r1_4;
  output [5:5] r1_5;
  output [6:6] r1_6;
  output [7:7] r1_7;
  output [0:0] r2;
  output [1:1] r2_1;
  output [2:2] r2_2;
  output [3:3] r2_3;
  output [4:4] r2_4;
  output [5:5] r2_5;
  output [6:6] r2_6;
  output [7:7] r2_7;
  input vdd;
  input vdd_1;
  input vdd_10;
  input vdd_1_1;
  input vdd_2;
  input vdd_3;
  input vdd_4;
  input vdd_5;
  input vdd_6;
  input vdd_7;
  input vdd_8;
  input vdd_9;
  input gnd;
  input gnd_1;
  input gnd_10;
  input gnd_1_1;
  input gnd_2;
  input gnd_3;
  input gnd_4;
  input gnd_5;
  input gnd_6;
  input gnd_7;
  input gnd_8;
  input gnd_9;

  supply1 vdd;
  supply0 gnd;
  wire net_72, net_73, net_74, net_75, net_76, net_77;

  mips8__regram_zipper regfile__0(.RegWrite(RegWrite), .ph2(ph2), 
      .r1a(r1a[0:0]), .r1a_1(r1a_1[1:1]), .r1a_2(r1a_2[2:2]), .r2a(r2a[0:0]), 
      .r2a_1(r2a_1[1:1]), .r2a_2(r2a_2[2:2]), .wa(wa[0:0]), .wa_1(wa_1[1:1]), 
      .wa_2(wa_2[2:2]), .read1(net_74), .read1b(net_75), .read2(net_77), 
      .read2b(net_76), .write(net_73), .writeb(net_72), .vdd(vdd_3), 
      .vdd_1(vdd_1), .vdd_1_1(vdd_1_1), .vdd_2(vdd_2), .gnd(gnd_3), 
      .gnd_1(gnd), .gnd_1_1(gnd_1_1), .gnd_2(gnd_2));
  mips8__regram_8 regram_8_0(.read1(net_74), .read1b(net_75), .read2(net_77), 
      .read2b(net_76), .w(w[0:0]), .w_1(w_1[1:1]), .w_2(w_2[2:2]), 
      .w_3(w_3[3:3]), .w_4(w_4[4:4]), .w_5(w_5[5:5]), .w_6(w_6[6:6]), 
      .w_7(w_7[7:7]), .write(net_73), .writeb(net_72), .r1(r1[0:0]), 
      .r1_1(r1_1[1:1]), .r1_2(r1_2[2:2]), .r1_3(r1_3[3:3]), .r1_4(r1_4[4:4]), 
      .r1_5(r1_5[5:5]), .r1_6(r1_6[6:6]), .r1_7(r1_7[7:7]), .r2(r2[0:0]), 
      .r2_1(r2_1[1:1]), .r2_2(r2_2[2:2]), .r2_3(r2_3[3:3]), .r2_4(r2_4[4:4]), 
      .r2_5(r2_5[5:5]), .r2_6(r2_6[6:6]), .r2_7(r2_7[7:7]), .vdd(vdd), 
      .vdd_1(vdd_4), .vdd_1_1(vdd_5), .vdd_2(vdd_6), .vdd_3(vdd_7), 
      .vdd_4(vdd_8), .vdd_5(vdd_9), .vdd_6(vdd_10), .gnd(gnd_1), .gnd_1(gnd_4), 
      .gnd_1_1(gnd_5), .gnd_2(gnd_6), .gnd_3(gnd_7), .gnd_4(gnd_8), 
      .gnd_5(gnd_9), .gnd_6(gnd_10));
endmodule   /* mips8__regramvector_dp */

module mips8__regram0(read1, read2, r1, r2, gnd);
  input read1;
  input read2;
  output r1;
  output r2;
  input gnd;

  supply0 gnd;
  wire plnode_0_well;

  tranif1 nmos_0(gnd, r2, read2);
  tranif1 nmos_1(gnd, r1, read1);
endmodule   /* mips8__regram0 */

module mips8__regramvector0_dp(RegWrite, ph2, r1a, r1a_1, r1a_2, r2a, r2a_1, 
      r2a_2, wa, wa_1, wa_2, r1, r1_1, r1_2, r1_3, r1_4, r1_5, r1_6, r1_7, r2, 
      r2_1, r2_2, r2_3, r2_4, r2_5, r2_6, r2_7, vdd, vdd_1, vdd_1_1, vdd_2, 
      gnd, gnd_1, gnd_10, gnd_11, gnd_1_1, gnd_2, gnd_4, gnd_5, gnd_6, gnd_7, 
      gnd_8, gnd_9);
  input RegWrite;
  input ph2;
  input [0:0] r1a;
  input [1:1] r1a_1;
  input [2:2] r1a_2;
  input [0:0] r2a;
  input [1:1] r2a_1;
  input [2:2] r2a_2;
  input [0:0] wa;
  input [1:1] wa_1;
  input [2:2] wa_2;
  output [0:0] r1;
  output [1:1] r1_1;
  output [2:2] r1_2;
  output [3:3] r1_3;
  output [4:4] r1_4;
  output [5:5] r1_5;
  output [6:6] r1_6;
  output [7:7] r1_7;
  output [0:0] r2;
  output [1:1] r2_1;
  output [2:2] r2_2;
  output [3:3] r2_3;
  output [4:4] r2_4;
  output [5:5] r2_5;
  output [6:6] r2_6;
  output [7:7] r2_7;
  input vdd;
  input vdd_1;
  input vdd_1_1;
  input vdd_2;
  input gnd;
  input gnd_1;
  input gnd_10;
  input gnd_11;
  input gnd_1_1;
  input gnd_2;
  input gnd_4;
  input gnd_5;
  input gnd_6;
  input gnd_7;
  input gnd_8;
  input gnd_9;

  supply1 vdd;
  supply0 gnd;
  wire net_0, net_7, regfile__0_read1b, regfile__0_read2b, regfile__0_write;
  wire regfile__0_writeb;

  mips8__regram_zipper regfile__0(.RegWrite(RegWrite), .ph2(ph2), 
      .r1a(r1a[0:0]), .r1a_1(r1a_1[1:1]), .r1a_2(r1a_2[2:2]), .r2a(r2a[0:0]), 
      .r2a_1(r2a_1[1:1]), .r2a_2(r2a_2[2:2]), .wa(wa[0:0]), .wa_1(wa_1[1:1]), 
      .wa_2(wa_2[2:2]), .read1(net_7), .read1b(regfile__0_read1b), 
      .read2(net_0), .read2b(regfile__0_read2b), .write(regfile__0_write), 
      .writeb(regfile__0_writeb), .vdd(vdd_1), .vdd_1(vdd), .vdd_1_1(vdd_1_1), 
      .vdd_2(vdd_2), .gnd(gnd_1), .gnd_1(gnd), .gnd_1_1(gnd_1_1), 
      .gnd_2(gnd_2));
  mips8__regram0 regram_0_0(.read1(net_7), .read2(net_0), .r1(r1[0]), 
      .r2(r2[0]), .gnd(gnd_4));
  mips8__regram0 regram_0_8(.read1(net_7), .read2(net_0), .r1(r1_1[1]), 
      .r2(r2_1[1]), .gnd(gnd_5));
  mips8__regram0 regram_0_9(.read1(net_7), .read2(net_0), .r1(r1_2[2]), 
      .r2(r2_2[2]), .gnd(gnd_6));
  mips8__regram0 regram_0_10(.read1(net_7), .read2(net_0), .r1(r1_3[3]), 
      .r2(r2_3[3]), .gnd(gnd_7));
  mips8__regram0 regram_0_11(.read1(net_7), .read2(net_0), .r1(r1_4[4]), 
      .r2(r2_4[4]), .gnd(gnd_8));
  mips8__regram0 regram_0_12(.read1(net_7), .read2(net_0), .r1(r1_5[5]), 
      .r2(r2_5[5]), .gnd(gnd_9));
  mips8__regram0 regram_0_13(.read1(net_7), .read2(net_0), .r1(r1_6[6]), 
      .r2(r2_6[6]), .gnd(gnd_10));
  mips8__regram0 regram_0_14(.read1(net_7), .read2(net_0), .r1(r1_7[7]), 
      .r2(r2_7[7]), .gnd(gnd_11));
endmodule   /* mips8__regramvector0_dp */

module mips8__regramarray_dp(ph2, ra1, ra1_1, ra1_2, ra2, ra2_1, ra2_2, 
      regwrite, w, w_1, w_2, w_3, w_4, w_5, w_6, w_7, wa, wa_1, wa_2, r1, r1_1, 
      r1_2, r1_3, r1_4, r1_5, r1_6, r1_7, r2, r2_1, r2_2, r2_3, r2_4, r2_5, 
      r2_6, r2_7, vdd, vdd_1, vdd_10, vdd_11, vdd_12, vdd_3, vdd_4, vdd_5, 
      vdd_6, vdd_7, vdd_8, vdd_9, gnd, gnd_1, gnd_10, gnd_1_1, gnd_2, gnd_3, 
      gnd_4, gnd_5, gnd_6, gnd_7, gnd_8, gnd_9);
  input ph2;
  input [0:0] ra1;
  input [1:1] ra1_1;
  input [2:2] ra1_2;
  input [0:0] ra2;
  input [1:1] ra2_1;
  input [2:2] ra2_2;
  input regwrite;
  input [0:0] w;
  input [1:1] w_1;
  input [2:2] w_2;
  input [3:3] w_3;
  input [4:4] w_4;
  input [5:5] w_5;
  input [6:6] w_6;
  input [7:7] w_7;
  input [0:0] wa;
  input [1:1] wa_1;
  input [2:2] wa_2;
  output [0:0] r1;
  output [1:1] r1_1;
  output [2:2] r1_2;
  output [3:3] r1_3;
  output [4:4] r1_4;
  output [5:5] r1_5;
  output [6:6] r1_6;
  output [7:7] r1_7;
  output [0:0] r2;
  output [1:1] r2_1;
  output [2:2] r2_2;
  output [3:3] r2_3;
  output [4:4] r2_4;
  output [5:5] r2_5;
  output [6:6] r2_6;
  output [7:7] r2_7;
  input vdd;
  input vdd_1;
  input vdd_10;
  input vdd_11;
  input vdd_12;
  input vdd_3;
  input vdd_4;
  input vdd_5;
  input vdd_6;
  input vdd_7;
  input vdd_8;
  input vdd_9;
  input gnd;
  input gnd_1;
  input gnd_10;
  input gnd_1_1;
  input gnd_2;
  input gnd_3;
  input gnd_4;
  input gnd_5;
  input gnd_6;
  input gnd_7;
  input gnd_8;
  input gnd_9;

  supply1 vdd;
  supply0 gnd;
  wire net_323, net_324, net_325, net_327, net_328, net_330, net_331, net_359;
  wire net_360, net_361, net_363, net_364, net_373, net_374, net_375, net_376;
  wire net_377, net_378, plnode_2_select, plnode_3_select, plnode_4_select;
  wire plnode_5_select, plnode_6_select, plnode_7_select;

  muddlib07__invbuf_4x invbuf_4_0(.s(ra1[0]), .s_out(net_323), 
      .sb_out(net_325), .vdd(vdd), .gnd(gnd_1));
  muddlib07__invbuf_4x invbuf_4_1(.s(ra2[0]), .s_out(net_324), 
      .sb_out(net_364), .vdd(vdd_12), .gnd(gnd_10));
  muddlib07__invbuf_4x invbuf_4_2(.s(wa[0]), .s_out(net_378), .sb_out(net_377), 
      .vdd(vdd_1), .gnd(gnd_9));
  muddlib07__invbuf_4x invbuf_4_3(.s(ra1_1[1]), .s_out(net_327), 
      .sb_out(net_328), .vdd(vdd), .gnd(gnd_1));
  muddlib07__invbuf_4x invbuf_4_4(.s(ra2_1[1]), .s_out(net_363), 
      .sb_out(net_361), .vdd(vdd_12), .gnd(gnd_10));
  muddlib07__invbuf_4x invbuf_4_5(.s(wa_1[1]), .s_out(net_376), 
      .sb_out(net_375), .vdd(vdd_1), .gnd(gnd_9));
  muddlib07__invbuf_4x invbuf_4_6(.s(ra1_2[2]), .s_out(net_330), 
      .sb_out(net_331), .vdd(vdd), .gnd(gnd_1));
  muddlib07__invbuf_4x invbuf_4_7(.s(ra2_2[2]), .s_out(net_359), 
      .sb_out(net_360), .vdd(vdd_12), .gnd(gnd_10));
  muddlib07__invbuf_4x invbuf_4_8(.s(wa_2[2]), .s_out(net_373), 
      .sb_out(net_374), .vdd(vdd_1), .gnd(gnd_9));
  mips8__regramvector_dp regramve_1(.RegWrite(regwrite), .ph2(ph2), 
      .r1a({net_323}), .r1a_1({net_328}), .r1a_2({net_331}), .r2a({net_324}), 
      .r2a_1({net_361}), .r2a_2({net_360}), .w(w[0:0]), .w_1(w_1[1:1]), 
      .w_2(w_2[2:2]), .w_3(w_3[3:3]), .w_4(w_4[4:4]), .w_5(w_5[5:5]), 
      .w_6(w_6[6:6]), .w_7(w_7[7:7]), .wa({net_378}), .wa_1({net_375}), 
      .wa_2({net_374}), .r1(r1[0:0]), .r1_1(r1_1[1:1]), .r1_2(r1_2[2:2]), 
      .r1_3(r1_3[3:3]), .r1_4(r1_4[4:4]), .r1_5(r1_5[5:5]), .r1_6(r1_6[6:6]), 
      .r1_7(r1_7[7:7]), .r2(r2[0:0]), .r2_1(r2_1[1:1]), .r2_2(r2_2[2:2]), 
      .r2_3(r2_3[3:3]), .r2_4(r2_4[4:4]), .r2_5(r2_5[5:5]), .r2_6(r2_6[6:6]), 
      .r2_7(r2_7[7:7]), .vdd(vdd_4), .vdd_1(vdd_12), .vdd_10(vdd_5), 
      .vdd_1_1(vdd_1), .vdd_2(vdd), .vdd_3(vdd_3), .vdd_4(vdd_11), 
      .vdd_5(vdd_10), .vdd_6(vdd_9), .vdd_7(vdd_8), .vdd_8(vdd_7), 
      .vdd_9(vdd_6), .gnd(gnd_10), .gnd_1(gnd_7), .gnd_10(gnd_6), 
      .gnd_1_1(gnd_9), .gnd_2(gnd_1), .gnd_3(gnd_8), .gnd_4(gnd), 
      .gnd_5(gnd_1_1), .gnd_6(gnd_2), .gnd_7(gnd_3), .gnd_8(gnd_4), 
      .gnd_9(gnd_5));
  mips8__regramvector_dp regramve_2(.RegWrite(regwrite), .ph2(ph2), 
      .r1a({net_325}), .r1a_1({net_327}), .r1a_2({net_331}), .r2a({net_364}), 
      .r2a_1({net_363}), .r2a_2({net_360}), .w(w[0:0]), .w_1(w_1[1:1]), 
      .w_2(w_2[2:2]), .w_3(w_3[3:3]), .w_4(w_4[4:4]), .w_5(w_5[5:5]), 
      .w_6(w_6[6:6]), .w_7(w_7[7:7]), .wa({net_377}), .wa_1({net_376}), 
      .wa_2({net_374}), .r1(r1[0:0]), .r1_1(r1_1[1:1]), .r1_2(r1_2[2:2]), 
      .r1_3(r1_3[3:3]), .r1_4(r1_4[4:4]), .r1_5(r1_5[5:5]), .r1_6(r1_6[6:6]), 
      .r1_7(r1_7[7:7]), .r2(r2[0:0]), .r2_1(r2_1[1:1]), .r2_2(r2_2[2:2]), 
      .r2_3(r2_3[3:3]), .r2_4(r2_4[4:4]), .r2_5(r2_5[5:5]), .r2_6(r2_6[6:6]), 
      .r2_7(r2_7[7:7]), .vdd(vdd_4), .vdd_1(vdd_12), .vdd_10(vdd_5), 
      .vdd_1_1(vdd_1), .vdd_2(vdd), .vdd_3(vdd_3), .vdd_4(vdd_11), 
      .vdd_5(vdd_10), .vdd_6(vdd_9), .vdd_7(vdd_8), .vdd_8(vdd_7), 
      .vdd_9(vdd_6), .gnd(gnd_10), .gnd_1(gnd_7), .gnd_10(gnd_6), 
      .gnd_1_1(gnd_9), .gnd_2(gnd_1), .gnd_3(gnd_8), .gnd_4(gnd), 
      .gnd_5(gnd_1_1), .gnd_6(gnd_2), .gnd_7(gnd_3), .gnd_8(gnd_4), 
      .gnd_9(gnd_5));
  mips8__regramvector_dp regramve_3(.RegWrite(regwrite), .ph2(ph2), 
      .r1a({net_323}), .r1a_1({net_327}), .r1a_2({net_331}), .r2a({net_324}), 
      .r2a_1({net_363}), .r2a_2({net_360}), .w(w[0:0]), .w_1(w_1[1:1]), 
      .w_2(w_2[2:2]), .w_3(w_3[3:3]), .w_4(w_4[4:4]), .w_5(w_5[5:5]), 
      .w_6(w_6[6:6]), .w_7(w_7[7:7]), .wa({net_378}), .wa_1({net_376}), 
      .wa_2({net_374}), .r1(r1[0:0]), .r1_1(r1_1[1:1]), .r1_2(r1_2[2:2]), 
      .r1_3(r1_3[3:3]), .r1_4(r1_4[4:4]), .r1_5(r1_5[5:5]), .r1_6(r1_6[6:6]), 
      .r1_7(r1_7[7:7]), .r2(r2[0:0]), .r2_1(r2_1[1:1]), .r2_2(r2_2[2:2]), 
      .r2_3(r2_3[3:3]), .r2_4(r2_4[4:4]), .r2_5(r2_5[5:5]), .r2_6(r2_6[6:6]), 
      .r2_7(r2_7[7:7]), .vdd(vdd_4), .vdd_1(vdd_12), .vdd_10(vdd_5), 
      .vdd_1_1(vdd_1), .vdd_2(vdd), .vdd_3(vdd_3), .vdd_4(vdd_11), 
      .vdd_5(vdd_10), .vdd_6(vdd_9), .vdd_7(vdd_8), .vdd_8(vdd_7), 
      .vdd_9(vdd_6), .gnd(gnd_10), .gnd_1(gnd_7), .gnd_10(gnd_6), 
      .gnd_1_1(gnd_9), .gnd_2(gnd_1), .gnd_3(gnd_8), .gnd_4(gnd), 
      .gnd_5(gnd_1_1), .gnd_6(gnd_2), .gnd_7(gnd_3), .gnd_8(gnd_4), 
      .gnd_9(gnd_5));
  mips8__regramvector_dp regramve_4(.RegWrite(regwrite), .ph2(ph2), 
      .r1a({net_325}), .r1a_1({net_328}), .r1a_2({net_330}), .r2a({net_364}), 
      .r2a_1({net_361}), .r2a_2({net_359}), .w(w[0:0]), .w_1(w_1[1:1]), 
      .w_2(w_2[2:2]), .w_3(w_3[3:3]), .w_4(w_4[4:4]), .w_5(w_5[5:5]), 
      .w_6(w_6[6:6]), .w_7(w_7[7:7]), .wa({net_377}), .wa_1({net_375}), 
      .wa_2({net_373}), .r1(r1[0:0]), .r1_1(r1_1[1:1]), .r1_2(r1_2[2:2]), 
      .r1_3(r1_3[3:3]), .r1_4(r1_4[4:4]), .r1_5(r1_5[5:5]), .r1_6(r1_6[6:6]), 
      .r1_7(r1_7[7:7]), .r2(r2[0:0]), .r2_1(r2_1[1:1]), .r2_2(r2_2[2:2]), 
      .r2_3(r2_3[3:3]), .r2_4(r2_4[4:4]), .r2_5(r2_5[5:5]), .r2_6(r2_6[6:6]), 
      .r2_7(r2_7[7:7]), .vdd(vdd_4), .vdd_1(vdd_12), .vdd_10(vdd_5), 
      .vdd_1_1(vdd_1), .vdd_2(vdd), .vdd_3(vdd_3), .vdd_4(vdd_11), 
      .vdd_5(vdd_10), .vdd_6(vdd_9), .vdd_7(vdd_8), .vdd_8(vdd_7), 
      .vdd_9(vdd_6), .gnd(gnd_10), .gnd_1(gnd_7), .gnd_10(gnd_6), 
      .gnd_1_1(gnd_9), .gnd_2(gnd_1), .gnd_3(gnd_8), .gnd_4(gnd), 
      .gnd_5(gnd_1_1), .gnd_6(gnd_2), .gnd_7(gnd_3), .gnd_8(gnd_4), 
      .gnd_9(gnd_5));
  mips8__regramvector_dp regramve_5(.RegWrite(regwrite), .ph2(ph2), 
      .r1a({net_323}), .r1a_1({net_328}), .r1a_2({net_330}), .r2a({net_324}), 
      .r2a_1({net_361}), .r2a_2({net_359}), .w(w[0:0]), .w_1(w_1[1:1]), 
      .w_2(w_2[2:2]), .w_3(w_3[3:3]), .w_4(w_4[4:4]), .w_5(w_5[5:5]), 
      .w_6(w_6[6:6]), .w_7(w_7[7:7]), .wa({net_378}), .wa_1({net_375}), 
      .wa_2({net_373}), .r1(r1[0:0]), .r1_1(r1_1[1:1]), .r1_2(r1_2[2:2]), 
      .r1_3(r1_3[3:3]), .r1_4(r1_4[4:4]), .r1_5(r1_5[5:5]), .r1_6(r1_6[6:6]), 
      .r1_7(r1_7[7:7]), .r2(r2[0:0]), .r2_1(r2_1[1:1]), .r2_2(r2_2[2:2]), 
      .r2_3(r2_3[3:3]), .r2_4(r2_4[4:4]), .r2_5(r2_5[5:5]), .r2_6(r2_6[6:6]), 
      .r2_7(r2_7[7:7]), .vdd(vdd_4), .vdd_1(vdd_12), .vdd_10(vdd_5), 
      .vdd_1_1(vdd_1), .vdd_2(vdd), .vdd_3(vdd_3), .vdd_4(vdd_11), 
      .vdd_5(vdd_10), .vdd_6(vdd_9), .vdd_7(vdd_8), .vdd_8(vdd_7), 
      .vdd_9(vdd_6), .gnd(gnd_10), .gnd_1(gnd_7), .gnd_10(gnd_6), 
      .gnd_1_1(gnd_9), .gnd_2(gnd_1), .gnd_3(gnd_8), .gnd_4(gnd), 
      .gnd_5(gnd_1_1), .gnd_6(gnd_2), .gnd_7(gnd_3), .gnd_8(gnd_4), 
      .gnd_9(gnd_5));
  mips8__regramvector_dp regramve_6(.RegWrite(regwrite), .ph2(ph2), 
      .r1a({net_325}), .r1a_1({net_327}), .r1a_2({net_330}), .r2a({net_364}), 
      .r2a_1({net_363}), .r2a_2({net_359}), .w(w[0:0]), .w_1(w_1[1:1]), 
      .w_2(w_2[2:2]), .w_3(w_3[3:3]), .w_4(w_4[4:4]), .w_5(w_5[5:5]), 
      .w_6(w_6[6:6]), .w_7(w_7[7:7]), .wa({net_377}), .wa_1({net_376}), 
      .wa_2({net_373}), .r1(r1[0:0]), .r1_1(r1_1[1:1]), .r1_2(r1_2[2:2]), 
      .r1_3(r1_3[3:3]), .r1_4(r1_4[4:4]), .r1_5(r1_5[5:5]), .r1_6(r1_6[6:6]), 
      .r1_7(r1_7[7:7]), .r2(r2[0:0]), .r2_1(r2_1[1:1]), .r2_2(r2_2[2:2]), 
      .r2_3(r2_3[3:3]), .r2_4(r2_4[4:4]), .r2_5(r2_5[5:5]), .r2_6(r2_6[6:6]), 
      .r2_7(r2_7[7:7]), .vdd(vdd_4), .vdd_1(vdd_12), .vdd_10(vdd_5), 
      .vdd_1_1(vdd_1), .vdd_2(vdd), .vdd_3(vdd_3), .vdd_4(vdd_11), 
      .vdd_5(vdd_10), .vdd_6(vdd_9), .vdd_7(vdd_8), .vdd_8(vdd_7), 
      .vdd_9(vdd_6), .gnd(gnd_10), .gnd_1(gnd_7), .gnd_10(gnd_6), 
      .gnd_1_1(gnd_9), .gnd_2(gnd_1), .gnd_3(gnd_8), .gnd_4(gnd), 
      .gnd_5(gnd_1_1), .gnd_6(gnd_2), .gnd_7(gnd_3), .gnd_8(gnd_4), 
      .gnd_9(gnd_5));
  mips8__regramvector_dp regramve_7(.RegWrite(regwrite), .ph2(ph2), 
      .r1a({net_323}), .r1a_1({net_327}), .r1a_2({net_330}), .r2a({net_324}), 
      .r2a_1({net_363}), .r2a_2({net_359}), .w(w[0:0]), .w_1(w_1[1:1]), 
      .w_2(w_2[2:2]), .w_3(w_3[3:3]), .w_4(w_4[4:4]), .w_5(w_5[5:5]), 
      .w_6(w_6[6:6]), .w_7(w_7[7:7]), .wa({net_378}), .wa_1({net_376}), 
      .wa_2({net_373}), .r1(r1[0:0]), .r1_1(r1_1[1:1]), .r1_2(r1_2[2:2]), 
      .r1_3(r1_3[3:3]), .r1_4(r1_4[4:4]), .r1_5(r1_5[5:5]), .r1_6(r1_6[6:6]), 
      .r1_7(r1_7[7:7]), .r2(r2[0:0]), .r2_1(r2_1[1:1]), .r2_2(r2_2[2:2]), 
      .r2_3(r2_3[3:3]), .r2_4(r2_4[4:4]), .r2_5(r2_5[5:5]), .r2_6(r2_6[6:6]), 
      .r2_7(r2_7[7:7]), .vdd(vdd_4), .vdd_1(vdd_12), .vdd_10(vdd_5), 
      .vdd_1_1(vdd_1), .vdd_2(vdd), .vdd_3(vdd_3), .vdd_4(vdd_11), 
      .vdd_5(vdd_10), .vdd_6(vdd_9), .vdd_7(vdd_8), .vdd_8(vdd_7), 
      .vdd_9(vdd_6), .gnd(gnd_10), .gnd_1(gnd_7), .gnd_10(gnd_6), 
      .gnd_1_1(gnd_9), .gnd_2(gnd_1), .gnd_3(gnd_8), .gnd_4(gnd), 
      .gnd_5(gnd_1_1), .gnd_6(gnd_2), .gnd_7(gnd_3), .gnd_8(gnd_4), 
      .gnd_9(gnd_5));
  mips8__regramvector0_dp regramve_8(.RegWrite(regwrite), .ph2(ph2), 
      .r1a({net_325}), .r1a_1({net_328}), .r1a_2({net_331}), .r2a({net_364}), 
      .r2a_1({net_361}), .r2a_2({net_360}), .wa({net_377}), .wa_1({net_375}), 
      .wa_2({net_374}), .r1(r1[0:0]), .r1_1(r1_1[1:1]), .r1_2(r1_2[2:2]), 
      .r1_3(r1_3[3:3]), .r1_4(r1_4[4:4]), .r1_5(r1_5[5:5]), .r1_6(r1_6[6:6]), 
      .r1_7(r1_7[7:7]), .r2(r2[0:0]), .r2_1(r2_1[1:1]), .r2_2(r2_2[2:2]), 
      .r2_3(r2_3[3:3]), .r2_4(r2_4[4:4]), .r2_5(r2_5[5:5]), .r2_6(r2_6[6:6]), 
      .r2_7(r2_7[7:7]), .vdd(vdd_12), .vdd_1(vdd_3), .vdd_1_1(vdd_1), 
      .vdd_2(vdd), .gnd(gnd_10), .gnd_1(gnd_8), .gnd_10(gnd_6), .gnd_11(gnd_7), 
      .gnd_1_1(gnd_9), .gnd_2(gnd_1), .gnd_4(gnd), .gnd_5(gnd_1_1), 
      .gnd_6(gnd_2), .gnd_7(gnd_3), .gnd_8(gnd_4), .gnd_9(gnd_5));
endmodule   /* mips8__regramarray_dp */

module datapath(alucontrol, alucontrol_1, alucontrol_2, alusrca, alusrcb, 
      alusrcb_1, iord, irwrite, irwrite_1, irwrite_2, irwrite_3, memdata, 
      memdata_1, memdata_2, memdata_3, memdata_4, memdata_5, memdata_6, 
      memdata_7, memtoreg, pcen, pcsrc, pcsrc_1, ph1, ph2, regdst, regwrite, 
      reset, adr, adr_1, adr_2, adr_3, adr_4, adr_5, adr_6, adr_7, funct, 
      funct_1, funct_2, funct_3, funct_4, funct_5, op, op_1, op_2, op_3, op_4, 
      op_5, writedata, writedata_1, writedata_2, writedata_3, writedata_4, 
      writedata_5, writedata_6, writedata_7, zero, vdd, gnd);
  input [0:0] alucontrol;
  input [1:1] alucontrol_1;
  input [2:2] alucontrol_2;
  input alusrca;
  input [0:0] alusrcb;
  input [1:1] alusrcb_1;
  input iord;
  input [0:0] irwrite;
  input [1:1] irwrite_1;
  input [2:2] irwrite_2;
  input [3:3] irwrite_3;
  input [0:0] memdata;
  input [1:1] memdata_1;
  input [2:2] memdata_2;
  input [3:3] memdata_3;
  input [4:4] memdata_4;
  input [5:5] memdata_5;
  input [6:6] memdata_6;
  input [7:7] memdata_7;
  input memtoreg;
  input pcen;
  input [0:0] pcsrc;
  input [1:1] pcsrc_1;
  input ph1;
  input ph2;
  input regdst;
  input regwrite;
  input reset;
  output [0:0] adr;
  output [1:1] adr_1;
  output [2:2] adr_2;
  output [3:3] adr_3;
  output [4:4] adr_4;
  output [5:5] adr_5;
  output [6:6] adr_6;
  output [7:7] adr_7;
  output [0:0] funct;
  output [1:1] funct_1;
  output [2:2] funct_2;
  output [3:3] funct_3;
  output [4:4] funct_4;
  output [5:5] funct_5;
  output [0:0] op;
  output [1:1] op_1;
  output [2:2] op_2;
  output [3:3] op_3;
  output [4:4] op_4;
  output [5:5] op_5;
  output [0:0] writedata;
  output [1:1] writedata_1;
  output [2:2] writedata_2;
  output [3:3] writedata_3;
  output [4:4] writedata_4;
  output [5:5] writedata_5;
  output [6:6] writedata_6;
  output [7:7] writedata_7;
  output zero;
  input vdd;
  input gnd;

  supply1 vdd;
  supply0 gnd;
  wire net_593, net_594, plnode_10_select, plnode_11_select, plnode_12_select;
  wire plnode_13_select, plnode_14_select, plnode_15_select, plnode_16_select;
  wire plnode_17_select, plnode_18_select, plnode_19_select, plnode_1_select;
  wire plnode_20_select, plnode_21_select, plnode_22_select, plnode_23_select;
  wire plnode_24_select, plnode_25_select, plnode_26_select, plnode_27_select;
  wire plnode_28_select, plnode_29_select, plnode_2_select, plnode_30_select;
  wire plnode_31_select, plnode_32_select, plnode_33_select, plnode_34_select;
  wire plnode_35_select, plnode_36_select, plnode_37_select, plnode_38_select;
  wire plnode_39_select, plnode_3_select, plnode_40_select, plnode_41_select;
  wire plnode_42_select, plnode_43_select, plnode_44_select, plnode_45_select;
  wire plnode_46_select, plnode_47_select, plnode_48_select, plnode_49_select;
  wire plnode_4_select, plnode_50_select, plnode_51_select, plnode_52_select;
  wire plnode_53_select, plnode_54_select, plnode_55_select, plnode_5_select;
  wire plnode_6_select, plnode_7_select, plnode_8_select, plnode_9_select;
  wire [7:0] a;
  wire [7:0] aluout;
  wire [7:0] aluresult;
  wire [7:0] data;
  wire [13:11] instr;
  wire [18:16] instr_1;
  wire [23:21] instr_2;
  wire [2:0] ir1_q;
  wire [7:6] ir1_q_1;
  wire [4:3] ir2_q;
  wire [1:0] ir3_q;
  wire [7:0] nextpc;
  wire [7:0] pc;
  wire [7:0] rd1;
  wire [7:0] rd2;
  wire [7:0] srca;
  wire [7:0] srcb;
  wire [2:0] wa;
  wire [7:0] wd;

  wordlib8__mux2_1x_8 adrmux(.d0(pc[0:0]), .d0_1(pc[1:1]), .d0_2(pc[2:2]), 
      .d0_3(pc[3:3]), .d0_4(pc[4:4]), .d0_5(pc[5:5]), .d0_6(pc[6:6]), 
      .d0_7(pc[7:7]), .d1(aluout[0:0]), .d1_1(aluout[1:1]), .d1_2(aluout[2:2]), 
      .d1_3(aluout[3:3]), .d1_4(aluout[4:4]), .d1_5(aluout[5:5]), 
      .d1_6(aluout[6:6]), .d1_7(aluout[7:7]), .s(iord), .y(adr[0:0]), 
      .y_1(adr_1[1:1]), .y_2(adr_2[2:2]), .y_3(adr_3[3:3]), .y_4(adr_4[4:4]), 
      .y_5(adr_5[5:5]), .y_6(adr_6[6:6]), .y_7(adr_7[7:7]), .vdd(vdd), 
      .vdd_1(vdd), .vdd_32(vdd), .vdd_33(vdd), .vdd_34(vdd), .vdd_35(vdd), 
      .vdd_36(vdd), .vdd_37(vdd), .vdd_38(vdd), .gnd(gnd), .gnd_1(gnd), 
      .gnd_32(gnd), .gnd_33(gnd), .gnd_34(gnd), .gnd_35(gnd), .gnd_36(gnd), 
      .gnd_37(gnd), .gnd_38(gnd));
  mips8__alu alu_0(.a(srca[0:0]), .a_1(srca[1:1]), .a_2(srca[2:2]), 
      .a_3(srca[3:3]), .a_4(srca[4:4]), .a_5(srca[5:5]), .a_6(srca[6:6]), 
      .a_7(srca[7:7]), .alucontrol(alucontrol[0:0]), 
      .alucontrol_1(alucontrol_1[1:1]), .alucontrol_2(alucontrol_2[2:2]), 
      .b(srcb[0:0]), .b_1(srcb[1:1]), .b_2(srcb[2:2]), .b_3(srcb[3:3]), 
      .b_4(srcb[4:4]), .b_5(srcb[5:5]), .b_6(srcb[6:6]), .b_7(srcb[7:7]), 
      .result(aluresult[0:0]), .result_1(aluresult[1:1]), 
      .result_2(aluresult[2:2]), .result_3(aluresult[3:3]), 
      .result_4(aluresult[4:4]), .result_5(aluresult[5:5]), 
      .result_6(aluresult[6:6]), .result_7(aluresult[7:7]), .zero(zero), 
      .vdd(vdd), .vdd_1(vdd), .vdd_2(vdd), .vdd_3(vdd), .vdd_32(vdd), 
      .vdd_4(vdd), .vdd_40(vdd), .vdd_5(vdd), .vdd_6(vdd), .vdd_7(vdd), 
      .gnd(gnd), .gnd_1(gnd), .gnd_2(gnd), .gnd_3(gnd), .gnd_32(gnd), 
      .gnd_4(gnd), .gnd_40(gnd), .gnd_5(gnd), .gnd_6(gnd), .gnd_7(gnd));
  wordlib8__flop_1x_8 areg(.d(rd1[0:0]), .d_1(rd1[1:1]), .d_2(rd1[2:2]), 
      .d_3(rd1[3:3]), .d_4(rd1[4:4]), .d_5(rd1[5:5]), .d_6(rd1[6:6]), 
      .d_7(rd1[7:7]), .ph1(ph1), .ph2(ph2), .q(a[0:0]), .q_1(a[1:1]), 
      .q_2(a[2:2]), .q_3(a[3:3]), .q_4(a[4:4]), .q_5(a[5:5]), .q_6(a[6:6]), 
      .q_7(a[7:7]), .vdd(vdd), .vdd_1(vdd), .vdd_10(vdd), .vdd_11(vdd), 
      .vdd_12(vdd), .vdd_13(vdd), .vdd_14(vdd), .vdd_15(vdd), .vdd_9(vdd), 
      .gnd(gnd), .gnd_1(gnd), .gnd_10(gnd), .gnd_11(gnd), .gnd_12(gnd), 
      .gnd_13(gnd), .gnd_14(gnd), .gnd_15(gnd), .gnd_9(gnd));
  wordlib8__flop_1x_8 datareg(.d(memdata[0:0]), .d_1(memdata_1[1:1]), 
      .d_2(memdata_2[2:2]), .d_3(memdata_3[3:3]), .d_4(memdata_4[4:4]), 
      .d_5(memdata_5[5:5]), .d_6(memdata_6[6:6]), .d_7(memdata_7[7:7]), 
      .ph1(ph1), .ph2(ph2), .q(data[0:0]), .q_1(data[1:1]), .q_2(data[2:2]), 
      .q_3(data[3:3]), .q_4(data[4:4]), .q_5(data[5:5]), .q_6(data[6:6]), 
      .q_7(data[7:7]), .vdd(vdd), .vdd_1(vdd), .vdd_10(vdd), .vdd_11(vdd), 
      .vdd_12(vdd), .vdd_13(vdd), .vdd_14(vdd), .vdd_15(vdd), .vdd_9(vdd), 
      .gnd(gnd), .gnd_1(gnd), .gnd_10(gnd), .gnd_11(gnd), .gnd_12(gnd), 
      .gnd_13(gnd), .gnd_14(gnd), .gnd_15(gnd), .gnd_9(gnd));
  wordlib8__flopenr_1x_8 flopenr__0(.d(nextpc[0:0]), .d_1(nextpc[1:1]), 
      .d_2(nextpc[2:2]), .d_3(nextpc[3:3]), .d_4(nextpc[4:4]), 
      .d_5(nextpc[5:5]), .d_6(nextpc[6:6]), .d_7(nextpc[7:7]), .en(pcen), 
      .ph1(ph1), .ph2(ph2), .reset(reset), .q(pc[0:0]), .q_1(pc[1:1]), 
      .q_2(pc[2:2]), .q_3(pc[3:3]), .q_4(pc[4:4]), .q_5(pc[5:5]), 
      .q_6(pc[6:6]), .q_7(pc[7:7]), .vdd(vdd), .vdd_1(vdd), .vdd_32(vdd), 
      .vdd_33(vdd), .vdd_34(vdd), .vdd_35(vdd), .vdd_36(vdd), .vdd_37(vdd), 
      .vdd_38(vdd), .gnd(gnd), .gnd_1(gnd), .gnd_32(gnd), .gnd_33(gnd), 
      .gnd_34(gnd), .gnd_35(gnd), .gnd_36(gnd), .gnd_37(gnd), .gnd_38(gnd));
  wordlib8__flopen_1x_8 ir0(.d(memdata[0:0]), .d_1(memdata_1[1:1]), 
      .d_2(memdata_2[2:2]), .d_3(memdata_3[3:3]), .d_4(memdata_4[4:4]), 
      .d_5(memdata_5[5:5]), .d_6(memdata_6[6:6]), .d_7(memdata_7[7:7]), 
      .en(irwrite[0]), .ph1(ph1), .ph2(ph2), .q(funct[0:0]), 
      .q_1(funct_1[1:1]), .q_2(funct_2[2:2]), .q_3(funct_3[3:3]), 
      .q_4(funct_4[4:4]), .q_5(funct_5[5:5]), .q_6({net_593}), .q_7({net_594}), 
      .vdd(vdd), .vdd_1(vdd), .vdd_1_1(vdd), .vdd_2(vdd), .vdd_3(vdd), 
      .vdd_4(vdd), .vdd_5(vdd), .vdd_6(vdd), .vdd_7(vdd), .gnd(gnd), 
      .gnd_1(gnd), .gnd_1_1(gnd), .gnd_2(gnd), .gnd_3(gnd), .gnd_4(gnd), 
      .gnd_5(gnd), .gnd_6(gnd), .gnd_7(gnd));
  wordlib8__flopen_1x_8 ir1(.d(memdata[0:0]), .d_1(memdata_1[1:1]), 
      .d_2(memdata_2[2:2]), .d_3(memdata_3[3:3]), .d_4(memdata_4[4:4]), 
      .d_5(memdata_5[5:5]), .d_6(memdata_6[6:6]), .d_7(memdata_7[7:7]), 
      .en(irwrite_1[1]), .ph1(ph1), .ph2(ph2), .q(ir1_q[0:0]), 
      .q_1(ir1_q[1:1]), .q_2(ir1_q[2:2]), .q_3(instr[11:11]), 
      .q_4(instr[12:12]), .q_5(instr[13:13]), .q_6(ir1_q_1[6:6]), 
      .q_7(ir1_q_1[7:7]), .vdd(vdd), .vdd_1(vdd), .vdd_1_1(vdd), .vdd_2(vdd), 
      .vdd_3(vdd), .vdd_4(vdd), .vdd_5(vdd), .vdd_6(vdd), .vdd_7(vdd), 
      .gnd(gnd), .gnd_1(gnd), .gnd_1_1(gnd), .gnd_2(gnd), .gnd_3(gnd), 
      .gnd_4(gnd), .gnd_5(gnd), .gnd_6(gnd), .gnd_7(gnd));
  wordlib8__flopen_1x_8 ir2(.d(memdata[0:0]), .d_1(memdata_1[1:1]), 
      .d_2(memdata_2[2:2]), .d_3(memdata_3[3:3]), .d_4(memdata_4[4:4]), 
      .d_5(memdata_5[5:5]), .d_6(memdata_6[6:6]), .d_7(memdata_7[7:7]), 
      .en(irwrite_2[2]), .ph1(ph1), .ph2(ph2), .q(instr_1[16:16]), 
      .q_1(instr_1[17:17]), .q_2(instr_1[18:18]), .q_3(ir2_q[3:3]), 
      .q_4(ir2_q[4:4]), .q_5(instr_2[21:21]), .q_6(instr_2[22:22]), 
      .q_7(instr_2[23:23]), .vdd(vdd), .vdd_1(vdd), .vdd_1_1(vdd), .vdd_2(vdd), 
      .vdd_3(vdd), .vdd_4(vdd), .vdd_5(vdd), .vdd_6(vdd), .vdd_7(vdd), 
      .gnd(gnd), .gnd_1(gnd), .gnd_1_1(gnd), .gnd_2(gnd), .gnd_3(gnd), 
      .gnd_4(gnd), .gnd_5(gnd), .gnd_6(gnd), .gnd_7(gnd));
  wordlib8__flopen_1x_8 ir3(.d(memdata[0:0]), .d_1(memdata_1[1:1]), 
      .d_2(memdata_2[2:2]), .d_3(memdata_3[3:3]), .d_4(memdata_4[4:4]), 
      .d_5(memdata_5[5:5]), .d_6(memdata_6[6:6]), .d_7(memdata_7[7:7]), 
      .en(irwrite_3[3]), .ph1(ph1), .ph2(ph2), .q(ir3_q[0:0]), 
      .q_1(ir3_q[1:1]), .q_2(op[0:0]), .q_3(op_1[1:1]), .q_4(op_2[2:2]), 
      .q_5(op_3[3:3]), .q_6(op_4[4:4]), .q_7(op_5[5:5]), .vdd(vdd), 
      .vdd_1(vdd), .vdd_1_1(vdd), .vdd_2(vdd), .vdd_3(vdd), .vdd_4(vdd), 
      .vdd_5(vdd), .vdd_6(vdd), .vdd_7(vdd), .gnd(gnd), .gnd_1(gnd), 
      .gnd_1_1(gnd), .gnd_2(gnd), .gnd_3(gnd), .gnd_4(gnd), .gnd_5(gnd), 
      .gnd_6(gnd), .gnd_7(gnd));
  muddlib07__mux2_c_1x mux2_c_1_0(.d0(instr_1[16]), .d1(instr[11]), .s(regdst), 
      .y(wa[0]), .vdd(vdd), .gnd(gnd));
  muddlib07__mux2_c_1x mux2_c_1_1(.d0(instr_1[17]), .d1(instr[12]), .s(regdst), 
      .y(wa[1]), .vdd(vdd), .gnd(gnd));
  muddlib07__mux2_c_1x mux2_c_1_2(.d0(instr_1[18]), .d1(instr[13]), .s(regdst), 
      .y(wa[2]), .vdd(vdd), .gnd(gnd));
  wordlib8__mux3_1x_8 pcmux(.d0(aluresult[0:0]), .d0_1(aluresult[1:1]), 
      .d0_2(aluresult[2:2]), .d0_3(aluresult[3:3]), .d0_4(aluresult[4:4]), 
      .d0_5(aluresult[5:5]), .d0_6(aluresult[6:6]), .d0_7(aluresult[7:7]), 
      .d1(aluout[0:0]), .d1_1(aluout[1:1]), .d1_2(aluout[2:2]), 
      .d1_3(aluout[3:3]), .d1_4(aluout[4:4]), .d1_5(aluout[5:5]), 
      .d1_6(aluout[6:6]), .d1_7(aluout[7:7]), .d2({gnd}), .d2_1({gnd}), 
      .d2_2(funct[0:0]), .d2_3(funct_1[1:1]), .d2_4(funct_2[2:2]), 
      .d2_5(funct_3[3:3]), .d2_6(funct_4[4:4]), .d2_7(funct_5[5:5]), 
      .s0(pcsrc[0]), .s1(pcsrc_1[1]), .y(nextpc[0:0]), .y_1(nextpc[1:1]), 
      .y_2(nextpc[2:2]), .y_3(nextpc[3:3]), .y_4(nextpc[4:4]), 
      .y_5(nextpc[5:5]), .y_6(nextpc[6:6]), .y_7(nextpc[7:7]), .vdd(vdd), 
      .vdd_1(vdd), .vdd_32(vdd), .vdd_33(vdd), .vdd_34(vdd), .vdd_35(vdd), 
      .vdd_36(vdd), .vdd_37(vdd), .vdd_38(vdd), .gnd(gnd), .gnd_1(gnd), 
      .gnd_32(gnd), .gnd_33(gnd), .gnd_34(gnd), .gnd_35(gnd), .gnd_36(gnd), 
      .gnd_37(gnd), .gnd_38(gnd));
  wordlib8__flop_1x_8 resreg(.d(aluresult[0:0]), .d_1(aluresult[1:1]), 
      .d_2(aluresult[2:2]), .d_3(aluresult[3:3]), .d_4(aluresult[4:4]), 
      .d_5(aluresult[5:5]), .d_6(aluresult[6:6]), .d_7(aluresult[7:7]), 
      .ph1(ph1), .ph2(ph2), .q(aluout[0:0]), .q_1(aluout[1:1]), 
      .q_2(aluout[2:2]), .q_3(aluout[3:3]), .q_4(aluout[4:4]), 
      .q_5(aluout[5:5]), .q_6(aluout[6:6]), .q_7(aluout[7:7]), .vdd(vdd), 
      .vdd_1(vdd), .vdd_10(vdd), .vdd_11(vdd), .vdd_12(vdd), .vdd_13(vdd), 
      .vdd_14(vdd), .vdd_15(vdd), .vdd_9(vdd), .gnd(gnd), .gnd_1(gnd), 
      .gnd_10(gnd), .gnd_11(gnd), .gnd_12(gnd), .gnd_13(gnd), .gnd_14(gnd), 
      .gnd_15(gnd), .gnd_9(gnd));
  mips8__regramarray_dp rf(.ph2(ph2), .ra1(instr_2[21:21]), 
      .ra1_1(instr_2[22:22]), .ra1_2(instr_2[23:23]), .ra2(instr_1[16:16]), 
      .ra2_1(instr_1[17:17]), .ra2_2(instr_1[18:18]), .regwrite(regwrite), 
      .w(wd[0:0]), .w_1(wd[1:1]), .w_2(wd[2:2]), .w_3(wd[3:3]), .w_4(wd[4:4]), 
      .w_5(wd[5:5]), .w_6(wd[6:6]), .w_7(wd[7:7]), .wa(wa[0:0]), 
      .wa_1(wa[1:1]), .wa_2(wa[2:2]), .r1(rd1[0:0]), .r1_1(rd1[1:1]), 
      .r1_2(rd1[2:2]), .r1_3(rd1[3:3]), .r1_4(rd1[4:4]), .r1_5(rd1[5:5]), 
      .r1_6(rd1[6:6]), .r1_7(rd1[7:7]), .r2(rd2[0:0]), .r2_1(rd2[1:1]), 
      .r2_2(rd2[2:2]), .r2_3(rd2[3:3]), .r2_4(rd2[4:4]), .r2_5(rd2[5:5]), 
      .r2_6(rd2[6:6]), .r2_7(rd2[7:7]), .vdd(vdd), .vdd_1(vdd), .vdd_10(vdd), 
      .vdd_11(vdd), .vdd_12(vdd), .vdd_3(vdd), .vdd_4(vdd), .vdd_5(vdd), 
      .vdd_6(vdd), .vdd_7(vdd), .vdd_8(vdd), .vdd_9(vdd), .gnd(gnd), 
      .gnd_1(gnd), .gnd_10(gnd), .gnd_1_1(gnd), .gnd_2(gnd), .gnd_3(gnd), 
      .gnd_4(gnd), .gnd_5(gnd), .gnd_6(gnd), .gnd_7(gnd), .gnd_8(gnd), 
      .gnd_9(gnd));
  wordlib8__mux2_1x_8 src1mux(.d0(pc[0:0]), .d0_1(pc[1:1]), .d0_2(pc[2:2]), 
      .d0_3(pc[3:3]), .d0_4(pc[4:4]), .d0_5(pc[5:5]), .d0_6(pc[6:6]), 
      .d0_7(pc[7:7]), .d1(a[0:0]), .d1_1(a[1:1]), .d1_2(a[2:2]), .d1_3(a[3:3]), 
      .d1_4(a[4:4]), .d1_5(a[5:5]), .d1_6(a[6:6]), .d1_7(a[7:7]), .s(alusrca), 
      .y(srca[0:0]), .y_1(srca[1:1]), .y_2(srca[2:2]), .y_3(srca[3:3]), 
      .y_4(srca[4:4]), .y_5(srca[5:5]), .y_6(srca[6:6]), .y_7(srca[7:7]), 
      .vdd(vdd), .vdd_1(vdd), .vdd_32(vdd), .vdd_33(vdd), .vdd_34(vdd), 
      .vdd_35(vdd), .vdd_36(vdd), .vdd_37(vdd), .vdd_38(vdd), .gnd(gnd), 
      .gnd_1(gnd), .gnd_32(gnd), .gnd_33(gnd), .gnd_34(gnd), .gnd_35(gnd), 
      .gnd_36(gnd), .gnd_37(gnd), .gnd_38(gnd));
  wordlib8__mux4_1x_8 src2mux(.d0(writedata[0:0]), .d0_1(writedata_1[1:1]), 
      .d0_2(writedata_2[2:2]), .d0_3(writedata_3[3:3]), 
      .d0_4(writedata_4[4:4]), .d0_5(writedata_5[5:5]), 
      .d0_6(writedata_6[6:6]), .d0_7(writedata_7[7:7]), .d1({vdd}), 
      .d1_1({gnd}), .d1_2({gnd}), .d1_3({gnd}), .d1_4({gnd}), .d1_5({gnd}), 
      .d1_6({gnd}), .d1_7({gnd}), .d2(funct[0:0]), .d2_1(funct_1[1:1]), 
      .d2_2(funct_2[2:2]), .d2_3(funct_3[3:3]), .d2_4(funct_4[4:4]), 
      .d2_5(funct_5[5:5]), .d2_6({net_593}), .d2_7({net_594}), .d3({gnd}), 
      .d3_1({gnd}), .d3_2(funct[0:0]), .d3_3(funct_1[1:1]), 
      .d3_4(funct_2[2:2]), .d3_5(funct_3[3:3]), .d3_6(funct_4[4:4]), 
      .d3_7(funct_5[5:5]), .s0(alusrcb[0]), .s1(alusrcb_1[1]), .y(srcb[0:0]), 
      .y_1(srcb[1:1]), .y_2(srcb[2:2]), .y_3(srcb[3:3]), .y_4(srcb[4:4]), 
      .y_5(srcb[5:5]), .y_6(srcb[6:6]), .y_7(srcb[7:7]), .vdd(vdd), 
      .vdd_1(vdd), .vdd_2(vdd), .vdd_34(vdd), .vdd_35(vdd), .vdd_36(vdd), 
      .vdd_37(vdd), .vdd_38(vdd), .vdd_39(vdd), .gnd(gnd), .gnd_1(gnd), 
      .gnd_2(gnd), .gnd_34(gnd), .gnd_35(gnd), .gnd_36(gnd), .gnd_37(gnd), 
      .gnd_38(gnd), .gnd_39(gnd));
  wordlib8__mux2_1x_8 wdmux(.d0(aluout[0:0]), .d0_1(aluout[1:1]), 
      .d0_2(aluout[2:2]), .d0_3(aluout[3:3]), .d0_4(aluout[4:4]), 
      .d0_5(aluout[5:5]), .d0_6(aluout[6:6]), .d0_7(aluout[7:7]), 
      .d1(data[0:0]), .d1_1(data[1:1]), .d1_2(data[2:2]), .d1_3(data[3:3]), 
      .d1_4(data[4:4]), .d1_5(data[5:5]), .d1_6(data[6:6]), .d1_7(data[7:7]), 
      .s(memtoreg), .y(wd[0:0]), .y_1(wd[1:1]), .y_2(wd[2:2]), .y_3(wd[3:3]), 
      .y_4(wd[4:4]), .y_5(wd[5:5]), .y_6(wd[6:6]), .y_7(wd[7:7]), .vdd(vdd), 
      .vdd_1(vdd), .vdd_32(vdd), .vdd_33(vdd), .vdd_34(vdd), .vdd_35(vdd), 
      .vdd_36(vdd), .vdd_37(vdd), .vdd_38(vdd), .gnd(gnd), .gnd_1(gnd), 
      .gnd_32(gnd), .gnd_33(gnd), .gnd_34(gnd), .gnd_35(gnd), .gnd_36(gnd), 
      .gnd_37(gnd), .gnd_38(gnd));
  wordlib8__flop_1x_8 wrdreg(.d(rd2[0:0]), .d_1(rd2[1:1]), .d_2(rd2[2:2]), 
      .d_3(rd2[3:3]), .d_4(rd2[4:4]), .d_5(rd2[5:5]), .d_6(rd2[6:6]), 
      .d_7(rd2[7:7]), .ph1(ph1), .ph2(ph2), .q(writedata[0:0]), 
      .q_1(writedata_1[1:1]), .q_2(writedata_2[2:2]), .q_3(writedata_3[3:3]), 
      .q_4(writedata_4[4:4]), .q_5(writedata_5[5:5]), .q_6(writedata_6[6:6]), 
      .q_7(writedata_7[7:7]), .vdd(vdd), .vdd_1(vdd), .vdd_10(vdd), 
      .vdd_11(vdd), .vdd_12(vdd), .vdd_13(vdd), .vdd_14(vdd), .vdd_15(vdd), 
      .vdd_9(vdd), .gnd(gnd), .gnd_1(gnd), .gnd_10(gnd), .gnd_11(gnd), 
      .gnd_12(gnd), .gnd_13(gnd), .gnd_14(gnd), .gnd_15(gnd), .gnd_9(gnd));
endmodule   /* datapath */

/* Verilog for cell 'cache{sch}' from library 'vlsi' */
/* Created on Sun Nov 16, 2014 18:34:03 */
/* Last revised on Sun Nov 16, 2014 18:47:02 */
/* Written on Fri Nov 21, 2014 14:04:16 by Electric VLSI Design System, version 8.06 */

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

module vlsi__comp2_1x_4(a, b, status);
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
endmodule   /* vlsi__comp2_1x_4 */

module vlsi__decoder16_1x(a, y);
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
endmodule   /* vlsi__decoder16_1x */

module muddlib07__buftri_c_1x(d, en, y);
  input d;
  input en;
  output y;

  supply1 vdd;
  supply0 gnd;
  wire net_1, net_3, net_54, net_6;

  tranif1 nmos_0(gnd, net_3, net_6);
  tranif1 nmos_1(net_3, y, en);
  tranif1 nmos_2(gnd, net_54, en);
  tranif1 nmos_3(gnd, net_6, d);
  tranif0 pmos_0(y, net_1, net_54);
  tranif0 pmos_1(net_1, vdd, net_6);
  tranif0 pmos_2(net_54, vdd, en);
  tranif0 pmos_3(net_6, vdd, d);
endmodule   /* muddlib07__buftri_c_1x */

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

module vlsi__sramcol(clk, in, we, wl, out);
  input clk;
  input in;
  input we;
  input [15:0] wl;
  output out;

  supply1 vdd;
  supply0 gnd;
  wire inv_hi_0_y, net_103, net_115, net_120, net_148, net_149, net_150;
  wire net_151, net_152, net_153, net_154, net_155, net_2, net_33, net_34;
  wire net_59, net_64, net_78, net_8, net_83, net_9, net_98;

  tranif1 nmos_0(net_33, net_9, we);
  tranif1 nmos_1(net_34, net_8, we);
  tranif1 nmos_2(gnd, net_33, in);
  tranif1 nmos_3(gnd, net_34, net_2);
  tranif0 pmos_0(net_8, vdd, clk);
  tranif0 pmos_1(net_9, vdd, clk);
  muddlib07__buftri_c_1x buftri_c_0(.d(wl[15]), .en(clk), .y(net_64));
  muddlib07__buftri_c_1x buftri_c_2(.d(wl[14]), .en(clk), .y(net_59));
  muddlib07__buftri_c_1x buftri_c_3(.d(wl[13]), .en(clk), .y(net_148));
  muddlib07__buftri_c_1x buftri_c_4(.d(wl[12]), .en(clk), .y(net_149));
  muddlib07__buftri_c_1x buftri_c_5(.d(wl[11]), .en(clk), .y(net_83));
  muddlib07__buftri_c_1x buftri_c_6(.d(wl[10]), .en(clk), .y(net_78));
  muddlib07__buftri_c_1x buftri_c_7(.d(wl[9]), .en(clk), .y(net_150));
  muddlib07__buftri_c_1x buftri_c_8(.d(wl[8]), .en(clk), .y(net_151));
  muddlib07__buftri_c_1x buftri_c_9(.d(wl[4]), .en(clk), .y(net_153));
  muddlib07__buftri_c_1x buftri_c_10(.d(wl[3]), .en(clk), .y(net_120));
  muddlib07__buftri_c_1x buftri_c_11(.d(wl[2]), .en(clk), .y(net_115));
  muddlib07__buftri_c_1x buftri_c_12(.d(wl[1]), .en(clk), .y(net_154));
  muddlib07__buftri_c_1x buftri_c_13(.d(wl[0]), .en(clk), .y(net_155));
  muddlib07__buftri_c_1x buftri_c_14(.d(wl[7]), .en(clk), .y(net_103));
  muddlib07__buftri_c_1x buftri_c_15(.d(wl[6]), .en(clk), .y(net_98));
  muddlib07__buftri_c_1x buftri_c_16(.d(wl[5]), .en(clk), .y(net_152));
  muddlib07__inv_1x inv_1x_0(.a(in), .y(net_2));
  vlsi__inv_hi inv_hi_0(.a(net_9), .y(inv_hi_0_y));
  vlsi__inv_hi inv_hi_1(.a(net_8), .y(out));
  muddlib07__srambit srambit_0(.bit_a(net_9), .bit_b(net_8), .word(net_64));
  muddlib07__srambit srambit_1(.bit_a(net_9), .bit_b(net_8), .word(net_59));
  muddlib07__srambit srambit_2(.bit_a(net_9), .bit_b(net_8), .word(net_148));
  muddlib07__srambit srambit_3(.bit_a(net_9), .bit_b(net_8), .word(net_149));
  muddlib07__srambit srambit_4(.bit_a(net_9), .bit_b(net_8), .word(net_83));
  muddlib07__srambit srambit_5(.bit_a(net_9), .bit_b(net_8), .word(net_78));
  muddlib07__srambit srambit_6(.bit_a(net_9), .bit_b(net_8), .word(net_150));
  muddlib07__srambit srambit_7(.bit_a(net_9), .bit_b(net_8), .word(net_151));
  muddlib07__srambit srambit_8(.bit_a(net_9), .bit_b(net_8), .word(net_152));
  muddlib07__srambit srambit_9(.bit_a(net_9), .bit_b(net_8), .word(net_153));
  muddlib07__srambit srambit_10(.bit_a(net_9), .bit_b(net_8), .word(net_120));
  muddlib07__srambit srambit_11(.bit_a(net_9), .bit_b(net_8), .word(net_115));
  muddlib07__srambit srambit_12(.bit_a(net_9), .bit_b(net_8), .word(net_154));
  muddlib07__srambit srambit_13(.bit_a(net_9), .bit_b(net_8), .word(net_155));
  muddlib07__srambit srambit_14(.bit_a(net_9), .bit_b(net_8), .word(net_103));
  muddlib07__srambit srambit_15(.bit_a(net_9), .bit_b(net_8), .word(net_98));
endmodule   /* vlsi__sramcol */



module vlsi__sram(clk, data_in, tag_in, we, wl, data_out, tag_out);
  input clk;
  input [7:0] data_in;
  input [3:0] tag_in;
  input we;
  input [15:0] wl;
  output [7:0] data_out;
  output [3:0] tag_out;

  supply1 vdd;
  supply0 gnd;
  vlsi__sramcol sramcol_0(.clk(clk), .in(tag_in[3]), .we(we), .wl(wl[15:0]), 
      .out(tag_out[3]));
  vlsi__sramcol sramcol_1(.clk(clk), .in(tag_in[2]), .we(we), .wl(wl[15:0]), 
      .out(tag_out[2]));
  vlsi__sramcol sramcol_2(.clk(clk), .in(tag_in[1]), .we(we), .wl(wl[15:0]), 
      .out(tag_out[1]));
  vlsi__sramcol sramcol_3(.clk(clk), .in(tag_in[0]), .we(we), .wl(wl[15:0]), 
      .out(tag_out[0]));
  vlsi__sramcol sramcol_4(.clk(clk), .in(data_in[7]), .we(we), .wl(wl[15:0]), 
      .out(data_out[7]));
  vlsi__sramcol sramcol_5(.clk(clk), .in(data_in[6]), .we(we), .wl(wl[15:0]), 
      .out(data_out[6]));
  vlsi__sramcol sramcol_6(.clk(clk), .in(data_in[5]), .we(we), .wl(wl[15:0]), 
      .out(data_out[5]));
  vlsi__sramcol sramcol_7(.clk(clk), .in(data_in[4]), .we(we), .wl(wl[15:0]), 
      .out(data_out[4]));
  vlsi__sramcol sramcol_8(.clk(clk), .in(data_in[0]), .we(we), .wl(wl[15:0]), 
      .out(data_out[0]));
  vlsi__sramcol sramcol_9(.clk(clk), .in(data_in[3]), .we(we), .wl(wl[15:0]), 
      .out(data_out[3]));
  vlsi__sramcol sramcol_10(.clk(clk), .in(data_in[2]), .we(we), .wl(wl[15:0]), 
      .out(data_out[2]));
  vlsi__sramcol sramcol_11(.clk(clk), .in(data_in[1]), .we(we), .wl(wl[15:0]), 
      .out(data_out[1]));
endmodule   /* vlsi__sram */

module cache(address, clk, data_in, we, data_out, hit);
  input [7:0] address;
  input clk;
  input [7:0] data_in;
  input we;
  output [7:0] data_out;
  output hit;

  supply1 vdd;
  supply0 gnd;
  wire [15:0] net_4;
  wire [3:0] net_8;

  //vlsi__comp2_1x_4 comp2_1x_0(.a(address[7:4]), .b(net_8[3:0]), .status(hit));
  assign hit = (address[7:4]===net_8)?1:0;
  
  vlsi__decoder16_1x decoder1_0(.a(address[3:0]), .y(net_4[15:0]));
  /*vlsi__sram sram_0(.clk(clk), .data_in(data_in[7:0]), .tag_in(address[7:4]), 
      .we(we), .wl(net_4[15:0]), .data_out(data_out[7:0]), 
      .tag_out(net_8[3:0]));
	*/  
	  sram sram_0(clk, we, net_4, address[7:4], data_in, net_8, data_out);
	  
endmodule   /* cache */