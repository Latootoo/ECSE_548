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
/*
  always@(negedge clk)
    begin
      if(memwrite) begin
        assert(adr == 76 & writedata == 7)
          $display("Simulation completely successful");
        else $error("Simulation failed");
        $stop;
      end
    end
	*/
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
   
   controller_plabased  cont(clk, reset, hit, op, funct, zero, memread, memwrite, cachewrite, morc, morw,
                    alusrca, memtoreg, iord, pcen, regwrite, regdst,
                    pcsrc, alusrcb, alucontrol, irwrite);
   datapath    #(WIDTH, REGBITS) 
               dp(clk, reset, dpdata, alusrca, memtoreg, iord, pcen,
                  regwrite, regdst, pcsrc, alusrcb, irwrite, alucontrol,
                  zero, op, funct, adr, writedata);
				  
	mux2       #(WIDTH)  cachemux(memdata, writedata, morw, cachedata);	
  	cache idcache(clk, cachewrite, adr, cachedata, cacheout, hit);
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

  /*
        12'b??????00000?: out <= 27'b000000000000000000010000000;
      12'b??????100000: out <= 27'b000100010100000000100001100;
      12'b??????100001: out <= 27'b000100010100000000100001011;
      12'b??????00001?: out <= 27'b000000000000000000010001000;
      12'b??????100010: out <= 27'b000100100100000000100010100;
      12'b??????100011: out <= 27'b000100100100000000100010001;
      12'b??????00010?: out <= 27'b000000000000000000010010000;
      12'b??????100100: out <= 27'b000101000100000000100011000;
      12'b??????100101: out <= 27'b000101000100000000100011001;
      12'b??????00011?: out <= 27'b000110000100000000100100000;

  */
  
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
	  
      default:        	out <= 27'b000000000000000000000000000;
	  
	  /*
	  12'b??????00000?: out <= 27'b000100010100000000100001000;
      12'b??????00001?: out <= 27'b000100100100000000100010000;
      12'b??????00010?: out <= 27'b000101000100000000100011000;
      12'b??????00011?: out <= 27'b000110000100000000100100000;
      12'b10000000100?: out <= 27'b000000001100000000000101000;
      12'b10100000100?: out <= 27'b000000001100000000000101000;
      12'b00000000100?: out <= 27'b000000001100000000001001000;
      12'b00010000100?: out <= 27'b000000001100000000001011000;
      12'b00001000100?: out <= 27'b000000001100000000001100000;
      12'b10000000101?: out <= 27'b000000001000000010000110000;
      12'b10100000101?: out <= 27'b000000001000000010001000000;
      12'b??????00110?: out <= 27'b000000000000001000100111000;
      12'b??????00111?: out <= 27'b000000000000010100000000000;
      12'b??????01000?: out <= 27'b000000000000001001000000000;
      12'b??????01001?: out <= 27'b100000000000000010001010000;
      12'b??????01010?: out <= 27'b000000000000110000000000000;
	  12'b??????01011?: out <= 27'b011000000001000010000000000;
      12'b??????01100?: out <= 27'b000100000010000000000000000;
      default:        	out <= 27'b000000000000000000000000000;
	  */
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

module datapath #(parameter WIDTH = 8, REGBITS = 3)
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

module cache(clk, we, addr, data_in, data_out, hit);
	input logic clk;
	input logic we;
	input logic [7:0] addr;
	input logic [7:0] data_in;
	output logic [7:0] data_out;
	output logic hit;
	
	logic [15:0] wl;
	decoder dec(addr[3:0], wl);
	
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