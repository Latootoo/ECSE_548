
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
	  default: out = 16'bx;
	  endcase
endmodule