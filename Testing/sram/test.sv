module testbench();
    logic clk;
	logic [3:0] expected_tag;
	logic [7:0] expected_data;
	
	logic [40:0] vectors[100:0], currentvec;
    logic [3:0] vectornum, errors;

	logic we;
	logic [15:0] wl;
	logic [7:0] data_in;
	logic [3:0] tag_in;
	logic [7:0] data_out;
	logic [3:0] tag_out;
	
    // device under test
	sram dut(clk, we, wl, tag_in, data_in, tag_out, data_out);
			
	// read test file and initialize test
    initial begin
        $readmemb("test-out.txt", vectors);
        vectornum = 0; errors = 0;
    end
    // generate clock
    always begin
        clk = 1; #5; clk = 0; #5;
    end
    // apply test
    always @(posedge clk) begin
        currentvec = vectors[vectornum];
		we = currentvec[0];
		wl = currentvec[16:1];
		data_in = currentvec[24:17];
		tag_in = currentvec[28:25];
		
        if (currentvec[0] === 1'bx) begin
			$display("Completed %d tests with %d errors.", 
				vectornum, errors);
            $stop;
        end
    end
    // check errors
    always @(negedge clk) begin
		expected_data = currentvec[36:29];
		expected_tag = currentvec[40:37];
		if (tag_out !== expected_tag) begin
            $display("At %h, tag output mismatches as %h (%h expected)", 
                        vectornum, tag_out, expected_tag);
            errors = errors + 1;
        end
		if (data_out !== expected_data) begin
            $display("At %h, data output mismatches as %h (%h expected)", 
                        vectornum, data_out, expected_data);
            errors = errors + 1;
        end
		
        vectornum = vectornum + 1;
    end
endmodule /* testbench */

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
		
	assign	tag_out = we ? tag_in : ramT[addr];
	assign	data_out = we ? data_in : ramD[addr];
	
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
