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

//assign in = {op,state};
//assign {aluop, branch, pcwrite, irwrite, alusrcb, pcsrc, regdst, regwrite, iord, memtoreg, alusrca, memwrite, memread, nextstate} = out;
//			2		1		1		4			2		2		1		1		1		1		1		1			1		4